#!/usr/bin/env python3
"""
PINN Training Script for AIRHOUND Trajectory Prediction.

Trains a Physics-Informed Neural Network (PINN) for predicting target
trajectories during detection dropout. The PINN is a simple MLP with a
physics loss term that constrains predicted velocities to be physically
plausible (|v| <= v_max).

Architecture (from SPIE roadmap):
    Input:  [x, y, z, vx, vy, vz, dt]  (current state + prediction horizon)
    Output: [x', y', z', vx', vy', vz'] (predicted state at t + dt)

Physics Loss:
    L = L_data + lambda_phys * L_physics
    L_data   = MSE(predicted, ground_truth)
    L_physics = mean(max(0, |v_pred| - v_max)^2)

Usage:
    python3 scripts/train_pinn.py --data data/pinn_training/trajectories.csv
    python3 scripts/train_pinn.py --data data/pinn_training/synthetic.csv --epochs 500 --lr 1e-3

Run with rf-detr venv:
    /home/rylan/dev/research/EPPL/code/rf-detr-training/.venv/bin/python3 scripts/train_pinn.py ...
"""

import argparse
import csv
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from torch.utils.data import Dataset, DataLoader, random_split
except ImportError:
    print("PyTorch required. Install with: pip install torch")
    sys.exit(1)


# ============================================================================
# Model
# ============================================================================

class TrajectoryPINN(nn.Module):
    """
    Physics-Informed Neural Network for trajectory prediction.

    Input:  [x, y, z, vx, vy, vz, dt]  (7 features)
    Output: [x', y', z', vx', vy', vz'] (6 outputs)

    Architecture: 3-layer MLP with ReLU activations.
    Kept small for Jetson Orin inference latency (<1ms target).
    """

    def __init__(self, hidden_dim: int = 128, num_layers: int = 3, dropout: float = 0.0):
        super().__init__()

        layers = []
        in_dim = 7  # [x, y, z, vx, vy, vz, dt]

        for i in range(num_layers):
            out_dim = hidden_dim
            layers.append(nn.Linear(in_dim, out_dim))
            layers.append(nn.ReLU())
            if dropout > 0:
                layers.append(nn.Dropout(dropout))
            in_dim = out_dim

        layers.append(nn.Linear(in_dim, 6))  # [x', y', z', vx', vy', vz']
        self.net = nn.Sequential(*layers)

        # Initialize weights (Xavier uniform for stable training)
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.xavier_uniform_(m.weight)
                nn.init.zeros_(m.bias)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


# ============================================================================
# Physics Loss
# ============================================================================

def physics_loss(pred_state: torch.Tensor, v_max: float = 10.0) -> torch.Tensor:
    """
    Physics constraint: predicted velocity magnitude must not exceed v_max.

    Parameters
    ----------
    pred_state : torch.Tensor
        Predicted states [..., 6] where last 3 are [vx', vy', vz']
    v_max : float
        Maximum allowable velocity magnitude (m/s). Default: 10.0

    Returns
    -------
    torch.Tensor
        Scalar penalty loss
    """
    vx = pred_state[..., 3]
    vy = pred_state[..., 4]
    vz = pred_state[..., 5]

    v_magnitude = torch.sqrt(vx**2 + vy**2 + vz**2 + 1e-8)
    violation = torch.clamp(v_magnitude - v_max, min=0.0)

    return torch.mean(violation**2)


# ============================================================================
# Dataset
# ============================================================================

class TrajectoryDataset(Dataset):
    """
    Dataset for PINN training from CSV trajectory data.

    Loads CSV in PINN_DATA_FORMAT.md format: [t, x, y, z, vx, vy, vz]
    Creates (input, target) pairs where:
        input  = [x_i, y_i, z_i, vx_i, vy_i, vz_i, dt]
        target = [x_j, y_j, z_j, vx_j, vy_j, vz_j]
    for all pairs (i, j) where j > i and dt = t_j - t_i <= max_dt.
    """

    def __init__(
        self,
        csv_path: str,
        max_dt: float = 1.0,
        min_dt: float = 0.03,
        stride: int = 1,
    ):
        self.data = self._load_csv(csv_path)
        self.inputs, self.targets = self._build_pairs(max_dt, min_dt, stride)

    def _load_csv(self, csv_path: str) -> np.ndarray:
        """Load trajectory CSV: [t, x, y, z, vx, vy, vz]."""
        rows = []
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append([
                    float(row['t']),
                    float(row['x']), float(row['y']), float(row['z']),
                    float(row['vx']), float(row['vy']), float(row['vz']),
                ])
        data = np.array(rows, dtype=np.float32)
        print(f"Loaded {len(data)} points from {csv_path}")
        return data

    def _build_pairs(
        self, max_dt: float, min_dt: float, stride: int
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Build (input, target) pairs from trajectory points."""
        inputs = []
        targets = []
        n = len(self.data)

        for i in range(0, n, stride):
            t_i = self.data[i, 0]
            state_i = self.data[i, 1:7]  # [x, y, z, vx, vy, vz]

            for j in range(i + 1, n):
                t_j = self.data[j, 0]
                dt = t_j - t_i

                if dt < min_dt:
                    continue
                if dt > max_dt:
                    break

                state_j = self.data[j, 1:7]

                # input: [x, y, z, vx, vy, vz, dt]
                inp = np.concatenate([state_i, [dt]])
                inputs.append(inp)
                targets.append(state_j)

        inputs = np.array(inputs, dtype=np.float32)
        targets = np.array(targets, dtype=np.float32)
        print(f"Built {len(inputs)} training pairs (max_dt={max_dt}s, min_dt={min_dt}s)")
        return inputs, targets

    def __len__(self) -> int:
        return len(self.inputs)

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        return (
            torch.from_numpy(self.inputs[idx]),
            torch.from_numpy(self.targets[idx]),
        )


# ============================================================================
# Normalization
# ============================================================================

@dataclass
class NormStats:
    """Input/output normalization statistics."""
    input_mean: np.ndarray
    input_std: np.ndarray
    target_mean: np.ndarray
    target_std: np.ndarray

    def save(self, path: str):
        np.savez(
            path,
            input_mean=self.input_mean,
            input_std=self.input_std,
            target_mean=self.target_mean,
            target_std=self.target_std,
        )

    @classmethod
    def load(cls, path: str) -> "NormStats":
        d = np.load(path)
        return cls(
            input_mean=d['input_mean'],
            input_std=d['input_std'],
            target_mean=d['target_mean'],
            target_std=d['target_std'],
        )


def compute_norm_stats(dataset: TrajectoryDataset) -> NormStats:
    """Compute mean/std for input and target normalization."""
    inputs = dataset.inputs
    targets = dataset.targets

    # Clamp std to avoid division by zero
    eps = 1e-6
    return NormStats(
        input_mean=inputs.mean(axis=0),
        input_std=np.maximum(inputs.std(axis=0), eps),
        target_mean=targets.mean(axis=0),
        target_std=np.maximum(targets.std(axis=0), eps),
    )


def normalize_batch(
    inputs: torch.Tensor,
    targets: torch.Tensor,
    stats: NormStats,
    device: torch.device,
) -> Tuple[torch.Tensor, torch.Tensor]:
    """Normalize a batch using precomputed stats."""
    in_mean = torch.from_numpy(stats.input_mean).to(device)
    in_std = torch.from_numpy(stats.input_std).to(device)
    tgt_mean = torch.from_numpy(stats.target_mean).to(device)
    tgt_std = torch.from_numpy(stats.target_std).to(device)

    inputs_norm = (inputs - in_mean) / in_std
    targets_norm = (targets - tgt_mean) / tgt_std

    return inputs_norm, targets_norm


def denormalize_output(
    pred_norm: torch.Tensor,
    stats: NormStats,
    device: torch.device,
) -> torch.Tensor:
    """Denormalize model output back to physical units."""
    tgt_mean = torch.from_numpy(stats.target_mean).to(device)
    tgt_std = torch.from_numpy(stats.target_std).to(device)
    return pred_norm * tgt_std + tgt_mean


# ============================================================================
# Training
# ============================================================================

def train(
    model: TrajectoryPINN,
    train_loader: DataLoader,
    val_loader: DataLoader,
    norm_stats: NormStats,
    epochs: int = 200,
    lr: float = 1e-3,
    lambda_phys: float = 0.1,
    v_max: float = 10.0,
    device: torch.device = torch.device('cpu'),
    save_dir: str = 'models/pinn',
) -> dict:
    """
    Train the PINN with combined data + physics loss.

    Returns dict with training history.
    """
    model = model.to(device)
    optimizer = optim.Adam(model.parameters(), lr=lr, weight_decay=1e-5)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode='min', factor=0.5, patience=20
    )

    mse_loss = nn.MSELoss()

    os.makedirs(save_dir, exist_ok=True)
    best_val_loss = float('inf')
    history = {'train_loss': [], 'val_loss': [], 'phys_loss': [], 'data_loss': []}

    print(f"\nTraining for {epochs} epochs on {device}")
    print(f"  lambda_phys={lambda_phys}, v_max={v_max} m/s")
    print(f"  Train batches: {len(train_loader)}, Val batches: {len(val_loader)}")
    print()

    for epoch in range(1, epochs + 1):
        # --- Train ---
        model.train()
        train_total = 0.0
        train_data = 0.0
        train_phys = 0.0
        n_batches = 0

        for inputs, targets in train_loader:
            inputs, targets = inputs.to(device), targets.to(device)

            # Normalize
            inputs_n, targets_n = normalize_batch(inputs, targets, norm_stats, device)

            # Forward
            pred_n = model(inputs_n)

            # Data loss (normalized space)
            loss_data = mse_loss(pred_n, targets_n)

            # Physics loss (physical space — denormalize predictions first)
            pred_phys = denormalize_output(pred_n, norm_stats, device)
            loss_phys = physics_loss(pred_phys, v_max=v_max)

            # Combined loss
            loss = loss_data + lambda_phys * loss_phys

            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()

            train_total += loss.item()
            train_data += loss_data.item()
            train_phys += loss_phys.item()
            n_batches += 1

        train_total /= n_batches
        train_data /= n_batches
        train_phys /= n_batches

        # --- Validate ---
        model.eval()
        val_total = 0.0
        n_val = 0
        with torch.no_grad():
            for inputs, targets in val_loader:
                inputs, targets = inputs.to(device), targets.to(device)
                inputs_n, targets_n = normalize_batch(inputs, targets, norm_stats, device)
                pred_n = model(inputs_n)
                loss_data = mse_loss(pred_n, targets_n)
                pred_phys = denormalize_output(pred_n, norm_stats, device)
                loss_phys = physics_loss(pred_phys, v_max=v_max)
                val_total += (loss_data + lambda_phys * loss_phys).item()
                n_val += 1

        val_total /= max(n_val, 1)
        scheduler.step(val_total)

        # Record history
        history['train_loss'].append(train_total)
        history['val_loss'].append(val_total)
        history['data_loss'].append(train_data)
        history['phys_loss'].append(train_phys)

        # Save best
        if val_total < best_val_loss:
            best_val_loss = val_total
            torch.save(model.state_dict(), os.path.join(save_dir, 'pinn_best.pth'))

        # Log
        if epoch % 10 == 0 or epoch == 1:
            print(
                f"Epoch {epoch:4d}/{epochs} | "
                f"Train: {train_total:.6f} (data={train_data:.6f}, phys={train_phys:.6f}) | "
                f"Val: {val_total:.6f} | "
                f"LR: {optimizer.param_groups[0]['lr']:.1e}"
            )

    # Save final model
    torch.save(model.state_dict(), os.path.join(save_dir, 'pinn_final.pth'))

    # Save normalization stats (needed for inference)
    norm_stats.save(os.path.join(save_dir, 'norm_stats.npz'))

    # Save training config
    config = {
        'hidden_dim': model.net[0].in_features,  # approximate
        'v_max': v_max,
        'lambda_phys': lambda_phys,
        'epochs': epochs,
        'best_val_loss': best_val_loss,
    }
    config_path = os.path.join(save_dir, 'config.txt')
    with open(config_path, 'w') as f:
        for k, v in config.items():
            f.write(f"{k}: {v}\n")

    print(f"\nTraining complete. Best val loss: {best_val_loss:.6f}")
    print(f"Models saved to {save_dir}/")

    return history


# ============================================================================
# Evaluation
# ============================================================================

def evaluate_model(
    model: TrajectoryPINN,
    val_loader: DataLoader,
    norm_stats: NormStats,
    device: torch.device,
    v_max: float = 10.0,
):
    """Print evaluation metrics on validation set."""
    model.eval()

    all_preds = []
    all_targets = []

    with torch.no_grad():
        for inputs, targets in val_loader:
            inputs, targets = inputs.to(device), targets.to(device)
            inputs_n, _ = normalize_batch(inputs, targets, norm_stats, device)
            pred_n = model(inputs_n)
            pred = denormalize_output(pred_n, norm_stats, device)
            all_preds.append(pred.cpu().numpy())
            all_targets.append(targets.cpu().numpy())

    preds = np.concatenate(all_preds)
    targets = np.concatenate(all_targets)

    # Position error
    pos_err = np.linalg.norm(preds[:, :3] - targets[:, :3], axis=1)
    vel_err = np.linalg.norm(preds[:, 3:6] - targets[:, 3:6], axis=1)

    # Velocity constraint violations
    v_mag = np.linalg.norm(preds[:, 3:6], axis=1)
    violations = (v_mag > v_max).sum()

    print(f"\n--- Evaluation ---")
    print(f"  Position RMSE:  {np.sqrt(np.mean(pos_err**2)):.4f} m")
    print(f"  Position Mean:  {pos_err.mean():.4f} m")
    print(f"  Position Max:   {pos_err.max():.4f} m")
    print(f"  Velocity RMSE:  {np.sqrt(np.mean(vel_err**2)):.4f} m/s")
    print(f"  Velocity Max:   {vel_err.max():.4f} m/s")
    print(f"  V constraint violations: {violations}/{len(v_mag)} ({100*violations/len(v_mag):.1f}%)")


# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description='Train AIRHOUND PINN')
    parser.add_argument('--data', required=True, help='Path to trajectory CSV')
    parser.add_argument('--save-dir', default='models/pinn', help='Output directory')
    parser.add_argument('--epochs', type=int, default=200)
    parser.add_argument('--lr', type=float, default=1e-3)
    parser.add_argument('--batch-size', type=int, default=256)
    parser.add_argument('--hidden-dim', type=int, default=128)
    parser.add_argument('--num-layers', type=int, default=3)
    parser.add_argument('--dropout', type=float, default=0.0)
    parser.add_argument('--lambda-phys', type=float, default=0.1,
                        help='Weight for physics loss')
    parser.add_argument('--v-max', type=float, default=10.0,
                        help='Max velocity constraint (m/s)')
    parser.add_argument('--max-dt', type=float, default=1.0,
                        help='Max prediction horizon (s)')
    parser.add_argument('--min-dt', type=float, default=0.03,
                        help='Min prediction horizon (s)')
    parser.add_argument('--val-split', type=float, default=0.15)
    parser.add_argument('--device', default='auto',
                        choices=['auto', 'cpu', 'cuda'])
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()

    # Device
    if args.device == 'auto':
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    else:
        device = torch.device(args.device)

    # Seed
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    # Load data
    dataset = TrajectoryDataset(
        args.data, max_dt=args.max_dt, min_dt=args.min_dt
    )

    if len(dataset) == 0:
        print("ERROR: No training pairs generated. Check data file.")
        sys.exit(1)

    # Split
    n_val = max(1, int(len(dataset) * args.val_split))
    n_train = len(dataset) - n_val
    train_set, val_set = random_split(dataset, [n_train, n_val])

    train_loader = DataLoader(train_set, batch_size=args.batch_size, shuffle=True)
    val_loader = DataLoader(val_set, batch_size=args.batch_size)

    # Normalization stats (from full dataset)
    norm_stats = compute_norm_stats(dataset)

    # Model
    model = TrajectoryPINN(
        hidden_dim=args.hidden_dim,
        num_layers=args.num_layers,
        dropout=args.dropout,
    )
    n_params = sum(p.numel() for p in model.parameters())
    print(f"Model: {args.num_layers}-layer MLP, {args.hidden_dim} hidden, {n_params} params")

    # Train
    history = train(
        model, train_loader, val_loader, norm_stats,
        epochs=args.epochs,
        lr=args.lr,
        lambda_phys=args.lambda_phys,
        v_max=args.v_max,
        device=device,
        save_dir=args.save_dir,
    )

    # Evaluate best model
    model.load_state_dict(
        torch.load(os.path.join(args.save_dir, 'pinn_best.pth'), weights_only=True)
    )
    model = model.to(device)
    evaluate_model(model, val_loader, norm_stats, device, v_max=args.v_max)


if __name__ == '__main__':
    main()
