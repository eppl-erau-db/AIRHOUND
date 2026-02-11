#!/usr/bin/env python3
"""
Export trained PINN model to ONNX for Jetson Orin deployment.

The ONNX model can be further converted to TensorRT for minimal
inference latency on the Jetson:
    trtexec --onnx=pinn.onnx --saveEngine=pinn.engine --fp16

Usage:
    python3 scripts/export_pinn_onnx.py --model models/pinn/pinn_best.pth
    python3 scripts/export_pinn_onnx.py --model models/pinn/pinn_best.pth \
        --output models/pinn/pinn.onnx --hidden-dim 128 --num-layers 3
"""

import argparse
import os
import sys

try:
    import torch
    import torch.nn as nn
except ImportError:
    print("PyTorch required. Install with: pip install torch")
    sys.exit(1)

import numpy as np


class TrajectoryPINN(nn.Module):
    """Mirror of the training script model architecture."""

    def __init__(self, hidden_dim: int = 128, num_layers: int = 3):
        super().__init__()
        layers = []
        in_dim = 7  # [x, y, z, vx, vy, vz, dt]
        for _ in range(num_layers):
            layers.extend([nn.Linear(in_dim, hidden_dim), nn.ReLU()])
            in_dim = hidden_dim
        layers.append(nn.Linear(in_dim, 6))  # [x', y', z', vx', vy', vz']
        self.net = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


def main():
    parser = argparse.ArgumentParser(description='Export PINN to ONNX')
    parser.add_argument('--model', required=True, help='Path to .pth model')
    parser.add_argument('--output', default=None, help='Output .onnx path')
    parser.add_argument('--hidden-dim', type=int, default=128)
    parser.add_argument('--num-layers', type=int, default=3)
    parser.add_argument('--opset', type=int, default=17)
    parser.add_argument('--verify', action='store_true',
                        help='Verify ONNX output matches PyTorch')
    args = parser.parse_args()

    if args.output is None:
        args.output = args.model.replace('.pth', '.onnx')

    # Load model
    model = TrajectoryPINN(hidden_dim=args.hidden_dim, num_layers=args.num_layers)
    state_dict = torch.load(args.model, map_location='cpu', weights_only=True)
    model.load_state_dict(state_dict)
    model.eval()

    n_params = sum(p.numel() for p in model.parameters())
    print(f"Loaded model: {args.num_layers} layers, {args.hidden_dim} hidden, {n_params} params")

    # Dummy input: [x, y, z, vx, vy, vz, dt]
    dummy = torch.randn(1, 7)

    # Export
    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
    torch.onnx.export(
        model,
        dummy,
        args.output,
        opset_version=args.opset,
        input_names=['state_dt'],
        output_names=['predicted_state'],
        dynamic_axes={
            'state_dt': {0: 'batch'},
            'predicted_state': {0: 'batch'},
        },
    )

    file_size = os.path.getsize(args.output) / 1024
    print(f"Exported to {args.output} ({file_size:.1f} KB, opset {args.opset})")

    # Verify
    if args.verify:
        try:
            import onnxruntime as ort

            session = ort.InferenceSession(args.output)
            input_name = session.get_inputs()[0].name

            # Compare outputs
            test_input = torch.randn(5, 7)
            with torch.no_grad():
                pt_out = model(test_input).numpy()

            ort_out = session.run(None, {input_name: test_input.numpy()})[0]

            max_diff = np.abs(pt_out - ort_out).max()
            print(f"Verification: max difference = {max_diff:.2e}")
            if max_diff < 1e-5:
                print("PASS: ONNX output matches PyTorch")
            else:
                print("WARN: Outputs differ (may be floating point precision)")
        except ImportError:
            print("onnxruntime not installed, skipping verification")

    # Print TensorRT conversion command
    print(f"\nTo convert to TensorRT (run on Jetson):")
    print(f"  trtexec --onnx={args.output} "
          f"--saveEngine={args.output.replace('.onnx', '.engine')} "
          f"--fp16 --workspace=256")


if __name__ == '__main__':
    main()
