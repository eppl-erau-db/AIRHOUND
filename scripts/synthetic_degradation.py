#!/usr/bin/env python3
"""
Synthetic Degradation Pipeline for AIRHOUND.

This module implements various image degradation techniques to test
detector robustness under challenging conditions, as required for the
SPIE "Machine Learning from Challenging Data" paper.

Degradation Types:
- Motion blur: Simulates fast target/camera movement
- Gaussian noise: Simulates low-light/sensor noise
- Occlusion: Simulates partial target visibility
- Combined: Multiple degradations applied together

Usage:
    from synthetic_degradation import DegradationPipeline
    
    pipeline = DegradationPipeline()
    degraded_img = pipeline.apply(img, degradation_type="motion_blur", severity=0.5)

Author: AIRHOUND Team (EPPL Lab, Embry-Riddle)
Date: January 2026
"""

from __future__ import annotations

import cv2
import numpy as np
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple, List, Callable
from pathlib import Path


class DegradationType(Enum):
    """Available degradation types."""
    MOTION_BLUR = "motion_blur"
    GAUSSIAN_NOISE = "gaussian_noise"
    OCCLUSION = "occlusion"
    LOW_LIGHT = "low_light"
    COMBINED = "combined"


@dataclass
class DegradationParams:
    """Parameters for a specific degradation."""
    severity: float  # 0.0 to 1.0
    seed: Optional[int] = None  # For reproducibility


class DegradationPipeline:
    """
    Applies synthetic degradations to images for robustness testing.
    
    Parameters
    ----------
    seed : Optional[int]
        Random seed for reproducibility.
    """
    
    def __init__(self, seed: Optional[int] = None):
        self.seed = seed
        if seed is not None:
            np.random.seed(seed)
        
        # Degradation function registry
        self._degradations: dict[str, Callable] = {
            DegradationType.MOTION_BLUR.value: self._apply_motion_blur,
            DegradationType.GAUSSIAN_NOISE.value: self._apply_gaussian_noise,
            DegradationType.OCCLUSION.value: self._apply_occlusion,
            DegradationType.LOW_LIGHT.value: self._apply_low_light,
            DegradationType.COMBINED.value: self._apply_combined,
        }
    
    def apply(
        self,
        image: np.ndarray,
        degradation_type: str,
        severity: float = 0.5,
        **kwargs,
    ) -> np.ndarray:
        """
        Apply degradation to an image.
        
        Parameters
        ----------
        image : np.ndarray
            Input BGR image.
        degradation_type : str
            Type of degradation (motion_blur, gaussian_noise, occlusion, etc.)
        severity : float
            Severity level from 0.0 (none) to 1.0 (maximum).
        **kwargs
            Additional parameters for specific degradation types.
            
        Returns
        -------
        np.ndarray
            Degraded BGR image.
        """
        if degradation_type not in self._degradations:
            raise ValueError(f"Unknown degradation type: {degradation_type}")
        
        severity = np.clip(severity, 0.0, 1.0)
        return self._degradations[degradation_type](image, severity, **kwargs)
    
    def _apply_motion_blur(
        self,
        image: np.ndarray,
        severity: float,
        angle: Optional[float] = None,
        **kwargs,
    ) -> np.ndarray:
        """
        Apply motion blur to simulate fast movement.
        
        Parameters
        ----------
        image : np.ndarray
            Input image.
        severity : float
            Blur severity (0.0 = no blur, 1.0 = severe blur).
        angle : Optional[float]
            Blur direction in degrees. If None, random angle.
        """
        # Kernel size scales with severity: 1 to 31 pixels
        kernel_size = int(1 + severity * 30)
        if kernel_size % 2 == 0:
            kernel_size += 1  # Ensure odd kernel size
        
        if kernel_size <= 1:
            return image.copy()
        
        # Random angle if not specified
        if angle is None:
            angle = np.random.uniform(0, 180)
        
        # Create motion blur kernel
        kernel = np.zeros((kernel_size, kernel_size))
        kernel[kernel_size // 2, :] = 1.0 / kernel_size
        
        # Rotate kernel to desired angle
        center = (kernel_size // 2, kernel_size // 2)
        rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
        kernel = cv2.warpAffine(
            kernel, 
            rotation_matrix, 
            (kernel_size, kernel_size),
            flags=cv2.INTER_LINEAR,
        )
        
        # Normalize kernel
        kernel = kernel / kernel.sum()
        
        # Apply blur
        blurred = cv2.filter2D(image, -1, kernel)
        return blurred
    
    def _apply_gaussian_noise(
        self,
        image: np.ndarray,
        severity: float,
        **kwargs,
    ) -> np.ndarray:
        """
        Apply Gaussian noise to simulate sensor noise.
        
        Parameters
        ----------
        image : np.ndarray
            Input image.
        severity : float
            Noise severity (0.0 = no noise, 1.0 = severe noise).
        """
        # Standard deviation scales with severity: 0 to 50
        sigma = severity * 50.0
        
        if sigma <= 0:
            return image.copy()
        
        # Generate noise
        noise = np.random.normal(0, sigma, image.shape).astype(np.float32)
        
        # Add noise and clip
        noisy = image.astype(np.float32) + noise
        noisy = np.clip(noisy, 0, 255).astype(np.uint8)
        
        return noisy
    
    def _apply_occlusion(
        self,
        image: np.ndarray,
        severity: float,
        num_patches: Optional[int] = None,
        patch_type: str = "rectangle",
        **kwargs,
    ) -> np.ndarray:
        """
        Apply random occlusions to simulate partial visibility.
        
        Parameters
        ----------
        image : np.ndarray
            Input image.
        severity : float
            Occlusion severity (0.0 = no occlusion, 1.0 = heavy occlusion).
        num_patches : Optional[int]
            Number of occlusion patches. If None, scales with severity.
        patch_type : str
            Type of patch: "rectangle", "circle", or "random".
        """
        if severity <= 0:
            return image.copy()
        
        h, w = image.shape[:2]
        result = image.copy()
        
        # Number of patches scales with severity
        if num_patches is None:
            num_patches = int(1 + severity * 5)
        
        # Patch size scales with severity (5% to 25% of image dimension)
        min_size = int(min(h, w) * 0.05)
        max_size = int(min(h, w) * (0.05 + severity * 0.20))
        
        for _ in range(num_patches):
            # Random patch size
            patch_h = np.random.randint(min_size, max(min_size + 1, max_size))
            patch_w = np.random.randint(min_size, max(min_size + 1, max_size))
            
            # Random position
            x = np.random.randint(0, max(1, w - patch_w))
            y = np.random.randint(0, max(1, h - patch_h))
            
            # Random color (usually dark for occlusions)
            if np.random.random() < 0.7:
                color = tuple(int(c) for c in np.random.randint(0, 50, 3))
            else:
                color = tuple(int(c) for c in np.random.randint(0, 255, 3))
            
            if patch_type == "rectangle" or (patch_type == "random" and np.random.random() < 0.5):
                cv2.rectangle(result, (x, y), (x + patch_w, y + patch_h), color, -1)
            else:
                # Circle/ellipse
                center = (x + patch_w // 2, y + patch_h // 2)
                axes = (patch_w // 2, patch_h // 2)
                cv2.ellipse(result, center, axes, 0, 0, 360, color, -1)
        
        return result
    
    def _apply_low_light(
        self,
        image: np.ndarray,
        severity: float,
        **kwargs,
    ) -> np.ndarray:
        """
        Simulate low-light conditions (reduced brightness + noise).
        
        Parameters
        ----------
        image : np.ndarray
            Input image.
        severity : float
            Severity (0.0 = normal light, 1.0 = very dark).
        """
        if severity <= 0:
            return image.copy()
        
        # Reduce brightness (gamma > 1 darkens)
        gamma = 1.0 + severity * 2.0  # 1.0 to 3.0
        inv_gamma = 1.0 / gamma
        
        # Apply gamma correction to darken
        table = np.array([
            ((i / 255.0) ** inv_gamma) * 255
            for i in range(256)
        ]).astype(np.uint8)
        
        darkened = cv2.LUT(image, table)
        
        # Add sensor noise (more visible in low light)
        noise_sigma = severity * 25.0
        if noise_sigma > 0:
            noise = np.random.normal(0, noise_sigma, darkened.shape).astype(np.float32)
            darkened = np.clip(darkened.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        
        return darkened
    
    def _apply_combined(
        self,
        image: np.ndarray,
        severity: float,
        degradations: Optional[List[str]] = None,
        **kwargs,
    ) -> np.ndarray:
        """
        Apply multiple degradations in sequence.
        
        Parameters
        ----------
        image : np.ndarray
            Input image.
        severity : float
            Base severity for all degradations.
        degradations : Optional[List[str]]
            List of degradation types to apply. 
            Default: motion_blur + gaussian_noise
        """
        if degradations is None:
            degradations = ["motion_blur", "gaussian_noise"]
        
        result = image.copy()
        
        # Apply each degradation with slightly reduced severity
        individual_severity = severity * 0.7
        
        for deg_type in degradations:
            if deg_type != "combined":  # Avoid recursion
                result = self.apply(result, deg_type, individual_severity)
        
        return result
    
    def get_severity_levels(self, num_levels: int = 5) -> List[float]:
        """Get evenly spaced severity levels for testing."""
        return [i / (num_levels - 1) for i in range(num_levels)]
    
    def preview_degradations(
        self,
        image: np.ndarray,
        output_dir: Path,
        severities: List[float] = [0.0, 0.25, 0.5, 0.75, 1.0],
    ) -> None:
        """
        Generate preview images for all degradation types at various severities.
        
        Parameters
        ----------
        image : np.ndarray
            Sample input image.
        output_dir : Path
            Directory to save preview images.
        severities : List[float]
            Severity levels to preview.
        """
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        degradation_types = [
            DegradationType.MOTION_BLUR.value,
            DegradationType.GAUSSIAN_NOISE.value,
            DegradationType.OCCLUSION.value,
            DegradationType.LOW_LIGHT.value,
            DegradationType.COMBINED.value,
        ]
        
        for deg_type in degradation_types:
            for severity in severities:
                degraded = self.apply(image, deg_type, severity)
                filename = f"{deg_type}_severity_{severity:.2f}.jpg"
                cv2.imwrite(str(output_dir / filename), degraded)
        
        print(f"Saved {len(degradation_types) * len(severities)} preview images to {output_dir}")


def batch_degrade_dataset(
    input_dir: Path,
    output_dir: Path,
    degradation_type: str,
    severity: float,
    max_images: Optional[int] = None,
    seed: int = 42,
) -> int:
    """
    Apply degradation to a batch of images.
    
    Parameters
    ----------
    input_dir : Path
        Directory containing input images.
    output_dir : Path
        Directory to save degraded images.
    degradation_type : str
        Type of degradation to apply.
    severity : float
        Degradation severity (0.0 to 1.0).
    max_images : Optional[int]
        Maximum number of images to process.
    seed : int
        Random seed for reproducibility.
        
    Returns
    -------
    int
        Number of images processed.
    """
    input_dir = Path(input_dir)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    pipeline = DegradationPipeline(seed=seed)
    
    image_files = list(input_dir.glob("*.jpg")) + list(input_dir.glob("*.png"))
    if max_images:
        image_files = image_files[:max_images]
    
    processed = 0
    for img_path in image_files:
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        
        degraded = pipeline.apply(img, degradation_type, severity)
        
        output_path = output_dir / img_path.name
        cv2.imwrite(str(output_path), degraded)
        processed += 1
        
        if processed % 100 == 0:
            print(f"Processed {processed}/{len(image_files)} images")
    
    return processed


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Synthetic Degradation Pipeline")
    parser.add_argument("--input", type=str, required=True, help="Input image or directory")
    parser.add_argument("--output", type=str, required=True, help="Output directory")
    parser.add_argument("--type", type=str, default="motion_blur",
                        choices=["motion_blur", "gaussian_noise", "occlusion", "low_light", "combined"],
                        help="Degradation type")
    parser.add_argument("--severity", type=float, default=0.5, help="Severity (0.0-1.0)")
    parser.add_argument("--preview", action="store_true", help="Generate preview of all types")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    
    args = parser.parse_args()
    
    input_path = Path(args.input)
    output_path = Path(args.output)
    
    pipeline = DegradationPipeline(seed=args.seed)
    
    if args.preview:
        # Single image preview mode
        if input_path.is_file():
            img = cv2.imread(str(input_path))
            pipeline.preview_degradations(img, output_path)
        else:
            # Use first image from directory
            first_img = next(input_path.glob("*.jpg"), None)
            if first_img:
                img = cv2.imread(str(first_img))
                pipeline.preview_degradations(img, output_path)
    else:
        # Batch mode
        if input_path.is_file():
            img = cv2.imread(str(input_path))
            degraded = pipeline.apply(img, args.type, args.severity)
            output_path.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(output_path / input_path.name), degraded)
            print(f"Saved degraded image to {output_path / input_path.name}")
        else:
            count = batch_degrade_dataset(
                input_path, output_path, args.type, args.severity, seed=args.seed
            )
            print(f"Processed {count} images")
