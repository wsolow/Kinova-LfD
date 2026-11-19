#!/usr/bin/env python3
"""
Recursively finds all "*_wrist_rgb.npy" files under a directory, center-crops
each image to a square, resizes it to a target size, and saves the result.

Usage:
    python crop_resize_wrist_rgb.py /path/to/data
    python crop_resize_wrist_rgb.py /path/to/data --size 360 --inplace
    python crop_resize_wrist_rgb.py /path/to/data --size 360 --out-dir /path/to/output --dry-run
"""

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("This script requires OpenCV. Install it with: pip install opencv-python")


def center_crop_square(img: np.ndarray) -> np.ndarray:
    """Center-crop an HxWxC (or HxW) array to a square using the shorter side."""
    h, w = img.shape[:2]
    side = min(h, w)
    top = (h - side) // 2
    left = (w - side) // 2
    return img[top : top + side, left : left + side]


def resize(img: np.ndarray, size: int) -> np.ndarray:
    """Resize an image to size x size using area interpolation (good for downscaling)."""
    return cv2.resize(img, (size, size), interpolation=cv2.INTER_AREA)


def process_array(img: np.ndarray, size: int) -> np.ndarray:
    """Crop to square then resize. Handles both single images and batches of images."""
    if img.ndim == 3:
        # Single image: (H, W, C)
        cropped = center_crop_square(img)
        return resize(cropped, size)
    elif img.ndim == 4:
        # Batch of images: (N, H, W, C)
        out = np.empty((img.shape[0], size, size, img.shape[3]), dtype=img.dtype)
        for i in range(img.shape[0]):
            cropped = center_crop_square(img[i])
            out[i] = resize(cropped, size)
        return out
    else:
        raise ValueError(f"Unexpected array shape {img.shape}, expected HxWxC or NxHxWxC")


def find_wrist_rgb_files(root: Path):
    return sorted(root.rglob("*_wrist_rgb.npy"))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--directory", type=Path, help="Root directory to search recursively")
    parser.add_argument("--size", type=int, default=360, help="Output size (square), default 360")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="If set, mirrors the input directory structure here instead of modifying files in place",
    )
    parser.add_argument(
        "--inplace",
        action="store_true",
        help="Overwrite the original .npy files (default behavior if --out-dir is not set)",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="List files that would be processed without writing anything"
    )
    args = parser.parse_args()

    root = args.directory
    if not root.is_dir():
        sys.exit(f"Error: {root} is not a directory")

    files = find_wrist_rgb_files(root)
    if not files:
        print(f"No '*_wrist_rgb.npy' files found under {root}")
        return

    print(f"Found {len(files)} file(s) to process.")

    for i, path in enumerate(files, 1):
        rel = path.relative_to(root)
        if args.dry_run:
            print(f"[{i}/{len(files)}] Would process: {rel}")
            continue

        try:
            img = np.load(path)
        except Exception as e:
            print(f"[{i}/{len(files)}] SKIP (failed to load) {rel}: {e}")
            continue

        try:
            result = process_array(img, args.size)
        except Exception as e:
            print(f"[{i}/{len(files)}] SKIP (failed to process) {rel} shape={img.shape}: {e}")
            continue

        if args.out_dir:
            out_path = args.out_dir / rel
            out_path.parent.mkdir(parents=True, exist_ok=True)
        else:
            out_path = path  # in-place overwrite (default)

        np.save(out_path, result)
        print(f"[{i}/{len(files)}] {rel}: {img.shape} -> {result.shape}  saved to {out_path}")

    print("Done.")


if __name__ == "__main__":
    main()
