"""
Generate Terrain Files
=======================
Run this once after cloning to generate:
  1. terrain_8bit.png  — 8-bit normalized heightmap from Unity export
  2. mars_colored.png  — Mars texture baked onto elevation for Ogre1

Usage:
  python3 scripts/generate_terrain.py \
    --heightmap path/to/terrain.png \
    --texture   path/to/mars_texture.png \
    --output    models/mars_terrain/materials/textures/
"""

import argparse
import numpy as np
from PIL import Image
import os


def normalize_heightmap(src_path, dst_path):
    img = Image.open(src_path)
    arr = np.array(img, dtype=np.float32)
    print(f"Input: {src_path}")
    print(f"  Size: {img.size}, Mode: {img.mode}")
    print(f"  Min: {arr.min():.0f}, Max: {arr.max():.0f}, "
          f"Mean: {arr.mean():.1f}")

    arr_norm = (arr - arr.min()) / (arr.max() - arr.min()) * 255
    out = Image.fromarray(arr_norm.astype(np.uint8), mode='L')
    out.save(dst_path)
    print(f"Saved: {dst_path} ({out.size}, {out.mode})")
    return arr_norm / 255.0


def bake_texture(hmap_norm, texture_path, dst_path, size=513):
    texture = Image.open(texture_path).convert('RGB').resize((size, size))
    tex_arr = np.array(texture, dtype=np.float32)

    # Resize heightmap to match texture
    hmap_resized = np.array(
        Image.fromarray((hmap_norm * 255).astype(np.uint8)).resize((size, size)),
        dtype=np.float32
    ) / 255.0

    # Blend: darker in crater (low elev), brighter on rim (high elev)
    elevation = hmap_resized[:, :, np.newaxis]
    blended   = (tex_arr * (0.3 + 0.7 * elevation)).astype(np.uint8)

    Image.fromarray(blended).save(dst_path)
    print(f"Saved: {dst_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--heightmap', required=True,
                        help='Path to Unity-exported heightmap PNG')
    parser.add_argument('--texture', required=True,
                        help='Path to Mars surface texture PNG')
    parser.add_argument('--output', default='models/mars_terrain/materials/textures/',
                        help='Output directory')
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    # 1. Normalize heightmap to 8-bit
    hmap_norm = normalize_heightmap(
        args.heightmap,
        os.path.join(args.output, 'terrain_8bit.png')
    )

    # Check dimensions
    h, w = hmap_norm.shape
    valid_sizes = [2**n + 1 for n in range(7, 12)]  # 129,257,513,1025,2049
    if h not in valid_sizes or w not in valid_sizes:
        print(f"\n[WARN] Size {w}x{h} may not be valid for Ogre2.")
        print(f"  Valid sizes: {valid_sizes}")
        print(f"  Current size is {'OK' if h == w == 513 else 'NOT IDEAL'} "
              f"for Ogre1.")

    # 2. Bake texture onto elevation
    bake_texture(
        hmap_norm,
        args.texture,
        os.path.join(args.output, 'mars_colored.png'),
        size=min(h, w)
    )

    print("\nDone. Copy files to ~/.gz/models/mars_terrain/materials/textures/")


if __name__ == '__main__':
    main()
