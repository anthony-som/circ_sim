#!/usr/bin/env python3
"""One-shot script to decimate STL meshes for Gazebo performance.

Usage:
    python3 decimate_meshes.py <mesh_dir> [target_faces]

Examples:
    python3 decimate_meshes.py ../src/robots/asimov/meshes 5000
    python3 decimate_meshes.py ../src/robots/mini_arm_ros2/meshes 10000
"""

import argparse
import os
import pymeshlab


def decimate(mesh_dir, default_target, targets=None, keep_as_is=None):
    targets = targets or {}
    keep_as_is = keep_as_is or []
    out_dir = os.path.join(mesh_dir, 'decimated')
    os.makedirs(out_dir, exist_ok=True)

    for fname in sorted(os.listdir(mesh_dir)):
        if not fname.upper().endswith('.STL'):
            continue

        src = os.path.join(mesh_dir, fname)
        dst = os.path.join(out_dir, fname)

        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(src)
        original = ms.current_mesh().face_number()

        if fname in keep_as_is:
            ms.save_current_mesh(dst)
            print(f'{fname}: {original} faces -> kept as-is')
            continue

        target = targets.get(fname, default_target)

        if original <= target:
            ms.save_current_mesh(dst)
            print(f'{fname}: {original} faces -> already below target ({target}), kept as-is')
            continue

        ms.apply_filter('meshing_decimation_quadric_edge_collapse',
                         targetfacenum=target,
                         qualitythr=0.3,
                         preserveboundary=True,
                         preservenormal=True,
                         optimalplacement=True)

        final = ms.current_mesh().face_number()
        ms.save_current_mesh(dst)
        reduction = (1 - final / original) * 100
        print(f'{fname}: {original} -> {final} faces ({reduction:.1f}% reduction)')

    print(f'\nDone! Decimated meshes saved to: {out_dir}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Decimate STL meshes')
    parser.add_argument('mesh_dir', help='Path to directory containing STL files')
    parser.add_argument('target', type=int, nargs='?', default=5000,
                        help='Default target face count (default: 5000)')
    args = parser.parse_args()

    decimate(os.path.abspath(args.mesh_dir), args.target)
