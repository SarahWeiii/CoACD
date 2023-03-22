#!/usr/bin/env python3
try:
    import trimesh
except ModuleNotFoundError:
    print("trimesh is required. Please install with `pip install trimesh`")
    exit(1)
import sys
import os
import argparse
import coacd

if __name__ == "__main__":

    def usage():
        print("Usage: coacd input_file output_file")
        exit(1)

    parser = argparse.ArgumentParser()
    parser.add_argument("input_file", type=str)
    parser.add_argument("output_file", type=str)
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--threshold", type=float, default=0.05)
    parser.add_argument("--max-convex-hull", type=int, default=-1)
    parser.add_argument("--preprocess", type=bool, default=True)
    parser.add_argument("--preprocess-resolution", type=int, default=30)
    parser.add_argument("--pca", type=bool, default=False)
    parser.add_argument("--merge", type=bool, default=True)
    parser.add_argument("--mcts-max-depth", type=int, default=3)
    parser.add_argument("--mcts-nodes", type=int, default=20)
    parser.add_argument("--mcts_iterations", type=int, default=150)
    parser.add_argument("--seed", type=int, default=0)

    args = parser.parse_args()
    input_file = args.input_file
    output_file = args.output_file

    if not os.path.isfile(input_file):
        print(input_file, "is not a file")
        exit(1)

    if args.quiet:
        coacd.set_log_level("off")

    mesh = trimesh.load(input_file)
    imesh = coacd.Mesh()
    imesh.vertices = mesh.vertices
    imesh.indices = mesh.faces
    parts = coacd.run_coacd(
        imesh,
        mcts_nodes=args.mcts_nodes,
        threshold=args.threshold,
        max_convex_hull=args.max_convex_hull,
        resolution=3000,
        seed=args.seed,
        preprocess=args.preprocess,
        preprocess_resolution=args.preprocess_resolution,
        pca=args.pca,
        merge=args.merge,
        mcts_iterations=args.mcts_iterations,
        mcts_max_depth=args.mcts_max_depth,
    )
    mesh_parts = [
        trimesh.Trimesh(p.vertices, p.indices.reshape((-1, 3))) for p in parts
    ]
    trimesh.save(mesh_parts, output_file)