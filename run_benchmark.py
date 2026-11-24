#!/usr/bin/env python3
"""Benchmark the CoACD command line tool and check that its output is stable.

Runs the `main` executable on a set of meshes with a fixed seed and reports,
for each mesh, the median wall time over several runs, how much the run times
vary, the number of convex hulls and a hash of the output OBJ. Given a second
executable, it alternates runs of the two and reports the speedup and whether
the outputs are identical, so a change can be checked for both performance and
unchanged results.

Examples:
    # Time one build on the bundled examples.
    python run_benchmark.py --bin build/main

    # Compare two builds (e.g. main vs. a branch).
    python run_benchmark.py --baseline build-main/main --candidate build/main

Only the Python standard library is needed.
"""

import argparse
import glob
import hashlib
import json
import os
import shlex
import statistics
import subprocess
import sys
import tempfile
import time


def run_once(binary, mesh, extra_args, timeout):
    """Runs `binary` on `mesh`; returns (seconds, hull count, output hash)."""
    with tempfile.TemporaryDirectory() as tmp:
        output = os.path.join(tmp, "output.obj")
        cmd = [binary, "-i", mesh, "-o", output, "-ro", os.path.join(tmp, "remesh.obj")]
        cmd += extra_args
        start = time.perf_counter()
        try:
            result = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                                    timeout=timeout)
        except subprocess.TimeoutExpired:
            sys.exit(f"error: {' '.join(cmd)} timed out after {timeout} s")
        seconds = time.perf_counter() - start
        if result.returncode != 0 or not os.path.exists(output):
            sys.exit(f"error: {' '.join(cmd)} failed:\n{result.stderr.decode(errors='replace')}")
        with open(output, "rb") as f:
            data = f.read()
    hulls = sum(1 for line in data.splitlines() if line.startswith(b"o "))
    return seconds, hulls, hashlib.sha256(data).hexdigest()


def benchmark(binaries, mesh, extra_args, runs, timeout):
    """Runs each binary `runs` times on `mesh`, alternating between them so that
    changes in machine load affect all of them alike."""
    times = {key: [] for key in binaries}
    outputs = {}
    for _ in range(runs):
        for key, binary in binaries.items():
            seconds, hulls, digest = run_once(binary, mesh, extra_args, timeout)
            if key in outputs and outputs[key][1] != digest:
                print(f"warning: {os.path.basename(mesh)}: output differs between runs of {binary}",
                      file=sys.stderr)
            outputs[key] = (hulls, digest)
            times[key].append(seconds)
    results = {}
    for key in binaries:
        median = statistics.median(times[key])
        # Half the range of the run times, relative to the median.
        spread = (max(times[key]) - min(times[key])) / 2 / median * 100
        results[key] = {"seconds": median, "spread_percent": spread, "times": times[key],
                        "hulls": outputs[key][0], "sha256": outputs[key][1]}
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bin", "--baseline", dest="baseline", required=True,
                        help="CoACD `main` executable to benchmark (the baseline when comparing)")
    parser.add_argument("--candidate", help="second `main` executable to compare against the baseline")
    parser.add_argument("--meshes", nargs="+", help="input OBJ files (default: examples/*.obj)")
    parser.add_argument("--runs", type=int, default=3,
                        help="runs per mesh and executable (default: 3); the median time is reported")
    parser.add_argument("--seed", type=int, default=0, help="seed passed to CoACD (default: 0)")
    parser.add_argument("--args", default="", help='extra arguments for `main`, e.g. "-t 0.1 -nm"')
    parser.add_argument("--timeout", type=float, default=3600, help="timeout per run, in seconds")
    parser.add_argument("--json", help="also write the results to this JSON file")
    args = parser.parse_args()
    if args.runs < 1:
        parser.error("--runs must be at least 1")

    root = os.path.dirname(os.path.abspath(__file__))
    meshes = args.meshes or sorted(glob.glob(os.path.join(root, "examples", "*.obj")))
    if not meshes:
        sys.exit("error: no meshes found")
    names = [os.path.splitext(os.path.basename(mesh))[0] for mesh in meshes]
    if len(set(names)) != len(names):
        sys.exit("error: mesh file names must be unique")
    extra_args = ["-s", str(args.seed)] + shlex.split(args.args)
    binaries = {"baseline": args.baseline}
    if args.candidate:
        binaries["candidate"] = args.candidate

    # The +-% columns are half the range of the run times, relative to the median.
    results = {}
    if args.candidate:
        print(f"{'mesh':<20} {'baseline s':>11} {'+-%':>5} {'candidate s':>12} {'+-%':>5} "
              f"{'speedup':>8} {'hulls':>6}  identical")
    else:
        print(f"{'mesh':<20} {'seconds':>9} {'+-%':>5} {'hulls':>6}  sha256")
    for mesh in meshes:
        name = os.path.splitext(os.path.basename(mesh))[0]
        results[name] = benchmark(binaries, mesh, extra_args, args.runs, args.timeout)
        base = results[name]["baseline"]
        if args.candidate:
            cand = results[name]["candidate"]
            same = base["sha256"] == cand["sha256"]
            hulls = str(base["hulls"]) if same else f"{base['hulls']}/{cand['hulls']}"
            print(f"{name:<20} {base['seconds']:>11.2f} {base['spread_percent']:>5.1f} "
                  f"{cand['seconds']:>12.2f} {cand['spread_percent']:>5.1f} "
                  f"{base['seconds'] / cand['seconds']:>7.2f}x {hulls:>6}  {'yes' if same else 'NO'}")
        else:
            print(f"{name:<20} {base['seconds']:>9.2f} {base['spread_percent']:>5.1f} "
                  f"{base['hulls']:>6}  {base['sha256'][:16]}")
        sys.stdout.flush()

    if args.candidate:
        total_base = sum(r["baseline"]["seconds"] for r in results.values())
        total_cand = sum(r["candidate"]["seconds"] for r in results.values())
        print(f"{'total':<20} {total_base:>11.2f} {'':>5} {total_cand:>12.2f} {'':>5} "
              f"{total_base / total_cand:>7.2f}x")
    if args.json:
        with open(args.json, "w") as f:
            json.dump({"binaries": binaries, "args": extra_args, "runs": args.runs,
                       "results": results}, f, indent=2)
    if args.candidate and any(r["baseline"]["sha256"] != r["candidate"]["sha256"]
                              for r in results.values()):
        sys.exit(1)


if __name__ == "__main__":
    main()
