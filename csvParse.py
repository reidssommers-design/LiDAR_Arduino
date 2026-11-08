#!/usr/bin/env python3
"""Pick a scan CSV from a folder, parse it, print a summary and plot it.

Usage:  python3 csvParse.py [FOLDER]        (default: this script's folder)

Reads files written by lidar_live.py:  scan,step,theta_deg,dist_cm,strength
Can also be imported:  from csvParse import load_scan
"""
import sys, csv
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

# Same filtering and geometry as lidar_live.py
R_OFF   = 0.02     # lens offset from rotation axis (m)
MIN_AMP = 100      # TF-Luna: below this, distance is unreliable
MAX_CM  = 800

COLUMNS = ["scan", "step", "theta_deg", "dist_cm", "strength"]


def load_scan(path):
    """Parse a scan CSV into a dict of numpy arrays (one per column), plus
    'valid' (bool mask using the live-plot filter) and 'x', 'y' in metres."""
    rows = []
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            raise ValueError("file is empty")
        if header != COLUMNS:
            raise ValueError(f"unexpected header {header!r}, expected {COLUMNS}")
        for i, r in enumerate(reader, start=2):
            try:
                rows.append((int(r[0]), int(r[1]), float(r[2]), int(r[3]), int(r[4])))
            except (ValueError, IndexError):
                print(f"  skipping bad line {i}: {r}")

    a = np.array(rows, dtype=float).reshape(-1, 5)
    d = {name: a[:, i] for i, name in enumerate(COLUMNS)}
    for k in ("scan", "step", "dist_cm", "strength"):
        d[k] = d[k].astype(int)

    d["valid"] = ((d["strength"] >= MIN_AMP) & (d["strength"] != 65535)
                  & (d["dist_cm"] > 0) & (d["dist_cm"] <= MAX_CM))
    t = np.radians(d["theta_deg"])
    r = R_OFF + d["dist_cm"] / 100.0
    d["x"], d["y"] = r * np.cos(t), r * np.sin(t)
    return d


def count_rows(path):
    with open(path) as f:
        return max(sum(1 for _ in f) - 1, 0)


def choose_file(folder):
    files = sorted(Path(folder).glob("*.csv"))
    if not files:
        sys.exit(f"no .csv files in {Path(folder).resolve()}")

    print(f"\nCSV files in {Path(folder).resolve()}:\n")
    for i, p in enumerate(files, 1):
        print(f"  {i:>2}) {p.name:<32} {count_rows(p):>7} rows  {p.stat().st_size/1024:8.1f} KB")

    while True:
        choice = input(f"\nSelect a file [1-{len(files)}] (Enter = newest, q = quit): ").strip().lower()
        if choice in ("q", "quit"):
            sys.exit(0)
        if choice == "":
            return max(files, key=lambda p: p.stat().st_mtime)
        if choice.isdigit() and 1 <= int(choice) <= len(files):
            return files[int(choice) - 1]
        print("  invalid choice")


def summarize(d, path):
    n, nv = len(d["scan"]), int(d["valid"].sum())
    print(f"\n{path.name}: {n} samples, {nv} valid ({100*nv/max(n,1):.0f}%)")
    print(f"\n  {'scan':>4} {'samples':>8} {'valid':>6} {'min cm':>7} {'max cm':>7} {'mean cm':>8}")
    for s in np.unique(d["scan"]):
        m = d["scan"] == s
        v = m & d["valid"]
        dist = d["dist_cm"][v]
        stats = (f"{dist.min():>7} {dist.max():>7} {dist.mean():>8.1f}"
                 if dist.size else f"{'-':>7} {'-':>7} {'-':>8}")
        print(f"  {s:>4} {int(m.sum()):>8} {int(v.sum()):>6} {stats}")


def plot(d, path):
    v = d["valid"]
    if not v.any():
        print("\nno valid points to plot")
        return
    fig, ax = plt.subplots(figsize=(7, 7))
    sc = ax.scatter(d["x"][v], d["y"][v], c=d["scan"][v], s=4, cmap="viridis")
    if len(np.unique(d["scan"][v])) > 1:
        fig.colorbar(sc, ax=ax, label="scan #", shrink=0.8)
    ax.plot(0, 0, "r+", ms=12)
    ax.set_aspect("equal"); ax.grid(True)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title(path.name)
    plt.show()


def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else Path(__file__).parent
    path = choose_file(folder)
    try:
        d = load_scan(path)
    except ValueError as e:
        sys.exit(f"{path.name}: {e}")
    if len(d["scan"]) == 0:
        sys.exit(f"{path.name} has no data rows")
    summarize(d, path)
    plot(d, path)


if __name__ == "__main__":
    main()
