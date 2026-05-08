#!/usr/bin/env python3

# -----------------------------------------------------------------------------
# Author: Simone Machetti
# -----------------------------------------------------------------------------

import os
import csv
import numpy as np
import matplotlib.pyplot as plt

_DIR = os.path.dirname(os.path.abspath(__file__))
CSV  = os.path.join(_DIR, '..', '..', 'data', 'area', 'area.csv')

C_AREA = "#005f73"

def load():
    with open(CSV) as f:
        rows = list(csv.DictReader(f))
    names = [r['design'] for r in rows]
    areas = np.array([float(r['area_um2']) for r in rows])
    return names, areas

def main():
    names, areas = load()
    x     = np.arange(len(names))
    width = 0.5

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.bar(x, areas, width, color=C_AREA)

    for i, a in enumerate(areas):
        ax.text(x[i], a * 1.01, f"{a:.1f} µm²", ha="center", va="bottom", fontsize=9)

    ax.set_ylim(0, max(areas) * 1.20)
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=15, ha='right')
    ax.set_ylabel("Area (µm²)")
    ax.set_title("Area Analysis: PE Level")
    plt.tight_layout()
    plt.savefig("area_pe_level.png", dpi=200)

if __name__ == "__main__":
    main()
