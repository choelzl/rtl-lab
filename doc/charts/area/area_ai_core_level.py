#!/usr/bin/env python3

# -----------------------------------------------------------------------------
# Author: Simone Machetti
# -----------------------------------------------------------------------------

import os
import csv
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

_DIR = os.path.dirname(os.path.abspath(__file__))
CSV  = os.path.join(_DIR, '..', '..', 'data', 'area', 'area.csv')

C_BAS_4   = "#97665b"
C_SQR_4   = "#005f73"
C_ALPHA_4 = "#97bdc5"

def load():
    with open(CSV) as f:
        return {r['design']: float(r['area_um2']) for r in csv.DictReader(f)}

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('ARRAY_SIZE', type=int)
    args = parser.parse_args()
    n = args.ARRAY_SIZE

    d = load()

    bas_4x8       = n*n * d['Baseline 4x8']
    bas_8x8       = n*n * d['Baseline 8x8']
    sqr_4x8_pe    = n*n * d['Square 4x8 SC']
    sqr_4x8_alpha = n   * (4 * d['Square 4x8 Alpha Squared'] + 3 * d['Square 4x8 Alpha'])
    sqr_4x8       = sqr_4x8_pe + sqr_4x8_alpha
    sqr_8x8_pe    = n*n * d['Square 8x8']
    sqr_8x8_alpha = n   * (4 * d['Square 8x8 Alpha Squared'])
    sqr_8x8       = sqr_8x8_pe + sqr_8x8_alpha

    x     = np.arange(4)
    width = 0.5

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.bar(x[0], bas_4x8,       width, color=C_BAS_4)
    ax.bar(x[1], bas_8x8,       width, color=C_SQR_4)
    ax.bar(x[2], sqr_4x8_pe,    width, color=C_SQR_4)
    ax.bar(x[2], sqr_4x8_alpha, width, bottom=sqr_4x8_pe, color=C_ALPHA_4)
    ax.bar(x[3], sqr_8x8_pe,    width, color=C_SQR_4)
    ax.bar(x[3], sqr_8x8_alpha, width, bottom=sqr_8x8_pe, color=C_ALPHA_4)

    ref = bas_4x8
    def pct(v): return (v - ref) / ref * 100.0

    ax.text(x[0], bas_4x8 * 1.01, f"{bas_4x8:.0f} µm²",    ha="center", va="bottom", fontsize=9)
    ax.text(x[1], bas_8x8 * 1.01, f"{pct(bas_8x8):+.1f}%", ha="center", va="bottom", fontsize=9)
    ax.text(x[2], sqr_4x8 * 1.01, f"{pct(sqr_4x8):+.1f}%", ha="center", va="bottom", fontsize=9)
    ax.text(x[3], sqr_8x8 * 1.01, f"{pct(sqr_8x8):+.1f}%", ha="center", va="bottom", fontsize=9)

    ax.set_ylim(0, max(bas_4x8, bas_8x8, sqr_4x8, sqr_8x8) * 1.40)
    ax.set_xticks(x)
    ax.set_xticklabels(['Baseline 4x8', 'Baseline 8x8', 'Square 4x8', 'Square 8x8'])
    ax.set_ylabel("Area (µm²)")
    ax.set_title(f"Area Analysis: AI-Core Level ({n}x{n})")
    ax.legend(handles=[
        Patch(color=C_BAS_4,   label=f"PE Baseline 4x8 ×{n*n}"),
        Patch(color=C_SQR_4,   label=f"PE ×{n*n}"),
        Patch(color=C_ALPHA_4, label=f"Alpha ×{n}"),
    ])
    plt.tight_layout()
    plt.savefig(f"area_ai_core_level_{n}.png", dpi=200)

if __name__ == "__main__":
    main()
