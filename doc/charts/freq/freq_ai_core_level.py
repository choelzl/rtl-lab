#!/usr/bin/env python3

# -----------------------------------------------------------------------------
# Author: Simone Machetti
# -----------------------------------------------------------------------------

import os
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

_DIR = os.path.dirname(os.path.abspath(__file__))
CSV  = os.path.join(_DIR, '..', '..', 'data', 'freq', 'freq.csv')

C_BAS_4 = "#97665b"
C_SQR_4 = "#005f73"

def load():
    with open(CSV) as f:
        return {r['design']: float(r['freq_mhz']) / 1000.0 for r in csv.DictReader(f)}  # GHz

def main():
    d = load()

    bas_4x8 = d['Baseline 4x8']
    bas_8x8 = d['Baseline 8x8']
    sqr_4x8 = min(d['Square 4x8 SC'], d['Square 4x8 Alpha'], d['Square 4x8 Alpha Squared'])
    sqr_8x8 = min(d['Square 8x8'],    d['Square 8x8 Alpha Squared'])

    x     = np.arange(4)
    width = 0.5

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.bar(x[0], bas_4x8, width, color=C_BAS_4)
    ax.bar(x[1], bas_8x8, width, color=C_SQR_4)
    ax.bar(x[2], sqr_4x8, width, color=C_SQR_4)
    ax.bar(x[3], sqr_8x8, width, color=C_SQR_4)

    ref = bas_4x8
    def pct(v): return (v - ref) / ref * 100.0

    ax.text(x[0], bas_4x8 * 1.01, f"{bas_4x8:.3f} GHz",    ha="center", va="bottom", fontsize=9)
    ax.text(x[1], bas_8x8 * 1.01, f"{pct(bas_8x8):+.1f}%", ha="center", va="bottom", fontsize=9)
    ax.text(x[2], sqr_4x8 * 1.01, f"{pct(sqr_4x8):+.1f}%", ha="center", va="bottom", fontsize=9)
    ax.text(x[3], sqr_8x8 * 1.01, f"{pct(sqr_8x8):+.1f}%", ha="center", va="bottom", fontsize=9)

    ax.set_ylim(0, max(bas_4x8, bas_8x8, sqr_4x8, sqr_8x8) * 1.40)
    ax.set_xticks(x)
    ax.set_xticklabels(['Baseline 4x8', 'Baseline 8x8', 'Square 4x8', 'Square 8x8'])
    ax.set_ylabel("f_max (GHz)")
    ax.set_title("Frequency Analysis: AI-Core Level")
    ax.legend(handles=[
        Patch(color=C_BAS_4, label="PE Baseline 4x8"),
        Patch(color=C_SQR_4, label="PE (f_max = min over components)"),
    ])
    plt.tight_layout()
    plt.savefig("freq_ai_core_level.png", dpi=200)

if __name__ == "__main__":
    main()
