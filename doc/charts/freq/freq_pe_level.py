#!/usr/bin/env python3

# -----------------------------------------------------------------------------
# Author: Simone Machetti
# -----------------------------------------------------------------------------

import os
import csv
import numpy as np
import matplotlib.pyplot as plt

_DIR = os.path.dirname(os.path.abspath(__file__))
CSV  = os.path.join(_DIR, '..', '..', 'data', 'freq', 'freq.csv')

C_FREQ = "#005f73"

def load():
    with open(CSV) as f:
        rows = list(csv.DictReader(f))
    names = [r['design'] for r in rows]
    freqs = np.array([float(r['freq_mhz']) / 1000.0 for r in rows])  # MHz → GHz
    return names, freqs

def main():
    names, freqs = load()
    x     = np.arange(len(names))
    width = 0.5

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.bar(x, freqs, width, color=C_FREQ)

    for i, f in enumerate(freqs):
        ax.text(x[i], f * 1.01, f"{f:.3f} GHz", ha="center", va="bottom", fontsize=9)

    ax.set_ylim(0, max(freqs) * 1.20)
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=15, ha='right')
    ax.set_ylabel("Freq Max (GHz)")
    ax.set_title("Frequency Analysis: PE Level")
    plt.tight_layout()
    plt.savefig("freq_pe_level.png", dpi=200)

if __name__ == "__main__":
    main()
