#!/usr/bin/env python3

# -----------------------------------------------------------------------------
# Author: Simone Machetti
# -----------------------------------------------------------------------------

import os
import csv
import numpy as np
import matplotlib.pyplot as plt

_DIR = os.path.dirname(os.path.abspath(__file__))
CSV  = os.path.join(_DIR, '..', '..', 'data', 'power', 'power.csv')

C_PWR = "#005f73"

def load():
    with open(CSV) as f:
        rows = list(csv.DictReader(f))
    names  = [r['design'] for r in rows]
    powers = np.array([float(r['power_mw']) for r in rows])
    return names, powers

def main():
    names, powers = load()
    x     = np.arange(len(names))
    width = 0.5

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.bar(x, powers, width, color=C_PWR)

    for i, p in enumerate(powers):
        ax.text(x[i], p * 1.01, f"{p:.2f} mW", ha="center", va="bottom", fontsize=9)

    ax.set_ylim(0, max(powers) * 1.20)
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=15, ha='right')
    ax.set_ylabel("Power (mW)")
    ax.set_title("Power Analysis: PE Level")
    plt.tight_layout()
    plt.savefig("pwr_pe_level.png", dpi=200)

if __name__ == "__main__":
    main()
