#!/usr/bin/env python3
"""Compare wind-test logs (e.g. ADRC vs PX4 baseline) phase by phase.
Usage: python3 analysis/compare_wind.py logs/adrc_A.csv logs/adrc_B.csv
       (no arguments: the two newest wind-test logs)
Phases follow wind_profile(): calm 0-5 s, steady 5-15 s, calm 15-25 s, gusts 25-40 s."""
import glob, os, sys
import numpy as np

def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)
    on = d["offboard"] > 0
    t = d["t"][on] - d["t"][on][0]
    e = np.column_stack([d[f"p_{a}"][on] - d[f"r_{a}"][on] for a in "xyz"])
    return t, e, str(d["controller"][0]), float(np.nanmax(d["wind"]))

paths = sys.argv[1:] or sorted(
    [p for p in glob.glob("logs/adrc_*.csv")
     if "wind" in open(p).readline() and np.nanmax(np.genfromtxt(p, delimiter=",", names=True)["wind"]) > 0],
    key=os.path.getmtime)[-2:]
phases = [("calm", 1, 5), ("steady wind", 5, 15), ("recovery", 15, 25), ("gusts", 25, 40)]
print(f"{'log':34s} {'ctrl':5s} {'phase':12s} {'max|e_y| cm':>11s} {'RMS e_y cm':>10s} {'RMS |e| 3D cm':>13s}")
for p in paths:
    t, e, ctrl, wmax = load(p)
    for name, lo, hi in phases:
        m = (t >= lo) & (t < hi)
        if not m.any(): continue
        ey = e[m, 1]
        print(f"{os.path.basename(p):34s} {ctrl:5s} {name:12s} {100*np.abs(ey).max():11.1f} "
              f"{100*np.sqrt(np.mean(ey**2)):10.1f} {100*np.sqrt(np.mean(np.sum(e[m]**2, 1))):13.1f}")
    print(f"{'':34s} peak wind {wmax:.1f} m/s")
