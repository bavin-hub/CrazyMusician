#!/usr/bin/env python3
"""Plot a cascaded-ADRC log from adrc_offboard.py.
Usage: python3 analysis/plot_adrc.py [logs/adrc_....csv]   (default: newest log)
Writes a PNG next to the CSV. z is plotted as height (up positive)."""
import glob, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

logs = glob.glob("logs/adrc_*.csv")
if len(sys.argv) < 2 and not logs:
    sys.exit("No logs/adrc_*.csv found; run adrc_offboard.py through the handover first.")
path = sys.argv[1] if len(sys.argv) > 1 else max(logs, key=os.path.getmtime)
d = np.genfromtxt(path, delimiter=",", names=True)
on = d["offboard"] > 0
t = d["t"] - (d["t"][on][0] if on.any() else d["t"][0])

fig, ax = plt.subplots(4, 3, figsize=(14, 11), sharex=True)
for j, a in enumerate("xyz"):
    s = -1 if a == "z" else 1
    name = "height" if a == "z" else a
    ax[0, j].plot(t, s * d[f"r_{a}"], "k--", lw=1, label="reference")
    ax[0, j].plot(t, s * d[f"p_{a}"], lw=1.2, label="PX4 estimate")
    ax[0, j].set_title(f"{name} position [m]")
    ax[1, j].plot(t, s * d[f"vref_{a}"], "k--", lw=1, label="v_ref (outer loop)")
    ax[1, j].plot(t, s * d[f"v_{a}"], lw=1.2, label="PX4 velocity")
    ax[1, j].plot(t, s * d[f"z1_{a}"], lw=0.8, alpha=0.7, label="ESO z1")
    ax[1, j].set_title(f"{name} velocity [m/s]")
    ax[2, j].plot(t, s * d[f"z2_{a}"], color="tab:red", lw=1.2)
    ax[2, j].set_title(f"disturbance estimate z2_{a} [m/s²]")
    ax[3, j].plot(t, s * d[f"u_{a}"], color="tab:green", lw=1.2)
    ax[3, j].set_title(f"acceleration command u_{a} [m/s²]")
    ax[3, j].set_xlabel("time since offboard [s]")
    for r in range(4):
        ax[r, j].grid(alpha=0.3); ax[r, j].axvline(0, color="gray", lw=0.6)
# Wind test: shade windy periods and overlay the applied wind acceleration
K_WIND = 0.3                       # must match the world's WindEffects factor
names = d.dtype.names
if "wind" in names and np.nanmax(d["wind"]) > 0.05:
    windy = d["wind"] > 0.05
    for r in range(4):
        for j in range(3):
            ax[r, j].fill_between(t, 0, 1, where=windy, color="tab:blue", alpha=0.07,
                                  transform=ax[r, j].get_xaxis_transform(), lw=0)
    # drag accel on a hovering drone ~ k * wind, split into NED x (north) / y (east)
    th = np.radians(d["wind_dir"]) if "wind_dir" in names else np.full(len(t), np.pi / 2)
    for j, comp in ((0, np.cos(th)), (1, np.sin(th))):
        ax[2, j].plot(t, K_WIND * d["wind"] * comp, "k--", lw=1, label="k·wind (applied)")
        ax[2, j].legend(fontsize=8)
    ax[0, 1].text(0.01, 0.95, f"peak wind {np.nanmax(d['wind']):.1f} m/s (shaded)",
                  transform=ax[0, 1].transAxes, fontsize=8, va="top")
ctrl = ""
if "controller" in names:
    raw = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)
    ctrl = f"   controller: {raw['controller'][0]}"
ax[0, 0].legend(fontsize=8); ax[1, 0].legend(fontsize=8)
fig.suptitle(os.path.basename(path) + ctrl + "   (z plotted as height, up positive)")
fig.tight_layout()
out = os.path.splitext(path)[0] + ".png"
fig.savefig(out, dpi=110)
print(f"Saved {out}")
