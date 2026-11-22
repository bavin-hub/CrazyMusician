#!/usr/bin/env python3
"""
Hover report in the PX4 NED frame (x north, y east, z down; no sign flips).

  python3 analysis/plot_hover.py [logs/adrc_....csv] [--settle 5] [--tilt-max 20] [--vmax 1.0]

Writes <log>_hover.png and <log>_hover.txt (the numbers to compare a hardware
hover against). Time 0 = the moment Offboard engaged; the grey band before it
is PX4's own hold. Statistics use the window after --settle seconds.
"""
import argparse, glob, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

G = 9.81
AX = ("x (north)", "y (east)", "z (down)")


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)
    f = lambda k: np.asarray(d[k], float)
    on = f("offboard") > 0
    if not on.any():
        raise SystemExit("Offboard never engaged in this log.")
    t = f("t") - f("t")[on][0]
    g = {"t": t, "on": on, "controller": str(d["controller"][0]).strip()}
    for k in ("r", "p", "vref", "v", "z1", "z2", "u"):
        g[k] = np.column_stack([f(f"{k}_{a}") for a in "xyz"])
    for k in ("roll", "pitch", "yaw"):
        g[k] = f(k) if k in d.dtype.names else np.full(len(t), np.nan)
    return g


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?")
    ap.add_argument("--settle", type=float, default=5.0, help="s after engagement excluded from stats")
    ap.add_argument("--tilt-max", type=float, default=20.0, help="MPC_TILTMAX_AIR [deg]")
    ap.add_argument("--vmax", type=float, default=1.0, help="horizontal velocity limit [m/s]")
    a = ap.parse_args()
    path = a.log or max(glob.glob("logs/adrc_*.csv"), key=os.path.getmtime)
    g = load(path)
    t, e = g["t"], g["p"] - g["r"]                      # position error, measured - desired
    win = g["on"] & (t >= a.settle)
    tilt = np.degrees(np.arccos(np.clip(np.cos(g["roll"]) * np.cos(g["pitch"]), -1, 1)))

    # ---------------------------------------------------------------- figure
    fig = plt.figure(figsize=(16, 13))
    gs = fig.add_gridspec(5, 3, height_ratios=[1, 1, 1, 0.9, 0.9])
    axs = [[fig.add_subplot(gs[i, j]) for j in range(3)] for i in range(3)]
    for j in range(3):
        x = axs[0][j]
        x.plot(t, g["r"][:, j], "k--", lw=1.3, label="desired")
        x.plot(t, g["p"][:, j], color="tab:blue", lw=1.2, label="measured (PX4 estimate)")
        x.set_title(f"position {AX[j]} [m]")
        rms = 100 * np.sqrt(np.mean(e[win, j] ** 2))
        x.text(0.02, 0.92, f"RMS error {rms:.1f} cm", transform=x.transAxes, fontsize=9,
               bbox=dict(fc="white", alpha=0.7, lw=0))
        x = axs[1][j]
        x.plot(t, g["vref"][:, j], "k--", lw=1.3, label="commanded v_ref")
        x.plot(t, g["v"][:, j], color="tab:blue", lw=1.1, label="measured")
        x.plot(t, g["z1"][:, j], color="tab:orange", lw=0.8, alpha=0.8, label="ESO estimate z1")
        x.set_title(f"velocity {AX[j]} [m/s]")
        x = axs[2][j]
        x.plot(t, g["z2"][:, j], color="tab:red", lw=1.2, label="disturbance estimate z2")
        x.plot(t, g["u"][:, j], color="tab:green", lw=1.1, label="acceleration sent (feedforward)")
        x.set_title(f"acceleration {AX[j]} [m/s²]")
    axs[0][0].legend(fontsize=8, loc="lower left"); axs[1][0].legend(fontsize=8, loc="lower left")
    axs[2][0].legend(fontsize=8, loc="lower left")

    x = fig.add_subplot(gs[3, :2])
    x.plot(t, 100 * np.hypot(e[:, 0], e[:, 1]), label="horizontal |e_xy|")
    x.plot(t, 100 * np.abs(e[:, 2]), label="vertical |e_z|")
    x.set_title("position error magnitude [cm]"); x.legend(fontsize=8)
    bottom = [x]
    x = fig.add_subplot(gs[3, 2])
    x.plot(t, np.degrees(g["roll"]), lw=1, label="roll")
    x.plot(t, np.degrees(g["pitch"]), lw=1, label="pitch")
    x.plot(t, tilt, "k", lw=0.8, alpha=0.6, label="total tilt")
    for s in (1, -1):
        x.axhline(s * a.tilt_max, color="tab:red", ls=":", lw=1)
    x.set_title(f"attitude [deg] (red: ±{a.tilt_max:.0f}° tilt limit)"); x.legend(fontsize=8)
    bottom.append(x)
    x = fig.add_subplot(gs[4, :2])
    dt = np.diff(t) * 1000
    x.plot(t[1:], dt, lw=0.6)
    x.set_ylim(0, max(50.0, 1.1 * dt.max()))
    x.ticklabel_format(axis="y", useOffset=False, style="plain")
    x.set_title("controller update interval [ms] (setpoints are sent once per update)")
    x.set_xlabel("time since Offboard engaged [s]")
    bottom.append(x)
    x = fig.add_subplot(gs[4, 2])
    x.plot(e[win, 1] * 100, e[win, 0] * 100, ".", ms=1.5)
    lim = max(5, 110 * np.abs(e[win, :2]).max())
    x.set_xlim(-lim, lim); x.set_ylim(-lim, lim); x.set_aspect("equal")
    x.axhline(0, color="k", lw=0.5); x.axvline(0, color="k", lw=0.5)
    x.set_xlabel("east error [cm]"); x.set_ylabel("north error [cm]")
    x.set_title("horizontal error, top view (after settle)")

    for x in [*sum(axs, []), *bottom[:3]]:
        x.grid(alpha=0.3)
        x.axvspan(t[0], 0, color="0.5", alpha=0.12, lw=0)
        x.axvline(a.settle, color="0.4", ls=":", lw=0.8)
    for j in range(3):
        axs[2][j].set_xlabel("time since Offboard engaged [s]")
    fig.suptitle(f"{os.path.basename(path)} — hover, controller {g['controller']}, PX4 NED frame "
                 f"(grey: before Offboard; dotted: end of settle window)", fontsize=12)
    fig.tight_layout()
    png = os.path.splitext(path)[0] + "_hover.png"
    fig.savefig(png, dpi=100)

    # ---------------------------------------------------------------- numbers
    L = []
    dur = t[g["on"]][-1]
    L.append(f"Log {os.path.basename(path)}  controller {g['controller']}  "
             f"Offboard {dur:.1f} s  stats window {a.settle:.0f}-{dur:.0f} s")
    L.append("Position error (measured - desired) [cm]   mean    std    RMS    max|.|")
    for j in range(3):
        ej = 100 * e[win, j]
        L.append(f"  {AX[j]:10s}                              {ej.mean():6.1f} {ej.std():6.1f} "
                 f"{np.sqrt(np.mean(ej**2)):6.1f} {np.abs(ej).max():8.1f}")
    eh = 100 * np.hypot(e[win, 0], e[win, 1])
    e3 = 100 * np.linalg.norm(e[win], axis=1)
    L.append(f"  horizontal |e_xy| RMS {np.sqrt(np.mean(eh**2)):.1f} cm, max {eh.max():.1f} cm;"
             f"  3D RMS {np.sqrt(np.mean(e3**2)):.1f} cm")
    vr = g["vref"][win]
    sat = np.mean(np.abs(vr[:, :2]).max(1) >= 0.99 * a.vmax) * 100
    L.append(f"Commanded velocity: RMS {np.sqrt(np.mean(np.sum(vr**2,1))):.3f} m/s, "
             f"max |v_ref| {np.abs(vr).max():.3f} m/s, at horizontal limit {sat:.1f}% of time")
    ff = g["u"][win]; z2 = g["z2"][win]
    L.append(f"Disturbance estimate z2 mean [m/s²]: x {z2[:,0].mean():+.3f}  y {z2[:,1].mean():+.3f}  "
             f"z {z2[:,2].mean():+.3f}   (z ≠ 0: hover-thrust mismatch)")
    L.append(f"Feedforward sent: max |a_xy| {np.hypot(ff[:,0], ff[:,1]).max():.3f} m/s² "
             f"(~{np.degrees(np.arctan(np.hypot(ff[:,0], ff[:,1]).max()/G)):.1f}° tilt), "
             f"max |a_z| {np.abs(ff[:,2]).max():.3f} m/s²")
    if np.isfinite(tilt).any():
        L.append(f"Attitude: roll RMS {np.degrees(np.sqrt(np.nanmean(g['roll'][win]**2))):.2f}°, "
                 f"pitch RMS {np.degrees(np.sqrt(np.nanmean(g['pitch'][win]**2))):.2f}°, "
                 f"max tilt {np.nanmax(tilt[win]):.1f}° (limit {a.tilt_max:.0f}°)")
    dtw = np.diff(t[g["on"]]) * 1000
    L.append(f"Update rate: median {1000/np.median(dtw):.0f} Hz, max gap {dtw.max():.0f} ms, "
             f"gaps > 50 ms: {np.sum(dtw > 50)}")
    warn = []
    if dtw.max() > 200: warn.append("update gap > 200 ms (PX4 offboard timeout is 1 s)")
    if np.isfinite(tilt).any() and np.nanmax(tilt[win]) > 0.8 * a.tilt_max: warn.append("tilt above 80% of the limit")
    if sat > 5: warn.append("velocity command at its limit > 5% of the time")
    L.append("Warnings: " + ("; ".join(warn) if warn else "none"))
    txt = os.path.splitext(path)[0] + "_hover.txt"
    open(txt, "w").write("\n".join(L) + "\n")
    print("\n".join(L)); print(f"Saved {png}\n      {txt}")


if __name__ == "__main__":
    main()
