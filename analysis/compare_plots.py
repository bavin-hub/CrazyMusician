#!/usr/bin/env python3
"""Compare ADRC and PX4's default position controller in the scripted wind test,
and summarize a slider-panel (controlled wind) session.
Usage: python3 compare_plots.py ADRC.csv PX4.csv [PANEL.csv]   -> PNGs next to ADRC.csv"""
import os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

K_WIND = 0.3
C_ADRC, C_PX4 = "#1f77b4", "#d62728"
PHASES = [("calm", 3, 5), ("steady wind", 5, 15), ("recovery", 15, 25), ("gusts", 25, 40)]


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)
    on = d["offboard"] > 0
    g = {k: np.asarray(d[k][on], float) for k in d.dtype.names if k != "controller"}
    g["t"] = g["t"] - g["t"][0]
    for a in "xyz":
        g[f"e_{a}"] = g[f"p_{a}"] - g[f"r_{a}"]
    g["e_z"] = -g["e_z"]                         # height error, up positive
    # PX4's hold mode keeps its own altitude setpoint (here ~20 cm below the logged
    # reference), so height is compared as deviation from each run's mean height
    # after the handover transient.
    settled = g["t"] >= 5
    g["e_z"] = g["e_z"] - g["e_z"][settled].mean()
    g["e_h"] = np.hypot(g["e_x"], g["e_y"])
    return g


def shade(ax, g):
    ax.fill_between(g["t"], 0, 1, where=g["wind"] > 0.05, color="0.5", alpha=0.12,
                    transform=ax.get_xaxis_transform(), lw=0)


def stats(g, key, lo, hi):
    m = (g["t"] >= lo) & (g["t"] < hi)
    e = g[key][m]
    return np.sqrt(np.mean(e ** 2)), np.abs(e).max()


def fig_timeseries(A, P, out):
    fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True,
                           gridspec_kw={"height_ratios": [0.7, 1.4, 1, 1]})
    ax[0].plot(A["t"], A["wind"], "k", lw=1.5)
    ax[0].set_ylabel("wind [m/s]")
    ax[0].set_title("Wind toward east (PX4 +y): calm · steady 4 m/s · calm · gusts")
    rows = [("e_y", "y error (along wind) [cm]"), ("e_x", "x error (across wind) [cm]"),
            ("e_z", "height deviation [cm]\n(from mean height)")]
    for i, (k, lab) in enumerate(rows, start=1):
        ax[i].plot(P["t"], 100 * P[k], color=C_PX4, lw=1.1, label="PX4 default")
        ax[i].plot(A["t"], 100 * A[k], color=C_ADRC, lw=1.1, label="ADRC")
        ax[i].axhline(0, color="k", lw=0.6)
        ax[i].set_ylabel(lab)
    ax[1].legend(loc="upper right")
    for a in ax:
        shade(a, A); a.grid(alpha=0.3)
        for _, lo, _ in PHASES[1:]:
            a.axvline(lo, color="0.4", lw=0.6, ls=":")
    ax[-1].set_xlabel("time since start of test [s]")
    fig.suptitle("Position error under wind: ADRC vs PX4 default controller", fontsize=14)
    fig.tight_layout(); fig.savefig(out, dpi=120); plt.close(fig)


def fig_summary(A, P, out):
    fig = plt.figure(figsize=(13, 5.5))
    gs = fig.add_gridspec(1, 3, width_ratios=[1.6, 1, 1])
    ax = fig.add_subplot(gs[0])
    names = [p[0] for p in PHASES]; x = np.arange(len(names)); w = 0.2
    for j, (g, c, lab) in enumerate(((P, C_PX4, "PX4"), (A, C_ADRC, "ADRC"))):
        rms = [100 * stats(g, "e_h", lo, hi)[0] for _, lo, hi in PHASES]
        mx = [100 * stats(g, "e_h", lo, hi)[1] for _, lo, hi in PHASES]
        b1 = ax.bar(x + (j - 0.5) * 2 * w - w / 2, rms, w, color=c, label=f"{lab} RMS")
        b2 = ax.bar(x + (j - 0.5) * 2 * w + w / 2, mx, w, color=c, alpha=0.4,
                    label=f"{lab} max")
        ax.bar_label(b1, fmt="%.1f", fontsize=8); ax.bar_label(b2, fmt="%.1f", fontsize=8)
    ax.set_xticks(x); ax.set_xticklabels(names)
    ax.set_ylabel("horizontal position error [cm]")
    ax.set_title("Horizontal error by phase"); ax.legend(fontsize=8, ncol=2); ax.grid(axis="y", alpha=0.3)
    lim = 100 * max(np.abs(A["e_x"]).max(), np.abs(A["e_y"]).max(),
                    np.abs(P["e_x"]).max(), np.abs(P["e_y"]).max()) * 1.1
    for k, (g, c, lab) in enumerate(((P, C_PX4, "PX4 default"), (A, C_ADRC, "ADRC"))):
        a = fig.add_subplot(gs[k + 1])
        windy = g["wind"] > 0.05
        a.plot(100 * g["e_y"][~windy], 100 * g["e_x"][~windy], ".", ms=1.5, color="0.6",
               label="calm")
        a.plot(100 * g["e_y"][windy], 100 * g["e_x"][windy], ".", ms=1.5, color=c,
               label="windy")
        a.plot(0, 0, "k+", ms=12, mew=2)
        a.annotate("", xy=(0.9 * lim, 0.85 * lim), xytext=(0.45 * lim, 0.85 * lim),
                   arrowprops=dict(arrowstyle="->", lw=2))
        a.text(0.67 * lim, 0.92 * lim, "wind", ha="center")
        a.set_xlim(-lim, lim); a.set_ylim(-lim, lim); a.set_aspect("equal")
        a.set_xlabel("east error, y [cm]"); a.set_ylabel("north error, x [cm]")
        a.set_title(f"{lab}: top view"); a.grid(alpha=0.3); a.legend(fontsize=8, loc="lower left")
    fig.suptitle("ADRC vs PX4 default controller under the same wind", fontsize=14)
    fig.tight_layout(); fig.savefig(out, dpi=120); plt.close(fig)


def fig_panel(C, out):
    th = np.radians(C["wind_dir"])
    wx, wy = C["wind"] * np.cos(th), C["wind"] * np.sin(th)      # NED north, east
    fig, ax = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    ax[0].plot(C["t"], C["wind"], "k", lw=1.4, label="speed [m/s]")
    a2 = ax[0].twinx()
    a2.plot(C["t"], np.where(C["wind"] > 0.05, C["wind_dir"], np.nan), ".", ms=2,
            color="tab:purple")
    a2.set_ylabel("toward [deg]", color="tab:purple"); a2.set_ylim(0, 360)
    a2.set_yticks([0, 90, 180, 270, 360]); a2.set_yticklabels(["N", "E", "S", "W", "N"])
    ax[0].set_ylabel("wind [m/s]"); ax[0].set_title("Wind set from the slider panel")
    for i, (k, comp, lab) in enumerate((("z2_x", wx, "north (x)"), ("z2_y", wy, "east (y)")), 1):
        ax[i].plot(C["t"], C[k], color="tab:red", lw=1.1, label="ESO disturbance estimate")
        ax[i].plot(C["t"], K_WIND * comp, "k--", lw=1, label="nominal k·wind")
        ax[i].set_ylabel(f"{lab} [m/s²]"); ax[i].axhline(0, color="k", lw=0.5)
    ax[1].legend(loc="upper right", fontsize=8)
    ax[3].plot(C["t"], 100 * C["e_y"], lw=1, label="east error (y)")
    ax[3].plot(C["t"], 100 * C["e_x"], lw=1, label="north error (x)")
    ax[3].plot(C["t"], 100 * C["e_z"], lw=1, alpha=0.7, label="height error")
    ax[3].set_ylabel("position error [cm]"); ax[3].legend(loc="upper right", fontsize=8, ncol=3)
    ax[3].set_xlabel("time [s]")
    for a in ax:
        shade(a, C); a.grid(alpha=0.3)
    fig.suptitle("ADRC with wind from any direction: the ESO tracks both components", fontsize=14)
    fig.tight_layout(); fig.savefig(out, dpi=120); plt.close(fig)


if __name__ == "__main__":
    A, P = load(sys.argv[1]), load(sys.argv[2])
    base = os.path.dirname(os.path.abspath(sys.argv[1]))
    fig_timeseries(A, P, os.path.join(base, "compare_timeseries.png"))
    fig_summary(A, P, os.path.join(base, "compare_summary.png"))
    print(f"{'phase':12s} {'PX4 RMS':>8s} {'ADRC RMS':>9s} {'PX4 max':>8s} {'ADRC max':>9s}  [cm, horizontal]")
    for name, lo, hi in PHASES:
        (pr, pm), (ar, am) = stats(P, "e_h", lo, hi), stats(A, "e_h", lo, hi)
        print(f"{name:12s} {100*pr:8.1f} {100*ar:9.1f} {100*pm:8.1f} {100*am:9.1f}")
    for name, lo, hi in PHASES:
        (pr, pm), (ar, am) = stats(P, "e_z", lo, hi), stats(A, "e_z", lo, hi)
        print(f"height dev {name:12s} PX4 RMS {100*pr:.1f} max {100*pm:.1f} | ADRC RMS {100*ar:.1f} max {100*am:.1f}")
    if len(sys.argv) > 3:
        fig_panel(load(sys.argv[3]), os.path.join(base, "panel_session.png"))
    print("plots saved in", base)
