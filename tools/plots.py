#!/usr/bin/env python3
"""Figures from a recorded run.

Every figure plots the firmware's estimate against the truth the simulation
knows and the firmware does not. That comparison is the only honest way to
show an estimator working: a plot of an estimate on its own shows a smooth
line and proves nothing.

Reads   data/replay.csv, data/scene.tsv
Writes  data/fig_*.png
"""
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(HERE, "data")

# Figures go in docs/, not data/.
#
# data/ is gitignored, because it holds the output of a run: a 170 KB CSV and
# a 279 KB page that are regenerated every time and would make the history
# unreadable. The figures are different. They are referenced by the README, so
# a reader who has only cloned the repository has to be able to see them, and
# a gitignored figure is a broken image on every page that links to it.
DOCS = os.path.join(HERE, "docs")

INK = "#1b1e24"
GRID = "#d8dce3"
TRUE = "#2b2f36"
EST = "#2f6fd0"
BAND = "#9dbdf0"
BAD = "#c7453f"
OK = "#2f9e5f"


def load():
    with open(os.path.join(DATA, "replay.csv")) as f:
        rows = list(csv.reader(f))
    head = rows[0]
    idx = {n: i for i, n in enumerate(head)}
    a = np.array([[float(v) for v in r] for r in rows[1:] if len(r) == len(head)])
    return {n: a[:, i] for n, i in idx.items()}


def load_scene():
    sc = {"objects": [], "cspace": [], "path": []}
    p = os.path.join(DATA, "scene.tsv")
    if not os.path.exists(p):
        return sc
    with open(p) as f:
        for line in f:
            q = line.rstrip("\n").split("\t")
            if q[0] in ("table_y", "link1", "link2"):
                sc[q[0]] = float(q[1])
            elif q[0] == "grid":
                sc["grid"] = int(q[1])
            elif q[0] == "object":
                sc["objects"].append([float(x) for x in q[1:4]])
            elif q[0] == "cspace":
                sc["cspace"].append(q[1])
            elif q[0] == "path":
                sc["path"].append([float(q[1]), float(q[2])])
    return sc


def style(ax, xlabel, ylabel, title):
    ax.set_facecolor("white")
    ax.grid(True, color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    for s in ax.spines.values():
        s.set_color(GRID)
    ax.tick_params(colors=INK, labelsize=9)
    ax.set_xlabel(xlabel, color=INK, fontsize=10)
    ax.set_ylabel(ylabel, color=INK, fontsize=10)
    ax.set_title(title, color=INK, fontsize=11, loc="left", pad=10)


def fk(q1, q2, l1, l2):
    return (l1 * np.cos(q1) + l2 * np.cos(q1 + q2),
            l1 * np.sin(q1) + l2 * np.sin(q1 + q2))


def save(fig, name):
    os.makedirs(DOCS, exist_ok=True)
    path = os.path.join(DOCS, name)
    fig.savefig(path, dpi=150, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"  docs/{name}")


def fig_payload(d):
    fig, ax = plt.subplots(figsize=(8, 3.4))
    t = d["t"]
    lo = d["epay"] - 3 * d["psig"]
    hi = d["epay"] + 3 * d["psig"]
    ax.fill_between(t, lo, hi, color=BAND, alpha=0.45, lw=0,
                    label="estimate, 3 sigma")
    ax.plot(t, d["epay"], color=EST, lw=1.5, label="estimated")
    ax.plot(t, d["tpay"], color=TRUE, lw=1.6, ls="--", label="true")
    style(ax, "time, s", "payload, kg",
          "What is in the gripper, with no scale and no force sensor")
    ax.set_ylim(-0.25, 0.95)
    ax.legend(frameon=False, fontsize=9, loc="upper left", ncol=3)
    save(fig, "fig_payload.png")


def fig_force(d):
    fig, ax = plt.subplots(figsize=(8, 3.0))
    ax.plot(d["t"], d["tforce"], color=TRUE, lw=1.6, ls="--", label="true")
    ax.plot(d["t"], d["eforce"], color=EST, lw=1.4, label="estimated")
    style(ax, "time, s", "contact force, N",
          "How hard the tool is pressing, inferred from motor current")
    ax.legend(frameon=False, fontsize=9, loc="upper left", ncol=2)
    save(fig, "fig_force.png")


def fig_error(d, sc):
    l1, l2 = sc.get("link1", 0.25), sc.get("link2", 0.20)
    tx, ty = fk(d["tq1"], d["tq2"], l1, l2)
    ex, ey = fk(d["eq1"], d["eq2"], l1, l2)
    err = np.hypot(tx - ex, ty - ey) * 1000.0

    fig, ax = plt.subplots(figsize=(8, 3.0))
    ax.plot(d["t"], err, color=EST, lw=1.4)

    # Where the arm was stopped, shaded. 2 is SAFE STOP.
    stopped = d["safety"] >= 2
    if stopped.any():
        ax.fill_between(d["t"], 0, err.max() * 1.1, where=stopped,
                        color=BAD, alpha=0.12, lw=0, label="safe stop")

    nominal = err[d["safety"] == 0]
    if nominal.size:
        ax.axhline(nominal.mean(), color=OK, lw=1.0, ls=":",
                   label=f"mean while healthy: {nominal.mean():.2f} mm")
    style(ax, "time, s", "tool position error, mm",
          "How far the firmware's idea of itself is from the truth")
    ax.set_ylim(bottom=0)
    ax.legend(frameon=False, fontsize=9, loc="upper left")
    save(fig, "fig_error.png")
    return err, nominal


def fig_path(d, sc):
    l1, l2 = sc.get("link1", 0.25), sc.get("link2", 0.20)
    tx, ty = fk(d["tq1"], d["tq2"], l1, l2)
    ex, ey = fk(d["eq1"], d["eq2"], l1, l2)

    fig, ax = plt.subplots(figsize=(7, 4.2))
    table = sc.get("table_y", -0.15)
    ax.axhspan(table - 0.012, table, color="#c3c8d0", lw=0)
    for ox, otop, ohw in sc["objects"]:
        ax.add_patch(plt.Rectangle((ox - ohw, table), 2 * ohw, otop - table,
                                   facecolor="#d8a05a", edgecolor="#a97c3c",
                                   lw=0.8))
    ax.plot(tx, ty, color=TRUE, lw=1.3, label="true tool path")
    ax.plot(ex, ey, color=EST, lw=1.0, ls="--", label="estimated")
    ax.plot(0, 0, "o", color=INK, ms=6)
    style(ax, "x, m", "y, m",
          "Where the tool went, and where it thought it went")
    ax.set_aspect("equal")
    ax.legend(frameon=False, fontsize=9, loc="upper left")
    save(fig, "fig_path.png")


def fig_cspace(sc):
    if not sc["cspace"]:
        return
    n = sc.get("grid", len(sc["cspace"]))
    grid = np.array([[1.0 if ch == "#" else 0.0 for ch in row]
                     for row in sc["cspace"]])
    fig, ax = plt.subplots(figsize=(5.2, 5.0))
    ax.imshow(grid, origin="lower", cmap="RdGy_r", vmin=-0.4, vmax=1.4,
              extent=[-np.pi, np.pi, -np.pi, np.pi], interpolation="nearest")
    if sc["path"]:
        p = np.array(sc["path"])
        ax.plot(p[:, 1], p[:, 0], color=OK, lw=1.8, label="planned route")
        ax.plot(p[0, 1], p[0, 0], "o", color=OK, ms=5)
        ax.plot(p[-1, 1], p[-1, 0], "s", color=OK, ms=5)
        ax.legend(frameon=False, fontsize=9, loc="lower left")
    style(ax, "joint 2, rad", "joint 1, rad",
          "Every pose the arm can hold.\nShaded poses would collide.")
    blocked = int(grid.sum())
    ax.text(0.98, 0.02, f"{blocked} of {n*n} poses blocked",
            transform=ax.transAxes, ha="right", va="bottom", fontsize=8.5,
            color=INK)
    save(fig, "fig_cspace.png")


def main():
    if not os.path.exists(os.path.join(DATA, "replay.csv")):
        print("no data/replay.csv, run 'make viewer' or build/record first")
        return 1
    d = load()
    sc = load_scene()
    print("figures:")
    fig_payload(d)
    fig_force(d)
    err, nominal = fig_error(d, sc)
    fig_path(d, sc)
    fig_cspace(sc)

    # The numbers these figures contain, printed so the README can quote
    # them without anyone reading them off a graph by eye.
    print("\nnumbers from this run:")

    # Carrying, and separately carrying with the estimate settled.
    #
    # Both are reported because either alone misleads. The worst error over
    # the whole carry happens in the instant after the jaws close, before the
    # filter has seen any evidence, and quoting that as the accuracy of the
    # method is unfair. Quoting only the settled figure is flattering, and
    # hides that the number takes a moment to arrive, which matters to anyone
    # deciding when to act on it.
    pay_mask = d["tpay"] > 0.05
    settled = pay_mask & (d["psig"] < 0.05)
    if pay_mask.any():
        pe = np.abs(d["epay"][pay_mask] - d["tpay"][pay_mask])
        print(f"  payload error while carrying   mean {pe.mean()*1000:.0f} g, "
              f"worst {pe.max()*1000:.0f} g  (includes the grip transient)")
    if settled.any():
        ps = np.abs(d["epay"][settled] - d["tpay"][settled])
        frac = 100.0 * settled.sum() / max(1, pay_mask.sum())
        print(f"  ... once the filter has settled  mean {ps.mean()*1000:.0f} g, "
              f"worst {ps.max()*1000:.0f} g  ({frac:.0f}% of the carry)")
    f_mask = d["tforce"] > 2.0
    if f_mask.any():
        fe = np.abs(d["eforce"][f_mask] - d["tforce"][f_mask])
        print(f"  contact force error in contact mean {fe.mean():.1f} N, "
              f"worst {fe.max():.1f} N")
    if nominal.size:
        print(f"  tool position error, healthy   mean {nominal.mean():.2f} mm, "
              f"worst {nominal.max():.2f} mm")
    print(f"  worst position error overall   {err.max():.1f} mm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
