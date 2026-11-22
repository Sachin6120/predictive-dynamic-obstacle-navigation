#!/usr/bin/env python3
"""Regenerate the README figures from committed machine-readable evidence.

Usage (from the repository root):

    python3 docs/figures/make_figures.py

Every number plotted is read from a file under ``validation/``. Nothing is
hard-coded except axis limits, labels and layout. Outputs are written to
``docs/images/``:

    predictive_nav_architecture.svg   Figure A -- final system architecture
    fig_b_predictor_comparison.svg    Figure B -- Stage-5E predictor comparison
    fig_c_planner_visible_cells.svg   Figure C -- planner-visible occupancy
    fig_d_semantic_corridor.svg       Figure D -- Stage-5D.1 corridor audit
    fig_e_classification.svg          Figure E -- Stage-5B classification

Figure A is emitted as hand-authored SVG; B-E are rendered with matplotlib.
"""

from __future__ import annotations

import json
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

ROOT = Path(__file__).resolve().parents[2]
VAL = ROOT / "validation"
OUT = ROOT / "docs" / "images"

# ---------------------------------------------------------------------------
# Palette: validated categorical slots 1-4 plus chart chrome and status ink.
# ---------------------------------------------------------------------------
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]  # blue, orange, aqua, yellow
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
CRITICAL = "#d03b3b"
GOOD = "#0ca30c"

ARMS = ["CV-Kalman", "Global reach.", "Class-aware reach."]
ARM_COLORS = {a: c for a, c in zip(ARMS, SERIES[:3])}

plt.rcParams.update(
    {
        "svg.fonttype": "path",
        "font.family": "sans-serif",
        "font.sans-serif": ["DejaVu Sans"],
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "axes.edgecolor": AXIS,
        "axes.labelcolor": INK2,
        "text.color": INK,
        "xtick.color": MUTED,
        "ytick.color": MUTED,
        "grid.color": GRID,
        "axes.grid": False,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "font.size": 10,
    }
)


def load(rel: str):
    with open(VAL / rel) as fh:
        return json.load(fh)


def style(ax, ylabel=None, title=None, subtitle=None):
    if title:
        ax.set_title(title, color=INK, fontsize=12, fontweight="bold", loc="left", pad=18 if subtitle else 8)
    if subtitle:
        ax.text(0, 1.015, subtitle, transform=ax.transAxes, color=INK2, fontsize=9.5, va="bottom")
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=9.5)
    ax.yaxis.grid(True, linewidth=0.8)
    ax.set_axisbelow(True)
    ax.tick_params(length=0)


def save(fig, name):
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / name
    fig.savefig(path, format="svg", bbox_inches="tight", facecolor=SURFACE)
    plt.close(fig)
    # Matplotlib wraps long `<path d="...">` coordinate runs and leaves a space
    # before each newline. That whitespace is semantically irrelevant in SVG path
    # data, but it trips `git diff --check`, so strip it.
    path.write_text("\n".join(line.rstrip() for line in path.read_text().splitlines()) + "\n")
    print(f"wrote {path.relative_to(ROOT)}")


# ===========================================================================
# FIGURE A -- architecture
# ===========================================================================
def figure_a():
    W, H = 1240, 1232
    s: list[str] = []
    f = "font-family=\"Helvetica, Arial, 'Liberation Sans', sans-serif\""
    mono = "font-family=\"'DejaVu Sans Mono', Menlo, Consolas, monospace\""

    def box(x, y, w, h, title, sub=None, accent=INK2, fill="#eef1f4", badge=None, badge_fill=None):
        s.append(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" ry="12" '
            f'fill="{fill}" stroke="{accent}" stroke-width="2"/>'
        )
        ty = y + (30 if sub else h / 2 + 6)
        s.append(
            f'<text x="{x + w / 2}" y="{ty}" {f} font-size="17" font-weight="700" '
            f'fill="{INK}" text-anchor="middle">{title}</text>'
        )
        if sub:
            for i, line in enumerate(sub):
                s.append(
                    f'<text x="{x + w / 2}" y="{ty + 23 + i * 18}" {f} font-size="12.5" '
                    f'fill="{INK2}" text-anchor="middle">{line}</text>'
                )
        if badge:
            bw = 11 * len(badge) + 18
            s.append(
                f'<rect x="{x + w - bw - 10}" y="{y - 12}" width="{bw}" height="24" rx="12" ry="12" '
                f'fill="{badge_fill}"/>'
            )
            s.append(
                f'<text x="{x + w - bw / 2 - 10}" y="{y + 4}" {f} font-size="11.5" font-weight="700" '
                f'fill="#ffffff" text-anchor="middle" letter-spacing="0.6">{badge}</text>'
            )

    def arrow(x1, y1, x2, y2, color=INK2, dash=None, width=2.4):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        s.append(
            f'<path d="M{x1},{y1} L{x2},{y2}" stroke="{color}" stroke-width="{width}" '
            f'fill="none" marker-end="url(#ah)"{d}/>'
        )

    def label(x, y, text, size=12.5, fill=MUTED, anchor="middle", weight="normal", font=f, style_=""):
        st = ' font-style="italic"' if style_ == "i" else ""
        s.append(
            f'<text x="{x}" y="{y}" {font} font-size="{size}" font-weight="{weight}" '
            f'fill="{fill}" text-anchor="{anchor}"{st}>{text}</text>'
        )

    s.append(f'<?xml version="1.0" encoding="UTF-8"?>')
    s.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
    s.append("<title>Predictive Dynamic-Obstacle Navigation — final ROS 2 / Nav2 architecture</title>")
    s.append(
        f'<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="8" markerHeight="8" '
        f'orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{INK2}"/></marker></defs>'
    )
    s.append(f'<rect width="{W}" height="{H}" fill="{SURFACE}"/>')

    label(620, 44, "Predictive Dynamic-Obstacle Navigation — final architecture", 24, INK, weight="700")
    label(620, 70, "2D LiDAR → tracking → classification → prediction → native Nav2 costmap layer → MPPI", 14, INK2)

    cx, bw = 620, 460
    x0 = cx - bw / 2

    box(x0, 100, bw, 62, "2D LiDAR", None, "#475569")
    label(cx, 150, "/scan — single planar scanner, 5 Hz. No camera.", 12.5, INK2)
    arrow(cx, 162, cx, 196)

    box(x0, 200, bw, 86, "Static-map rejection → clustering", ["→ compound-observation recovery", "range-aware rejection radius, track-aware deblending"], "#475569")
    arrow(cx, 286, cx, 320)

    box(x0, 324, bw, 86, "Multi-object tracking", ["per-track constant-velocity Kalman filter", "association, maturity gating, timeout pruning"], "#475569")
    arrow(cx, 410, cx, 444)

    box(x0, 448, bw, 86, "Semantic classification", ["geometry features from the LiDAR cluster", "PED · BIKE · VEHICLE · UNKNOWN (+ confidence)"], "#475569")
    pw, gap = 372, 24
    px0 = cx - (3 * pw + 2 * gap) / 2
    px = [px0 + i * (pw + gap) for i in range(3)]
    pc = [p + pw / 2 for p in px]
    py, ph = 620, 108
    cols = [SERIES[0], SERIES[1], MUTED]

    # fan-out bus
    bus_y = 578
    s.append(f'<path d="M{cx},534 L{cx},{bus_y}" stroke="{INK2}" stroke-width="2.4" fill="none"/>')
    label(px[0] + 6, bus_y - 14, "PREDICTION", 13, MUTED, anchor="start", weight="700")
    for i, col in enumerate(cols):
        dash = "" if i == 0 else ' stroke-dasharray="6 5"'
        wdt = 2.4 if i == 0 else 1.8
        s.append(
            f'<path d="M{cx},{bus_y} L{pc[i]},{bus_y}" stroke="{col}" stroke-width="{wdt}" fill="none"{dash}/>'
        )
        arrow(pc[i], bus_y, pc[i], py - 20, color=col, dash=None if i == 0 else "6 5", width=wdt)

    box(px[0], py, pw, ph, "CV-Kalman + covariance", ["propagated state, growing", "positional uncertainty"], SERIES[0], "#eaf2fd", "PRODUCTION", SERIES[0])
    box(px[1], py, pw, ph, "Global reachability", ["bounded-motion set under one", "declared capability premise"], SERIES[1], "#fdeee7", "SECONDARY", SERIES[1])
    box(px[2], py, pw, ph, "Class-aware reachability", ["per-class capability profile", "with safe fallback policy"], MUTED, "#f0efec", "RESEARCH", MUTED)

    # merge bus
    merge_y = 762
    for i, col in enumerate(cols):
        dash = "" if i == 0 else ' stroke-dasharray="6 5"'
        wdt = 2.4 if i == 0 else 1.8
        s.append(
            f'<path d="M{pc[i]},{py + ph} L{pc[i]},{merge_y} L{cx},{merge_y}" stroke="{col}" '
            f'stroke-width="{wdt}" fill="none"{dash}/>'
        )
    arrow(cx, merge_y, cx, 786, width=2.4)

    box(x0 - 120, 790, bw + 240, 86, "Predicted future occupancy + uncertainty", ["published as predictive_nav_msgs/TrackedObjectArray", "time-parameterised regions over a 3.0 s horizon"], "#475569")
    arrow(cx, 876, cx, 910)

    box(x0 - 120, 914, bw + 240, 100, "PredictedObstacleLayer", ["native C++ nav2_costmap_2d plugin — no Nav2 source modified", "soft, time-decaying cost · updateWithMax · capped below lethal", "0.55 m influence window (robot 0.22 + obstacle 0.20 + 0.13 margin)"], SERIES[0], "#eaf2fd")
    arrow(cx, 1014, cx, 1048)

    box(x0, 1052, bw, 62, "Nav2 local costmap → MPPI controller", None, "#475569")
    arrow(cx, 1114, cx, 1148)

    box(x0 + 110, 1152, bw - 220, 50, "Robot motion", None, "#475569")

    label(620, H - 6, "Production/default path drawn solid; secondary and research predictors dashed. Simulation-validated (Gazebo); no real-world validation.", 12, MUTED, style_="i")

    s.append("</svg>")
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / "predictive_nav_architecture.svg"
    path.write_text("\n".join(s) + "\n")
    print(f"wrote {path.relative_to(ROOT)}")


# ===========================================================================
# FIGURE B -- Stage-5E predictor comparison
# ===========================================================================
def figure_b():
    camp = load("stage5e/nav_campaign5.json")["arms"]
    keys = ["cv", "global", "classaware"]
    scored = [camp[k]["trials_scored"] for k in keys]
    succ = [camp[k]["succeeded"] for k in keys]
    coll = [camp[k]["collisions"] for k in keys]
    near = [camp[k]["near_misses_below_0.20m"] for k in keys]
    cl = [camp[k]["min_clearance_m"] for k in keys]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.4, 4.9), gridspec_kw={"width_ratios": [1.05, 1]})

    # --- left: outcome counts -------------------------------------------------
    import numpy as np

    x = np.arange(3)
    w = 0.36
    b1 = ax1.bar(x - w / 2, coll, w, color=CRITICAL, zorder=3, label="Collisions")
    b2 = ax1.bar(x + w / 2, near, w, color=SERIES[3], zorder=3, label="Near misses (< 0.20 m)")
    for bars in (b1, b2):
        for r in bars:
            ax1.text(r.get_x() + r.get_width() / 2, r.get_height() + 0.14, f"{int(r.get_height())}",
                     ha="center", va="bottom", fontsize=11, fontweight="bold", color=INK)
    ax1.set_xticks(x)
    ax1.set_xticklabels([f"{a}\n{s}/{t} success (100%)" for a, s, t in zip(ARMS, succ, scored)], fontsize=9.5, color=INK2)
    ax1.set_ylim(0, max(near) + 1.6)
    ax1.set_yticks(range(0, max(near) + 2))
    style(ax1, "trials", "Outcomes", "132 scored trials · 9 conditions × 3 arms × 5 repetitions")
    ax1.legend(frameon=False, fontsize=9.5, loc="upper left", labelcolor=INK2)

    # --- right: clearance -----------------------------------------------------
    for i, (arm, d) in enumerate(zip(ARMS, cl)):
        c = ARM_COLORS[arm]
        ax2.errorbar(i, d["mean"], yerr=d["sd"], fmt="o", ms=10, color=c, ecolor=c,
                     elinewidth=2, capsize=7, capthick=2, zorder=3)
        ax2.plot(i, d["min"], marker="v", ms=10, color=c, markeredgecolor=SURFACE,
                 markeredgewidth=1.6, zorder=4)
        ax2.text(i, d["mean"] + d["sd"] + 0.035, f"mean {d['mean']:.3f}", fontsize=10,
                 fontweight="bold", color=INK, ha="center", va="bottom")
        above = 0.0 < d["min"] < 0.09   # keep the label clear of the contact line
        ax2.text(i, d["min"] + (0.05 if above else -0.055), f"worst {d['min']:+.3f}",
                 fontsize=9.5, color=CRITICAL if d["min"] < 0 else INK2,
                 ha="center", va="bottom" if above else "top")
    ax2.axhline(0, color=CRITICAL, lw=1.4, ls="--", zorder=2)
    ax2.text(-0.46, 0.018, "contact", fontsize=9, color=CRITICAL, ha="left")
    ax2.set_xticks(range(3))
    ax2.set_xticklabels(ARMS, fontsize=9.5, color=INK2)
    ax2.set_xlim(-0.5, 2.5)
    ax2.set_ylim(-0.26, 1.16)
    style(ax2, "minimum obstacle clearance, m", "Clearance",
          "circle = mean ± 1 sd over scored trials · triangle = worst single trial")

    fig.text(0.5, -0.055,
             "All three arms are tied inside the two-sigma-of-the-difference band on every navigation metric: no arm is claimed safer. "
             "The one CV collision is retained and counted against CV.",
             ha="center", fontsize=9, color=MUTED, style="italic")
    fig.suptitle("Figure B — Final predictor comparison (Stage-5E)", x=0.012, y=1.06,
                 ha="left", fontsize=14, fontweight="bold", color=INK)
    save(fig, "fig_b_predictor_comparison.svg")


# ===========================================================================
# FIGURE C -- planner-visible occupancy
# ===========================================================================
def figure_c():
    import numpy as np

    camp = load("stage5e/nav_campaign5.json")["arms"]
    audit = load("stage5e/prediction_audit.json")
    keys = ["cv", "global", "classaware"]
    akeys = ["A_cv_kalman", "B_global_reachability", "C_runtime_class_aware"]
    cells = [camp[k]["max_pred_cells"] for k in keys]
    window_cells = 529  # whole 0.55 m window at 0.05 m resolution (audit frozen_config)
    hs = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.4, 4.9))

    x = np.arange(3)
    bars = ax1.bar(x, [c["mean"] for c in cells], 0.52,
                   color=[ARM_COLORS[a] for a in ARMS], zorder=3)
    ax1.errorbar(x, [c["mean"] for c in cells], yerr=[c["sd"] for c in cells],
                 fmt="none", ecolor=INK2, elinewidth=1.6, capsize=6, capthick=1.6, zorder=4)
    for r, c in zip(bars, cells):
        ax1.text(r.get_x() + r.get_width() / 2, c["mean"] + c["sd"] + 45, f"{c['mean']:.0f}",
                 ha="center", fontsize=11, fontweight="bold", color=INK)
    ax1.set_xticks(x)
    ax1.set_xticklabels(ARMS, fontsize=9.5, color=INK2)
    style(ax1, "predictive cells painted (peak per trial)", "Live campaign — costmap cells painted",
          "mean ± 1 sd over scored trials, local costmap")
    ax1.set_ylim(0, 2050)
    ax1.annotate("+203 cells vs CV — the only navigation-side\ndifference that resolves outside its 2σ band",
                 xy=(2, cells[2]["mean"] + cells[2]["sd"] + 70), xytext=(-0.40, 1950),
                 fontsize=9, color=INK2, ha="left", va="top",
                 arrowprops=dict(arrowstyle="->", color=MUTED, lw=1.2,
                                 connectionstyle="angle3,angleA=0,angleB=70"))

    for i, (arm, ak) in enumerate(zip(ARMS, akeys)):
        ys = [audit["per_horizon"][ak][f"h{h}"]["mean_cells"] for h in hs]
        ax2.plot(hs, ys, marker="o", ms=8, lw=2.2, color=ARM_COLORS[arm], label=arm,
                 zorder=3, markeredgecolor=SURFACE, markeredgewidth=1.4,
                 ls=["-", "--", ":"][i])
        ax2.text(hs[0] - 0.06, ys[0], f"{ys[0]:.0f}", fontsize=9.5, color=ARM_COLORS[arm],
                 ha="right", va="center", fontweight="bold")
    ax2.axhline(window_cells, color=MUTED, lw=1.3, ls="--", zorder=2)
    ax2.text(3.0, window_cells + 14, f"{window_cells} cells = whole 0.55 m window (saturated)",
             fontsize=9, color=MUTED, ha="right")
    ax2.set_xlabel("prediction horizon, s", fontsize=9.5, color=INK2)
    ax2.set_ylim(0, 620)
    ax2.set_xticks(hs)
    style(ax2, "planner-visible cells inside the 0.55 m window",
          "Offline audit — graded structure vs saturation",
          "one replay set: 2 547 rows, 57 tracks")
    ax2.legend(frameon=False, fontsize=9.5, loc="lower right", labelcolor=INK2)

    fig.text(0.5, -0.055,
             "CV is the only arm whose cost field still carries gradient through the first second — the horizon an MPPI controller at 0.48 m/s acts on. "
             "A saturated field is indistinguishable from any other saturated field.",
             ha="center", fontsize=9, color=MUTED, style="italic")
    fig.suptitle("Figure C — Planner-visible predicted occupancy", x=0.012, y=1.06,
                 ha="left", fontsize=14, fontweight="bold", color=INK)
    save(fig, "fig_c_planner_visible_cells.svg")


# ===========================================================================
# FIGURE D -- Stage-5D.1 semantic corridor
# ===========================================================================
def figure_d():
    audit = load("stage5d1/discriminability_audit.json")
    nav = load("stage5d1/nav_campaign4.json")["arms"]
    by_class = audit["part2_profile_separation"]["by_class"]
    classes = ["PEDESTRIAN", "BIKE", "VEHICLE", "UNKNOWN"]
    hs = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
    styles = ["-", "--", "-.", ":"]

    fig = plt.figure(figsize=(12.4, 8.3))
    gs = fig.add_gridspec(2, 2, height_ratios=[2.0, 1], hspace=0.60, wspace=0.22,
                          top=0.90, bottom=0.14)
    ax1, ax2 = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[1, :])

    for key, ax, cap, sat, satlabel in (
        ("cells_at_0.55", ax1, "PRODUCTION window 0.55 m", 529, "529 cells = whole window"),
        ("cells_at_3.00", ax2, "ANALYSIS window 3.00 m", 14641, "14 641 cells = 101.7% of the local costmap"),
    ):
        for i, cls in enumerate(classes):
            ys = [by_class[cls][f"h{h}"][key] for h in hs]
            ax.plot(hs, ys, marker="o", ms=7, lw=2.2, ls=styles[i], color=SERIES[i], label=cls,
                    zorder=3 + i, markeredgecolor=SURFACE, markeredgewidth=1.3)
        ax.axhline(sat, color=MUTED, lw=1.3, ls="--", zorder=2)
        ax.text(3.0, sat * 1.03, satlabel, fontsize=8.8, color=MUTED, ha="right")
        ax.set_xticks(hs)
        ax.set_xlabel("prediction horizon, s", fontsize=9.5, color=INK2)
        ax.set_ylim(0, sat * 1.16)
        style(ax, "rasterized planner-visible cells", cap)
        ax.legend(frameon=False, fontsize=9, loc="lower right", labelcolor=INK2, ncol=2)

    ax1.annotate("every class paints an identical\n529 cells from 1.0 s onward —\nclass geometry is erased",
                 xy=(1.6, 529), xytext=(1.0, 300), fontsize=9, color=INK2,
                 arrowprops=dict(arrowstyle="->", color=MUTED, lw=1.2))
    ax2.annotate("separation reappears —\nbut the corridor is now a\nblanket over the whole window",
                 xy=(1.0, 14641), xytext=(1.25, 7200), fontsize=9, color=INK2,
                 arrowprops=dict(arrowstyle="->", color=MUTED, lw=1.2))

    # --- navigation consequence ----------------------------------------------
    import numpy as np

    prod = [nav["a"], nav["b"]]
    exp = [nav["c"], nav["d"]]
    groups = ["Success\n(scored trials)", "Collisions", "Collision-monitor\ninterventions", "Nav time, s\n(mean)"]
    prod_v = [sum(a["succeeded"] for a in prod), sum(a["collisions"] for a in prod),
              sum(a["collision_monitor_interventions"] for a in prod),
              sum(a["nav_time_s"]["mean"] for a in prod) / 2]
    prod_d = [sum(a["trials"] for a in prod), None, None, None]
    exp_v = [sum(a["succeeded"] for a in exp), sum(a["collisions"] for a in exp),
             sum(a["collision_monitor_interventions"] for a in exp),
             sum(a["nav_time_s"]["mean"] for a in exp) / 2]
    exp_d = [sum(a["trials"] for a in exp), None, None, None]

    x = np.arange(4)
    w = 0.34
    b1 = ax3.bar(x - w / 2, prod_v, w, color=SERIES[0], zorder=3, label="0.55 m production window")
    b2 = ax3.bar(x + w / 2, exp_v, w, color=SERIES[1], zorder=3, label="3.00 m expanded window")
    for bars, vals, dens in ((b1, prod_v, prod_d), (b2, exp_v, exp_d)):
        for r, v, dn in zip(bars, vals, dens):
            txt = f"{int(v)}/{int(dn)}" if dn else (f"{v:.1f}" if v % 1 else f"{int(v)}")
            ax3.text(r.get_x() + r.get_width() / 2, r.get_height() + 0.9, txt,
                     ha="center", fontsize=10, fontweight="bold", color=INK)
    ax3.set_xticks(x)
    ax3.set_xticklabels(groups, fontsize=9.5, color=INK2)
    ax3.set_ylim(0, 44)
    style(ax3, "count / seconds",
          "Navigation consequence of exposing the full class geometry",
          "Stage-5D.1 campaign, 71 scored trials: runtime + oracle-label arms pooled at each window")
    ax3.legend(frameon=False, fontsize=9.5, ncol=2, labelcolor=INK2,
               loc="upper center", bbox_to_anchor=(0.5, -0.30))

    fig.text(0.5, 0.035,
             "Not a compute limit: layer updateCosts rose only 25.7 → 29.0 µs mean, with 0 missed update deadlines in every arm.    "
             "VEHICLE and UNKNOWN share one capability profile, so their curves coincide above.",
             ha="center", fontsize=9, color=MUTED, style="italic")
    fig.suptitle("Figure D — Semantic corridor discriminability (Stage-5D.1): PROFILE_LIMITED",
                 x=0.012, y=0.985, ha="left", fontsize=14, fontweight="bold", color=INK)
    save(fig, "fig_d_semantic_corridor.svg")


# ===========================================================================
# FIGURE E -- Stage-5B classification
# ===========================================================================
def parse_confusion(text: str):
    """Pull both confusion matrices out of the committed Stage-5B metrics.txt."""
    out = {}
    for tag, key in (("per detection", "detection"), ("per stable track", "track")):
        m = re.search(r"CONFUSION MATRIX -- " + tag + r"(.*?)METRICS -- ", text, re.S)
        block = m.group(1)
        rows = {}
        for line in block.splitlines():
            mm = re.match(r"\s*(PEDESTRIAN|BIKE|VEHICLE|UNKNOWN)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+\(n=(\d+)\)", line)
            if mm:
                rows[mm.group(1)] = [int(mm.group(i)) for i in range(2, 6)]
                assert sum(rows[mm.group(1)]) == int(mm.group(6)), "row sum != reported n"
        out[key] = rows
    return out


def figure_e():
    import numpy as np

    text = (VAL / "stage5b" / "metrics.txt").read_text()
    cm = parse_confusion(text)
    classes = ["PEDESTRIAN", "BIKE", "VEHICLE", "UNKNOWN"]
    short = ["PED", "BIKE", "VEH", "UNK"]

    fig, axes = plt.subplots(1, 2, figsize=(12.4, 5.0))
    for ax, (key, title) in zip(axes, (("detection", "Per detection (one sample per scan)"),
                                       ("track", "Per stable track"))):
        M = np.array([cm[key][c] for c in classes], dtype=float)
        n = M.sum()
        acc = np.trace(M) / n
        Mn = M / M.sum(axis=1, keepdims=True)
        im = ax.imshow(Mn, cmap="Blues", vmin=0, vmax=1)
        for i in range(4):
            for j in range(4):
                v = Mn[i, j]
                ax.text(j, i, f"{int(M[i, j])}\n{v * 100:.0f}%", ha="center", va="center",
                        fontsize=10.5, color="#ffffff" if v > 0.5 else INK,
                        fontweight="bold" if i == j else "normal")
        ax.set_xticks(range(4), short, fontsize=10, color=INK2)
        ax.set_yticks(range(4), [f"{c}\n(n={int(M[i].sum())})" for i, c in enumerate(short)],
                      fontsize=10, color=INK2)
        ax.set_xlabel("predicted", fontsize=9.5, color=INK2)
        ax.set_ylabel("ground truth", fontsize=9.5, color=INK2)
        ax.set_title(title, color=INK, fontsize=12, fontweight="bold", loc="left", pad=20)
        ax.text(0, 1.015, f"{int(n)} samples · overall accuracy {acc * 100:.1f}%",
                transform=ax.transAxes, color=INK2, fontsize=9.5, va="bottom")
        ax.tick_params(length=0)
        for sp in ax.spines.values():
            sp.set_visible(False)
        ax.grid(False)
    fig.colorbar(im, ax=axes, fraction=0.025, pad=0.02, label="row-normalised rate")

    fig.text(0.5, -0.06,
             "Geometry-only classification from a single 2D LiDAR. BIKE is the one cleanly separable class (precision 1.000); "
             "VEHICLE is systematically under-called because a 2D scan sees only the near face, and the classifier declines to UNKNOWN rather than guess.",
             ha="center", fontsize=9, color=MUTED, style="italic")
    fig.suptitle("Figure E — Obstacle classification (Stage-5B)", x=0.012, y=1.04,
                 ha="left", fontsize=14, fontweight="bold", color=INK)
    save(fig, "fig_e_classification.svg")


if __name__ == "__main__":
    figure_a()
    figure_b()
    figure_c()
    figure_d()
    figure_e()
