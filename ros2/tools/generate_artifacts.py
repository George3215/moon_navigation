"""Generate all reproduction artifacts: figures, traversal video, VLM dialogue.

Runs offline (no ROS): loads the terrain heightmap, recomputes the three mode
maps, reads the recorded ``runs/*/trajectory.csv`` files, and writes:

    docs/reproduction/figures/*.png   terrain / maps / trajectories / speed
    docs/reproduction/video/traversal.mp4
    docs/reproduction/vlm_dialogue.txt
    docs/reproduction/metrics_summary.md

Usage:  python3 tools/generate_artifacts.py
"""
import csv
import os
import re
import sys

import cv2
import numpy as np

REPO = "/home/lry/mars_navigation"
IMG = REPO + "/docs/picture/terrain/mixed_terrain_hm.png"
RUNS = REPO + "/ros2/runs"
OUT_FIG = REPO + "/docs/reproduction/figures"
OUT_VID = REPO + "/docs/reproduction/video"
OUT_TXT = REPO + "/docs/reproduction"

sys.path.insert(0, REPO + "/ros2/mars_navigation_ros2/mars_navigation_ros2")
from elevation_synthesis import compute_rock_map, compute_traversability_cost, load_height

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import patches

REAL_M = 54.0
RES = 0.2
OX = -27.0
OY = -27.0
HPG = 4.820803273566 / 255.0
MAX_SLOPE = 2.0

MODE_COLORS = {"Mode 1": "#2ecc71", "Mode 2": "#e67e22", "Mode 3": "#e74c3c", "unknown": "#95a5a6"}
MODE_NAMES = {"Mode 1": "Mode 1 (flat / efficient)", "Mode 2": "Mode 2 (rocky / safe)",
              "Mode 3": "Mode 3 (challenging / conservative)", "unknown": "startup"}


def base_map(height):
    """Replicate image_to_map.py accessibility -> cleanup -> 0..100."""
    h, w = height.shape
    acc = np.full((h, w), 255, np.uint8)
    nb = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    ratio = np.tan(np.radians(MAX_SLOPE))
    for i in range(h):
        for j in range(w):
            cur = float(height[i, j])
            for dx, dy in nb:
                ni, nj = i + dx, j + dy
                if 0 <= ni < h and 0 <= nj < w:
                    horiz = RES * (1 if dx == 0 or dy == 0 else np.sqrt(2))
                    if abs(cur - float(height[ni, nj])) / horiz <= ratio:
                        acc[i, j] = 0
                        break
    num, labels = cv2.connectedComponents(acc)
    out = np.zeros_like(acc)
    for lbl in range(1, num):
        if int(np.sum(labels == lbl)) >= 2:
            out[labels == lbl] = 255
    dist = cv2.distanceTransform(255 - out, cv2.DIST_L2, 5)
    connected = ((dist < 1).astype(np.uint8)) * 255
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
    morphed = cv2.erode(cv2.dilate(connected, k), k)
    return np.flip(cv2.normalize(morphed, None, 0, 100, cv2.NORM_MINMAX, cv2.CV_8U), 0).astype(np.float32)


def extent():
    return [OX, OX + REAL_M, OY, OY + REAL_M]


def load_trajectory(name):
    path = os.path.join(RUNS, name, "trajectory.csv")
    if not os.path.exists(path):
        return None
    rows = []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            rows.append((float(r["t"]), float(r["x"]), float(r["y"]), r["mode"]))
    return rows


def load_metrics(name):
    import json
    p = os.path.join(RUNS, name, "metrics.json")
    if not os.path.exists(p):
        return None
    with open(p) as fh:
        return json.load(fh)


def fig_heightmap(height):
    fig, ax = plt.subplots(figsize=(6, 6))
    im = ax.imshow(height, cmap="terrain", extent=extent(), origin="lower")
    ax.set_title("Terrain heightmap (mixed flat / rocky / challenging)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    fig.colorbar(im, label="height (m)")
    return fig


def fig_map(data, title, cmap, vmin, vmax, cbar_label):
    fig, ax = plt.subplots(figsize=(6, 6))
    im = ax.imshow(data, cmap=cmap, extent=extent(), origin="lower", vmin=vmin, vmax=vmax)
    ax.set_title(title)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    fig.colorbar(im, label=cbar_label)
    return fig


def fig_slope_roughness(slope, rough):
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5))
    im0 = axes[0].imshow(slope, cmap="magma", extent=extent(), origin="lower")
    axes[0].set_title("Slope (deg)")
    axes[1].imshow(rough, cmap="viridis", extent=extent(), origin="lower")
    axes[1].set_title("Roughness (m, local std-dev)")
    for ax in axes:
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
    fig.colorbar(im0, ax=axes[0], label="deg")
    return fig


def fig_trajectories(maps, trajs):
    fig, ax = plt.subplots(figsize=(7.5, 7))
    ax.imshow(maps["cost"], cmap="gray_r", extent=extent(), origin="lower",
              vmin=0, vmax=100, alpha=0.85)
    ax.set_title("Recorded traversals (all modes) over elevation cost")
    for name, traj in trajs.items():
        xs = [p[1] for p in traj]
        ys = [p[2] for p in traj]
        color = {"mode1": "#2ecc71", "mode2": "#e67e22", "mode3": "#e74c3c", "vlm": "#3498db"}[name]
        ax.plot(xs, ys, color=color, lw=2, label=f"{name} ({len(xs)} pts)")
        ax.plot(xs[0], ys[0], "o", color=color, ms=8)
        ax.plot(xs[-1], ys[-1], "s", color=color, ms=9)
    ax.plot([], [], "o", color="k", label="start")
    ax.plot([], [], "s", color="k", label="goal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(loc="upper left", fontsize=8)
    return fig


def fig_speed(metrics):
    names = ["mode1", "mode2", "mode3", "vlm"]
    speeds = [metrics[n]["average_speed_mps"] for n in names]
    colors = ["#2ecc71", "#e67e22", "#e74c3c", "#3498db"]
    fig, ax = plt.subplots(figsize=(7, 4.5))
    bars = ax.bar(names, speeds, color=colors)
    for b, s in zip(bars, speeds):
        ax.text(b.get_x() + b.get_width() / 2, s + 0.02, f"{s:.2f} m/s",
                ha="center", va="bottom", fontsize=9)
    ax.set_ylabel("average speed (m/s)")
    ax.set_title("Traversal speed by navigation mode")
    ax.set_ylim(0, max(speeds) * 1.25)
    return fig


def fig_synthetic():
    fig, axes = plt.subplots(1, 3, figsize=(12, 4.2))
    names = ["flat", "rocky", "challenging"]
    for ax, name in zip(axes, names):
        img = cv2.imread(f"{REPO}/docs/picture/perception/synthetic/{name}.png")
        ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        ax.set_title(f"{name} -> { {'flat':'Mode 1','rocky':'Mode 2','challenging':'Mode 3'}[name] }")
        ax.axis("off")
    fig.suptitle("Synthetic terrain images fed to the VLM classifier")
    return fig


def fig_nodegraph():
    fig, ax = plt.subplots(figsize=(10, 7))
    ax.axis("off")
    nodes = {
        "pose_simulator": (0.5, 0.85),
        "image_to_map": (0.5, 0.6),
        "hazard_mapper": (0.25, 0.4),
        "map_server": (0.5, 0.4),
        "global_planner": (0.75, 0.55),
        "global_path_optimizer": (0.9, 0.3),
        "local_planner": (0.25, 0.18),
        "path_follower": (0.5, 0.18),
        "terrain_image_publisher": (0.75, 0.18),
        "vlm_mode": (0.95, 0.18),
    }
    edges = [
        ("pose_simulator", "map_server"), ("pose_simulator", "global_planner"),
        ("pose_simulator", "local_planner"), ("pose_simulator", "path_follower"),
        ("pose_simulator", "hazard_mapper"),
        ("image_to_map", "map_server"), ("hazard_mapper", "map_server"),
        ("map_server", "global_planner"), ("map_server", "local_planner"),
        ("global_planner", "map_server"), ("global_planner", "global_path_optimizer"),
        ("global_path_optimizer", "path_follower"), ("local_planner", "path_follower"),
        ("terrain_image_publisher", "vlm_mode"), ("vlm_mode", "path_follower"),
        ("vlm_mode", "hazard_mapper"), ("vlm_mode", "global_path_optimizer"),
        ("path_follower", "pose_simulator"),
    ]
    for a, b in edges:
        ax.annotate("", xy=nodes[b], xytext=nodes[a],
                    arrowprops=dict(arrowstyle="->", color="#888", lw=1))
    for name, (x, y) in nodes.items():
        ax.add_patch(patches.FancyBboxPatch((x - 0.07, y - 0.035), 0.14, 0.07,
                                            boxstyle="round,pad=0.005", fc="#eaf2f8", ec="#2980b9"))
        ax.text(x, y, name, ha="center", va="center", fontsize=7)
    ax.set_xlim(0, 1.05)
    ax.set_ylim(0, 1)
    ax.set_title("ROS 2 node graph (smoke_stack.launch.py)")
    return fig


def make_video(cost, traj, out_path):
    """Animate the VLM multi-mode traversal over the cost map."""
    fig, ax = plt.subplots(figsize=(7.5, 7))
    ax.imshow(cost, cmap="gray_r", extent=extent(), origin="lower", vmin=0, vmax=100)
    ax.set_title("VLM multi-mode traversal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    xs = [p[1] for p in traj]
    ys = [p[2] for p in traj]
    modes = [p[3] for p in traj]
    (line,) = ax.plot([], [], lw=2, color="#3498db")
    (dot,) = ax.plot([], [], "o", ms=12, color="#3498db")
    # skip to keep ~150 frames
    step = max(1, len(traj) // 150)
    idx = list(range(0, len(traj), step))

    def init():
        line.set_data([], [])
        dot.set_data([], [])
        return line, dot

    def update(i):
        k = idx[i]
        line.set_data(xs[:k + 1], ys[:k + 1])
        dot.set_data([xs[k]], [ys[k]])
        dot.set_color(MODE_COLORS.get(modes[k], "#95a5a6"))
        return line, dot

    ani = animation.FuncAnimation(fig, update, frames=len(idx), init_func=init,
                                  interval=40, blit=True)
    writer = animation.FFMpegWriter(fps=25, bitrate=2000)
    ani.save(out_path, writer=writer)
    plt.close(fig)


def extract_dialogue():
    log = os.path.join(RUNS, "vlm", "stack.log")
    lines = []
    with open(log) as fh:
        for line in fh:
            if "vlm_mode" in line and ("terrain=" in line or "waiting" in line):
                m = re.search(r"\[vlm_mode\]: (.*)", line)
                if m:
                    lines.append(m.group(1))
            if "path_follower" in line and "mode ->" in line:
                m = re.search(r"\[path_follower\]: (.*)", line)
                if m:
                    lines.append("   " + m.group(1))
    return lines


def write_metrics_summary(metrics):
    rows = []
    for name in ["mode1", "mode2", "mode3", "vlm"]:
        m = metrics[name]
        success = "✅" if m["success"] else "❌"
        rows.append(f"| {name} | {success} | {m['reason']} | {m['traversal_time_s']:.2f} s | "
                    f"{m['traversal_distance_m']:.2f} m | {m['average_speed_mps']:.3f} m/s |")
    md = [
        "# Reproduction metrics",
        "",
        "Start (-20.0, 0.0) -> goal (20.0, 0.0), arrival radius 2.0 m.",
        "",
        "A slope-based rollover model differentiates the modes by *capability*",
        "(Mode 1 limit 8 deg, Mode 2 limit 12 deg, Mode 3 limit 40 deg).  The",
        "~15.8 deg ridge is the capability-differentiating terrain: Mode 1 and",
        "Mode 2 roll over on it, while Mode 3 -- and the VLM multi-mode system,",
        "which switches to Mode 3 before the ridge -- cross it successfully.",
        "",
        "| run | success | reason | time | distance | avg speed |",
        "| --- | --- | --- | --- | --- | --- |",
    ] + rows + [
        "",
        "## VLM multi-mode per-mode breakdown",
        "",
        "```",
    ]
    pm = metrics["vlm"]["per_mode"]
    for mode, e in sorted(pm.items()):
        if mode == "unknown":
            continue
        md.append(f"  {mode:<10} time={e['time']:.2f}s ({e['fraction_time']*100:.0f}%)  "
                  f"distance={e['distance']:.2f}m ({e['fraction_distance']*100:.0f}%)")
    md.append("```")
    return "\n".join(md)


def main():
    for d in (OUT_FIG, OUT_VID, OUT_TXT):
        os.makedirs(d, exist_ok=True)

    height = load_height(IMG, REAL_M, RES, HPG)
    base = base_map(height)
    rock_residual, rock = compute_rock_map(height)
    # Match hazard_mapper's roughness_kernel (3 px = 0.6 m window, faithful to the
    # CuPy plugin's 0.5 m window at 0.1 m grid).  The function's default (5 px =
    # 1.0 m) over-inflates the ridge-core roughness and would draw a cost >= 88
    # hard obstacle across the very ridge Mode 3 crosses.
    slope, rough, cost = compute_traversability_cost(height, RES, roughness_kernel=3)
    maps = {"base": base, "rock": rock, "cost": cost, "slope": slope, "rough": rough}

    trajs = {}
    for name in ["mode1", "mode2", "mode3", "vlm"]:
        trajs[name] = load_trajectory(name)
    metrics = {name: load_metrics(name) for name in ["mode1", "mode2", "mode3", "vlm"]}

    figs = {
        "fig01_terrain_heightmap.png": fig_heightmap(height),
        "fig02_base_map.png": fig_map(base, "Mode 1 base accessibility map", "gray_r", 0, 100, "cost (0..100)"),
        "fig03_rock_map.png": fig_map(rock, "Mode 2 rock/obstacle map", "Reds", 0, 100, "obstacle"),
        "fig04_cost_map.png": fig_map(cost, "Mode 3 elevation cost map", "magma", 0, 100, "traversal cost"),
        "fig05_slope_roughness.png": fig_slope_roughness(slope, rough),
        "fig06_trajectories.png": fig_trajectories(maps, trajs),
        "fig07_speed_comparison.png": fig_speed(metrics),
        "fig08_synthetic_terrain.png": fig_synthetic(),
        "fig09_node_graph.png": fig_nodegraph(),
    }
    for fname, fig in figs.items():
        fig.savefig(os.path.join(OUT_FIG, fname), dpi=130, bbox_inches="tight")
        plt.close(fig)
        print("wrote", f"figures/{fname}")

    if trajs["vlm"]:
        make_video(maps["cost"], trajs["vlm"], os.path.join(OUT_VID, "traversal.mp4"))
        print("wrote", "video/traversal.mp4")

    dialogue = extract_dialogue()
    with open(os.path.join(OUT_TXT, "vlm_dialogue.txt"), "w") as fh:
        fh.write("VLM terrain classification -> navigation mode (from runs/vlm/stack.log)\n")
        fh.write("=" * 72 + "\n")
        fh.write("\n".join(dialogue))
    print("wrote", "vlm_dialogue.txt", f"({len(dialogue)} lines)")

    with open(os.path.join(OUT_TXT, "metrics_summary.md"), "w") as fh:
        fh.write(write_metrics_summary(metrics))
    print("wrote", "metrics_summary.md")

    print("\nArtifacts written under", REPO + "/docs/reproduction/")


if __name__ == "__main__":
    main()
