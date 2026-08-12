#!/usr/bin/env python3
import argparse
import csv
import os
from pathlib import Path

import cv2
import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks


def load_map(yaml_path):
    yaml_path = Path(yaml_path).expanduser().resolve()
    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)
    image_path = Path(meta["image"])
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    img = np.asarray(Image.open(image_path).convert("L"))
    negate = int(meta.get("negate", 0))
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    p_occ = img.astype(np.float32) / 255.0 if negate else 1.0 - img.astype(np.float32) / 255.0
    occupied = p_occ >= occupied_thresh
    return occupied, float(meta["resolution"]), meta, image_path


def normal_profile(occupied, x0, y0, x1, y1, half_length_px=180, half_normal_px=60):
    dx, dy = x1 - x0, y1 - y0
    length = np.hypot(dx, dy)
    if length < 2:
        return None
    tx, ty = dx / length, dy / length
    nx, ny = -ty, tx
    center_x, center_y = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    s = np.arange(-min(half_length_px, int(length * 0.45)), min(half_length_px, int(length * 0.45)) + 1)
    n = np.arange(-half_normal_px, half_normal_px + 1)
    ss, nn = np.meshgrid(s, n, indexing="ij")
    xx = np.rint(center_x + ss * tx + nn * nx).astype(int)
    yy = np.rint(center_y + ss * ty + nn * ny).astype(int)
    valid = (xx >= 0) & (xx < occupied.shape[1]) & (yy >= 0) & (yy < occupied.shape[0])
    values = np.zeros_like(xx, dtype=np.float32)
    values[valid] = occupied[yy[valid], xx[valid]]
    return n, values.mean(axis=0), (center_x, center_y, tx, ty, nx, ny)


def analyze_line(occupied, line, resolution, max_ghost_separation_m):
    x0, y0, x1, y1 = map(int, line)
    half_normal_px = max(4, int(np.ceil(max_ghost_separation_m / resolution)))
    profile_data = normal_profile( occupied, x0, y0, x1, y1, half_normal_px=half_normal_px)
    if profile_data is None:
        return None
    normal_px, profile, geom = profile_data
    smooth = gaussian_filter1d(profile, 1.2)
    prominence = max(0.02, 0.12 * float(smooth.max()))
    peaks, props = find_peaks(smooth, prominence=prominence, distance=max(2, round(0.03 / resolution)))
    peaks = peaks[smooth[peaks] >= 0.18 * smooth.max()]
    if len(peaks) == 0:
        return None
    order = peaks[np.argsort(smooth[peaks])[::-1]]
    primary = int(order[0])
    secondary = None
    for p in order[1:]:
        if abs(normal_px[p] - normal_px[primary]) * resolution >= 0.04:
            secondary = int(p)
            break
    weights = profile.clip(min=0)
    if weights.sum() > 0:
        mean = np.sum(normal_px * weights) / weights.sum()
        std_m = np.sqrt(np.sum(((normal_px - mean) * resolution) ** 2 * weights) / weights.sum())
        cdf = np.cumsum(weights) / weights.sum()
        q025 = normal_px[np.searchsorted(cdf, 0.025)] * resolution
        q975 = normal_px[np.searchsorted(cdf, 0.975)] * resolution
        t95_m = q975 - q025
    else:
        std_m = t95_m = float("nan")
    length_m = np.hypot(x1 - x0, y1 - y0) * resolution
    ghost_sep_m = abs(normal_px[secondary] - normal_px[primary]) * resolution if secondary is not None else 0.0
    return {
        "x0_px": x0, "y0_px": y0, "x1_px": x1, "y1_px": y1,
        "length_m": length_m,
        "wall_t95_m": t95_m,
        "normal_std_m": std_m,
        "num_profile_peaks": int(len(peaks)),
        "ghost_separation_m": ghost_sep_m,
        "ghost_detected": int(secondary is not None),
        "profile_normal_m": (normal_px * resolution).tolist(),
        "profile_occupancy": profile.tolist(),
        "geom": geom,
    }


def draw_results(occupied, results, resolution, out_path):
    canvas = cv2.cvtColor((~occupied).astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)
    for i, r in enumerate(results):
        color = (0, 0, 255) if r["ghost_detected"] else (0, 180, 0)
        p0, p1 = (r["x0_px"], r["y0_px"]), (r["x1_px"], r["y1_px"])
        cv2.line(canvas, p0, p1, color, 2)
        label = f"{i}: {r['ghost_separation_m']*100:.1f}cm"
        cv2.putText(canvas, label, p0, cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA)
    cv2.imwrite(str(out_path), canvas)


def main():
    ap = argparse.ArgumentParser(description="Detect double-wall / wall-thickening artifacts from a ROS occupancy-grid map.")
    ap.add_argument("--map-yaml", required=True, help="Path to map_saver_cli YAML, e.g. ~/go2_ros2/src/man_mapping/runs/run1.yaml")
    ap.add_argument("--out-dir", default="wall_analysis", help="Directory for CSV, annotated PNG, and profiles")
    ap.add_argument("--min-wall-length", type=float, default=1.0, help="Minimum candidate line length in metres")
    ap.add_argument( "--max-ghost-separation", type=float, default=0.50, help="Only search for secondary wall bands within this distance in metres")
    args = ap.parse_args()

    occupied, resolution, meta, image_path = load_map(args.map_yaml)
    out_dir = Path(args.out_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)

    mask = (occupied.astype(np.uint8) * 255)
    edges = cv2.Canny(mask, 40, 120)
    min_len_px = max(10, round(args.min_wall_length / resolution))
    lines = cv2.HoughLinesP(edges, 1, np.pi / 360, threshold=max(25, min_len_px // 3), minLineLength=min_len_px, maxLineGap=max(3, round(0.15 / resolution)))
    if lines is None:
        raise SystemExit("No candidate walls found. Check the YAML path, occupancy thresholds, or lower --min-wall-length.")

    lines = np.asarray(lines)

    if lines.ndim == 3 and lines.shape[1:] == (1, 4):
        lines = lines[:, 0, :]
    elif lines.ndim != 2 or lines.shape[1] != 4:
        raise RuntimeError(f"Unexpected HoughLinesP output shape: {lines.shape}")



    results = []
    for line in lines:
        r = analyze_line(occupied, line, resolution, args.max_ghost_separation)
        if r and r["length_m"] >= args.min_wall_length:
            results.append(r)
    results.sort(key=lambda r: r["length_m"], reverse=True)

    fields = ["x0_px", "y0_px", "x1_px", "y1_px", "length_m", "wall_t95_m", "normal_std_m", "num_profile_peaks", "ghost_separation_m", "ghost_detected"]
    with open(out_dir / "wall_metrics.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for r in results:
            writer.writerow({k: r[k] for k in fields})

    draw_results(occupied, results, resolution, out_dir / "annotated_walls.png")
    with open(out_dir / "summary.txt", "w") as f:
        f.write(f"map_yaml: {Path(args.map_yaml).expanduser()}\nimage: {image_path}\nresolution_m_per_px: {resolution}\n")
        f.write(f"candidate_wall_segments: {len(results)}\n")
        if results:
            total = sum(r["length_m"] for r in results)
            ghost = sum(r["length_m"] for r in results if r["ghost_detected"])
            weighted_t95 = sum(r["length_m"] * r["wall_t95_m"] for r in results) / total
            f.write(f"length_weighted_t95_cm: {weighted_t95 * 100:.2f}\n")
            f.write(f"ghost_wall_length_ratio: {ghost / total:.3f}\n")
            f.write("\nTop segments by ghost separation:\n")
            for i, r in enumerate(sorted(results, key=lambda q: q["ghost_separation_m"], reverse=True)[:15]):
                f.write(f"{i}: length={r['length_m']:.2f}m t95={r['wall_t95_m']*100:.1f}cm peaks={r['num_profile_peaks']} ghost_sep={r['ghost_separation_m']*100:.1f}cm\n")

    print(f"Wrote {out_dir / 'summary.txt'}")
    print(f"Wrote {out_dir / 'wall_metrics.csv'}")
    print(f"Wrote {out_dir / 'annotated_walls.png'}")


if __name__ == "__main__":
    main()
