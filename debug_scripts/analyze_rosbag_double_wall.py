#!/usr/bin/env python3
import argparse
import csv
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_from_quaternion(q):
    return np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_matrix(t):
    yaw = yaw_from_quaternion(t.transform.rotation)
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, t.transform.translation.x], [s, c, t.transform.translation.y], [0.0, 0.0, 1.0]])


def transform_points(T, points):
    return points @ T[:2, :2].T + T[:2, 2]


def map_to_frame(frame, fixed_frame, dynamic, static):
    T = np.eye(3)
    current = frame
    visited = set()
    while current != fixed_frame:
        if current in visited:
            return None
        visited.add(current)
        item = dynamic.get(current) or static.get(current)
        if item is None:
            return None
        parent, T_parent_child = item
        T = T_parent_child @ T
        current = parent
    return T


def update_transforms(msg, dynamic, static, is_static):
    target = static if is_static else dynamic
    for t in msg.transforms:
        parent = t.header.frame_id.lstrip("/")
        child = t.child_frame_id.lstrip("/")
        if parent and child:
            target[child] = (parent, pose_matrix(t))


def extract_points(bag_path, scan_topic, tf_topic, tf_static_topic, fixed_frame, stride, max_range):
    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap")
    converter = rosbag2_py.ConverterOptions("cdr", "cdr")
    reader.open(storage, converter)
    topic_types = {x.name: x.type for x in reader.get_all_topics_and_types()}
    needed = {scan_topic, tf_topic, tf_static_topic}
    missing = [x for x in needed if x not in topic_types and x != tf_static_topic]
    if missing:
        raise RuntimeError(f"Bag does not contain required topics: {missing}. Available: {sorted(topic_types)}")
    scan_cls = get_message(topic_types[scan_topic])
    tf_cls = get_message(topic_types[tf_topic]) if tf_topic in topic_types else None
    tfs_cls = get_message(topic_types[tf_static_topic]) if tf_static_topic in topic_types else None
    dynamic, static = {}, {}
    points, missing_tf, scans_seen, scans_used = [], 0, 0, 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == tf_topic and tf_cls:
            update_transforms(deserialize_message(data, tf_cls), dynamic, static, False)
            continue
        if topic == tf_static_topic and tfs_cls:
            update_transforms(deserialize_message(data, tfs_cls), dynamic, static, True)
            continue
        if topic != scan_topic:
            continue
        scans_seen += 1
        if scans_seen % stride:
            continue
        scan = deserialize_message(data, scan_cls)
        frame = scan.header.frame_id.lstrip("/")
        T = map_to_frame(frame, fixed_frame, dynamic, static)
        if T is None:
            missing_tf += 1
            continue
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= min(scan.range_max, max_range))
        angles = scan.angle_min + np.flatnonzero(valid) * scan.angle_increment
        xy = np.column_stack((ranges[valid] * np.cos(angles), ranges[valid] * np.sin(angles)))
        if len(xy):
            points.append(transform_points(T, xy))
            scans_used += 1
    if not points:
        raise RuntimeError("No scan points could be transformed into the fixed frame. Verify /tf, /tf_static, frame IDs, and --fixed-frame.")
    return np.vstack(points), scans_seen, scans_used, missing_tf, topic_types


def rasterize(points, resolution, padding):
    lo = points.min(axis=0) - padding
    hi = points.max(axis=0) + padding
    size = np.ceil((hi - lo) / resolution).astype(int) + 1
    grid = np.zeros((size[1], size[0]), dtype=np.uint16)
    ij = np.floor((points - lo) / resolution).astype(int)
    valid = (ij[:, 0] >= 0) & (ij[:, 0] < size[0]) & (ij[:, 1] >= 0) & (ij[:, 1] < size[1])
    np.add.at(grid, (ij[valid, 1], ij[valid, 0]), 1)
    occupied = grid >= max(2, int(np.percentile(grid[grid > 0], 70)))
    return occupied, lo, grid


def profile(occupied, line, half_normal_px, half_length_px):
    x0, y0, x1, y1 = map(float, line)
    dx, dy = x1 - x0, y1 - y0
    length = np.hypot(dx, dy)
    if length < 2:
        return None
    tx, ty = dx / length, dy / length
    nx, ny = -ty, tx
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    s = np.arange(-min(int(length * .45), half_length_px), min(int(length * .45), half_length_px) + 1)
    n = np.arange(-half_normal_px, half_normal_px + 1)
    ss, nn = np.meshgrid(s, n, indexing="ij")
    x = np.rint(cx + ss * tx + nn * nx).astype(int)
    y = np.rint(cy + ss * ty + nn * ny).astype(int)
    valid = (x >= 0) & (x < occupied.shape[1]) & (y >= 0) & (y < occupied.shape[0])
    values = np.zeros_like(x, dtype=np.float32)
    values[valid] = occupied[y[valid], x[valid]]
    return n, values.mean(axis=0)


def analyze_walls(occupied, resolution, min_length_m, max_ghost_m):
    edges = cv2.Canny((occupied.astype(np.uint8) * 255), 40, 120)
    min_len = max(10, round(min_length_m / resolution))
    lines = cv2.HoughLinesP(edges, 1, np.pi / 360, threshold=max(25, min_len // 3), minLineLength=min_len, maxLineGap=max(3, round(.15 / resolution)))
    if lines is None:
        return []
    lines = np.asarray(lines).reshape(-1, 4)
    results = []
    for line in lines:
        length_m = np.hypot(line[2] - line[0], line[3] - line[1]) * resolution
        p = profile(occupied, line, max(4, round(max_ghost_m / resolution)), round(9.0 / resolution))
        if p is None:
            continue
        n, values = p
        smooth = gaussian_filter1d(values, 1.2)
        peaks, _ = find_peaks(smooth, prominence=max(.02, .12 * smooth.max()), distance=max(2, round(.03 / resolution)))
        if len(peaks) == 0:
            continue
        peaks = peaks[smooth[peaks] >= .18 * smooth.max()]
        primary = peaks[np.argmax(smooth[peaks])]
        secondary = next((p for p in peaks[np.argsort(smooth[peaks])[::-1]] if abs(n[p] - n[primary]) * resolution >= .04), None)
        w = values.clip(min=0)
        cdf = np.cumsum(w) / w.sum()
        t95 = (n[np.searchsorted(cdf, .975)] - n[np.searchsorted(cdf, .025)]) * resolution
        results.append({"x0_px": line[0], "y0_px": line[1], "x1_px": line[2], "y1_px": line[3], "length_m": length_m, "wall_t95_m": t95, "num_profile_peaks": len(peaks), "ghost_separation_m": 0.0 if secondary is None else abs(n[secondary] - n[primary]) * resolution, "ghost_detected": int(secondary is not None)})
    return sorted(results, key=lambda x: x["length_m"], reverse=True)


def main():
    ap = argparse.ArgumentParser(description="Offline double-wall analysis from ROS 2 bagged LaserScan + TF.")
    ap.add_argument("--bag", required=True)
    ap.add_argument("--scan-topic", default="/scan")
    ap.add_argument("--tf-topic", default="/tf")
    ap.add_argument("--tf-static-topic", default="/tf_static")
    ap.add_argument("--fixed-frame", default="map")
    ap.add_argument("--out-dir", default="bag_wall_analysis")
    ap.add_argument("--resolution", type=float, default=.05)
    ap.add_argument("--scan-stride", type=int, default=2)
    ap.add_argument("--max-range", type=float, default=12.0)
    ap.add_argument("--min-wall-length", type=float, default=1.0)
    ap.add_argument("--max-ghost-separation", type=float, default=.5)
    args = ap.parse_args()
    out = Path(args.out_dir).expanduser(); out.mkdir(parents=True, exist_ok=True)
    points, seen, used, missing, types = extract_points(Path(args.bag).expanduser(), args.scan_topic, args.tf_topic, args.tf_static_topic, args.fixed_frame, args.scan_stride, args.max_range)
    np.save(out / "registered_scan_endpoints_map.npy", points)
    occupied, origin, density = rasterize(points, args.resolution, padding=1.0)
    cv2.imwrite(str(out / "registered_scan_occupancy.png"), (~occupied).astype(np.uint8) * 255)
    walls = analyze_walls(occupied, args.resolution, args.min_wall_length, args.max_ghost_separation)
    fields = ["x0_px", "y0_px", "x1_px", "y1_px", "length_m", "wall_t95_m", "num_profile_peaks", "ghost_separation_m", "ghost_detected"]
    with open(out / "wall_metrics.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader(); writer.writerows(walls)
    total = sum(w["length_m"] for w in walls)
    ghost = sum(w["length_m"] for w in walls if w["ghost_detected"])
    with open(out / "summary.txt", "w") as f:
        f.write(f"fixed_frame: {args.fixed_frame}\nscan_topic: {args.scan_topic}\nscan_messages_seen: {seen}\nscan_messages_used: {used}\nscans_skipped_missing_tf: {missing}\npoints_used: {len(points)}\nresolution_m_per_px: {args.resolution}\norigin_xy_m: {origin.tolist()}\ncandidate_wall_segments: {len(walls)}\nghost_wall_length_ratio: {ghost / total if total else 0:.3f}\n")
        for i, w in enumerate(sorted(walls, key=lambda r: r["ghost_separation_m"], reverse=True)[:15]):
            f.write(f"{i}: length={w['length_m']:.2f}m t95={w['wall_t95_m']*100:.1f}cm peaks={w['num_profile_peaks']} ghost_sep={w['ghost_separation_m']*100:.1f}cm\n")
    print(f"Wrote {out / 'summary.txt'}")
    print(f"Wrote {out / 'registered_scan_occupancy.png'}")
    print(f"Wrote {out / 'wall_metrics.csv'}")

if __name__ == "__main__":
    main()
