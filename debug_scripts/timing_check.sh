#!/usr/bin/env bash
set -euo pipefail
IMU_TOPIC="${1:-/imu/data}"
LIDAR_TOPIC="${2:-/points_raw}"
DURATION="${3:-20}"
OUT="timing_check.txt"
TMPDIR=$(mktemp -d)
trap 'rm -rf "$TMPDIR"' EXIT

echo "Timing check started at $(date)" > "$OUT"
echo "IMU topic: $IMU_TOPIC" >> "$OUT"
echo "LiDAR topic: $LIDAR_TOPIC" >> "$OUT"
echo "Duration: ${DURATION}s" >> "$OUT"
echo >> "$OUT"

timeout "$DURATION"s ros2 topic echo "$IMU_TOPIC" --field header.stamp > "$TMPDIR/imu_stamps.txt" 2>/dev/null || true
timeout "$DURATION"s ros2 topic echo "$LIDAR_TOPIC" --field header.stamp > "$TMPDIR/lidar_stamps.txt" 2>/dev/null || true

python3 - <<'PY' "$TMPDIR/imu_stamps.txt" "$TMPDIR/lidar_stamps.txt" "$OUT"
import sys, statistics

def parse(path):
    vals=[]
    sec=None
    for line in open(path, 'r', errors='ignore'):
        s=line.strip()
        if s.startswith('sec:'): sec=int(s.split(':',1)[1])
        elif s.startswith('nanosec:') and sec is not None:
            ns=int(s.split(':',1)[1])
            vals.append(sec + ns*1e-9)
            sec=None
    return vals

def summarize(name, arr):
    out=[]
    out.append(f'[{name}] samples: {len(arr)}')
    if len(arr) < 2:
        out.append('not enough samples')
        return out
    d=[b-a for a,b in zip(arr, arr[1:])]
    out.append(f'first stamp: {arr[0]:.9f}')
    out.append(f'last stamp:  {arr[-1]:.9f}')
    out.append(f'mean dt: {statistics.mean(d):.6f} s')
    out.append(f'min dt:  {min(d):.6f} s')
    out.append(f'max dt:  {max(d):.6f} s')
    if statistics.mean(d) > 0:
        out.append(f'approx rate: {1.0/statistics.mean(d):.3f} Hz')
    backwards=sum(1 for x in d if x < 0)
    out.append(f'backward jumps: {backwards}')
    return out

imu=parse(sys.argv[1])
lidar=parse(sys.argv[2])
outfile=sys.argv[3]
with open(outfile,'a') as f:
    for line in summarize('IMU', imu): f.write(line+'\n')
    f.write('\n')
    for line in summarize('LiDAR', lidar): f.write(line+'\n')
    f.write('\n')
    if imu and lidar:
        f.write(f'header start delta (imu-lidar): {imu[0]-lidar[0]:.6f} s\n')
        f.write(f'header end delta   (imu-lidar): {imu[-1]-lidar[-1]:.6f} s\n')
PY

echo >> "$OUT"
echo "Saved to $OUT"
