#!/usr/bin/env bash
set -euo pipefail
BASE_FRAME="${1:-base_link}"
IMU_FRAME="${2:-utlidar_imu}"
LIDAR_FRAME="${3:-utlidar_lidar}"
IMU_TOPIC="${4:-/imu/data}"
DURATION="${5:-15}"
OUT="alignment_check.txt"
TMPDIR=$(mktemp -d)
trap 'rm -rf "$TMPDIR"' EXIT

echo "Alignment check started at $(date)" > "$OUT"
echo "Base frame: $BASE_FRAME" >> "$OUT"
echo "IMU frame: $IMU_FRAME" >> "$OUT"
echo "LiDAR frame: $LIDAR_FRAME" >> "$OUT"
echo "IMU topic: $IMU_TOPIC" >> "$OUT"
echo >> "$OUT"

echo "=== TF: base -> imu ===" >> "$OUT"
timeout 3s ros2 run tf2_ros tf2_echo "$BASE_FRAME" "$IMU_FRAME" >> "$OUT" 2>&1 || true
echo >> "$OUT"
echo "=== TF: base -> lidar ===" >> "$OUT"
timeout 3s ros2 run tf2_ros tf2_echo "$BASE_FRAME" "$LIDAR_FRAME" >> "$OUT" 2>&1 || true
echo >> "$OUT"
echo "=== IMU samples ===" >> "$OUT"
timeout "$DURATION"s ros2 topic echo "$IMU_TOPIC" > "$TMPDIR/imu_raw.txt" 2>/dev/null || true
head -n 160 "$TMPDIR/imu_raw.txt" >> "$OUT"

python3 - <<'PY' "$TMPDIR/imu_raw.txt" "$OUT"
import sys, math, statistics
path, out = sys.argv[1], sys.argv[2]
ax=[]; ay=[]; az=[]; gx=[]; gy=[]; gz=[]
cur=None
for line in open(path,'r',errors='ignore'):
    s=line.rstrip()
    if s.startswith('angular_velocity:'): cur='g'; continue
    if s.startswith('linear_acceleration:'): cur='a'; continue
    if s.startswith('orientation:'): cur='o'; continue
    st=s.strip()
    if cur=='g' and st.startswith('x:'): gx.append(float(st.split(':',1)[1]))
    elif cur=='g' and st.startswith('y:'): gy.append(float(st.split(':',1)[1]))
    elif cur=='g' and st.startswith('z:'): gz.append(float(st.split(':',1)[1]))
    elif cur=='a' and st.startswith('x:'): ax.append(float(st.split(':',1)[1]))
    elif cur=='a' and st.startswith('y:'): ay.append(float(st.split(':',1)[1]))
    elif cur=='a' and st.startswith('z:'): az.append(float(st.split(':',1)[1]))

def mean(v): return statistics.mean(v) if v else float('nan')
with open(out,'a') as f:
    f.write('\n=== Summary ===\n')
    if ax and ay and az:
        mags=[math.sqrt(x*x+y*y+z*z) for x,y,z in zip(ax,ay,az)]
        f.write(f'accel mean xyz: {mean(ax):.6f}, {mean(ay):.6f}, {mean(az):.6f}\n')
        f.write(f'accel |g| mean: {mean(mags):.6f} m/s^2\n')
    if gx and gy and gz:
        f.write(f'gyro mean xyz: {mean(gx):.6f}, {mean(gy):.6f}, {mean(gz):.6f}\n')
    f.write('Interpretation:\n')
    f.write('- While stationary, accel magnitude should be near 9.81 m/s^2.\n')
    f.write('- Gyro means should be near zero.\n')
    f.write('- If IMU data is published in base_link but the hardware axes are not really base-aligned, MOLA can misread gravity direction.\n')
PY

echo >> "$OUT"
echo "Saved to $OUT"
