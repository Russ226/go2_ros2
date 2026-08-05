#!/usr/bin/env bash
set -euo pipefail
LOCAL_PARENT="${1:-odom}"
LOCAL_CHILD="${2:-base_link}"
GLOBAL_PARENT="${3:-map}"
GLOBAL_CHILD="${4:-odom}"
DURATION="${5:-60}"
OUT="stationary_drift_check.txt"
TMPDIR=$(mktemp -d)
trap 'rm -rf "$TMPDIR"' EXIT

sample_tf() {
  local parent="$1"
  local child="$2"
  local outfile="$3"
  local end_ts=$(( $(date +%s) + DURATION ))
  : > "$outfile"
for i in $(seq 1 "$DURATION"); do
  {
    echo '---'
    date +%s.%N
    timeout 2s ros2 run tf2_ros tf2_echo "$parent" "$child" | head -n 12 || echo "no sample"
  } >> "$outfile" || true
  sleep 1
done
}

echo "Stationary drift check started at $(date)" > "$OUT"
echo "Local TF:  $LOCAL_PARENT -> $LOCAL_CHILD" >> "$OUT"
echo "Global TF: $GLOBAL_PARENT -> $GLOBAL_CHILD" >> "$OUT"
echo "Duration: ${DURATION}s" >> "$OUT"
echo >> "$OUT"

sample_tf "$LOCAL_PARENT" "$LOCAL_CHILD" "$TMPDIR/local.txt"
sample_tf "$GLOBAL_PARENT" "$GLOBAL_CHILD" "$TMPDIR/global.txt"

python3 - <<'PY' "$TMPDIR/local.txt" "$TMPDIR/global.txt" "$OUT"
import sys, math, re

def parse(path):
    entries=[]
    cur={}
    for line in open(path,'r',errors='ignore'):
        s=line.strip()
        if re.fullmatch(r'\d+\.\d+', s):
            cur={'wall': float(s)}
        elif s.startswith('- Translation:'):
            vals=re.findall(r'[-+]?\d+\.\d+', s)
            if len(vals)==3: cur['t']=tuple(map(float,vals))
        elif 'Rotation: in RPY (degree)' in s:
            vals=re.findall(r'[-+]?\d+\.\d+', s)
            if len(vals)==3:
                cur['rpy']=tuple(map(float,vals))
                if 't' in cur: entries.append(cur)
    return entries

def summarize(name, arr):
    if len(arr) < 2:
        return [f'[{name}] not enough samples']
    t0=arr[0]['t']; t1=arr[-1]['t']
    r0=arr[0]['rpy']; r1=arr[-1]['rpy']
    dt=math.sqrt(sum((b-a)**2 for a,b in zip(t0,t1)))
    dr=tuple(b-a for a,b in zip(r0,r1))
    return [
        f'[{name}] samples: {len(arr)}',
        f'start translation: {t0}',
        f'end translation:   {t1}',
        f'total translation drift: {dt:.6f} m',
        f'start rpy deg: {r0}',
        f'end rpy deg:   {r1}',
        f'total rpy change deg: ({dr[0]:.3f}, {dr[1]:.3f}, {dr[2]:.3f})'
    ]

local=parse(sys.argv[1])
global_=parse(sys.argv[2])
out=sys.argv[3]
with open(out,'a') as f:
    for line in summarize('odom->base_link', local): f.write(line+'\n')
    f.write('\n')
    for line in summarize('map->odom', global_): f.write(line+'\n')
    f.write('\nInterpretation:\n')
    f.write('- Local drift while stationary points to odometry/fusion instability.\n')
    f.write('- Global drift with low local drift points to map correction sliding around.\n')
    f.write('- Large drift in both usually means the root problem is upstream (timing, IMU alignment, or weak ICP constraints).\n')
    f.write('\n=== Raw local samples ===\n')
    f.write(open(sys.argv[1],'r',errors='ignore').read())
    f.write('\n=== Raw global samples ===\n')
    f.write(open(sys.argv[2],'r',errors='ignore').read())
PY

echo "Saved to $OUT"
