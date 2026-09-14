#!/bin/bash
# Runs CV and HYBRID arms of up to two scenarios CONCURRENTLY, so both arms of a
# comparison always experience the same machine contention. Four Gazebo sessions
# at a time on a 20-core host.
#   g6run.sh <out_root> <trials> <scenarioA> [scenarioB]
cd /home/sachin/predictive_nav_ws || exit 1
ROOT=$1; TRIALS=$2; SA=$3; SB=${4:-}
mkdir -p "$ROOT"
run() {  # mode scenario domain
  local mode=$1 sc=$2 dom=$3
  local out="$ROOT/${sc}__${mode}"
  rm -rf "$out"
  ( source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
    source install/setup.bash >/dev/null 2>&1
    timeout 9000 python3 src/predictive_nav_bringup/scripts/stage4g2/run.py \
      --out "$out" --layer-mode "$mode" --scenarios "$sc" \
      --trials "$TRIALS" --domain "$dom" ) > "$ROOT/${sc}__${mode}.log" 2>&1
  echo "FINISHED $sc/$mode rc=$?"
}
pids=()
run cv_covariance "$SA" 100 & pids+=($!)
run hybrid        "$SA" 130 & pids+=($!)
if [ -n "$SB" ]; then
  run cv_covariance "$SB" 160 & pids+=($!)
  run hybrid        "$SB" 190 & pids+=($!)
fi
wait "${pids[@]}"
