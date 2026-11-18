#!/usr/bin/env bash
# Ejecuta únicamente el ciclo aprobado, en el contenedor aislado de profiling.
set -euo pipefail
export NUMBA_CACHE_DIR=/tmp/hitl_numba_cache
export MPLCONFIGDIR=/tmp/hitl_mpl_cache
unset SLAM_DISABLE_NUMBA
HITL_INPUT=/worktree/slam/artifacts/profile_20261118/results/cafe_inputs.npz
HITL_OUTPUT=/tmp/hitl_cafe_01
if [[ -e "$HITL_OUTPUT" ]]; then
  echo "Output directory already exists: $HITL_OUTPUT" >&2
  exit 1
fi
mkdir -p "$HITL_OUTPUT"
for specification in \
  'A /tmp/hitl_baseline cafe_compare_8' \
  'B /worktree/slam cafe_compare_8' \
  'C /worktree/slam cafe_compare_16' \
  'D /worktree/slam cafe_compare_16_prior' \
  'E /worktree/slam cafe_compare_16_sample'
do
  read -r case_id source_root profile <<< "$specification"
  export PYTHONPATH="$source_root:/tmp/profile_site057"
  echo "START $case_id $profile $(date -u +%FT%TZ)"
  python3 /worktree/slam/scripts/profile_replay.py \
    --input "$HITL_INPUT" \
    --config "/worktree/slam/config/$profile.yaml" \
    --output "$HITL_OUTPUT/$case_id" > "$HITL_OUTPUT/$case_id.log" 2>&1
  echo "DONE $case_id $(date -u +%FT%TZ)"
done
