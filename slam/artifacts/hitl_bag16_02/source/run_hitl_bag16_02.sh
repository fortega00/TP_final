#!/usr/bin/env bash
# Ciclo autorizado de 24 partículas; incluye control E16 con carga actual.
set -euo pipefail
export NUMBA_CACHE_DIR=/tmp/hitl_numba_cache
export MPLCONFIGDIR=/tmp/hitl_mpl_cache
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export PYTHONPATH=/tmp/bag16_source/slam:/tmp/profile_site057
unset SLAM_DISABLE_NUMBA
BAG16_SOURCE=/tmp/bag16_source
BAG16_OUTPUT=/tmp/hitl_bag16_02
BAG16_INPUT="$BAG16_OUTPUT/input/bag_mapa2.npz"
if [[ -e "$BAG16_OUTPUT/E16" ]]; then
  echo "Experiment already exists: $BAG16_OUTPUT/E16" >&2
  exit 1
fi
python3 -c 'from slam.core.acceleration import BACKEND; assert BACKEND == "numba", BACKEND; print("Required backend:", BACKEND)'
for specification in \
  'E16 16_sample' \
  'F24 24' \
  'D24 24_prior' \
  'E24 24_sample'
do
  read -r case_id profile <<< "$specification"
  echo "START $case_id $profile $(date -u +%FT%TZ) load=$(cat /proc/loadavg)"
  nice -n 15 python3 "$BAG16_SOURCE/slam/scripts/profile_replay.py" \
    --input "$BAG16_INPUT" \
    --config "$BAG16_SOURCE/slam/config/bag16_compare_$profile.yaml" \
    --output "$BAG16_OUTPUT/$case_id" > "$BAG16_OUTPUT/$case_id.log" 2>&1
  echo "DONE $case_id $(date -u +%FT%TZ) load=$(cat /proc/loadavg)"
done
