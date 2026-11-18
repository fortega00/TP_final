#!/usr/bin/env bash
# Cinco replays del núcleo autorizados por el usuario, sin nodos ROS.
# Ejecutar dentro del contenedor aislado con los fuentes copiados en /tmp/bag16_source.
set -euo pipefail
export NUMBA_CACHE_DIR=/tmp/hitl_numba_cache
export MPLCONFIGDIR=/tmp/hitl_mpl_cache
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
unset SLAM_DISABLE_NUMBA
BAG16_SOURCE=/tmp/bag16_source
BAG16_OUTPUT=/tmp/hitl_bag16_01
BAG16_INPUT="$BAG16_OUTPUT/input/bag_mapa2.npz"
if [[ -e "$BAG16_OUTPUT/A" ]]; then
  echo "Experiment already exists: $BAG16_OUTPUT/A" >&2
  exit 1
fi
for specification in \
  'A /tmp/hitl_baseline 8' \
  'B /tmp/bag16_source/slam 8' \
  'C /tmp/bag16_source/slam 16' \
  'D /tmp/bag16_source/slam 16_prior' \
  'E /tmp/bag16_source/slam 16_sample'
do
  read -r case_id source_root profile <<< "$specification"
  export PYTHONPATH="$source_root:/tmp/profile_site057"
  echo "START $case_id $profile $(date -u +%FT%TZ)"
  nice -n 15 python3 "$BAG16_SOURCE/slam/scripts/profile_replay.py" \
    --input "$BAG16_INPUT" \
    --config "$BAG16_SOURCE/slam/config/bag16_compare_$profile.yaml" \
    --output "$BAG16_OUTPUT/$case_id" > "$BAG16_OUTPUT/$case_id.log" 2>&1
  echo "DONE $case_id $(date -u +%FT%TZ)"
done
