#!/usr/bin/env bash
# Run the occupancy-map -> USD generator with standalone pxr (no full Isaac launch).
# Usage: ./run_generate_map.sh [map.yaml] [out.usd] [wall_height]
set -e
unset PYTHONPATH
source /home/aes/miniconda3/etc/profile.d/conda.sh
conda activate isaacsim
USDLIB=/home/aes/miniconda3/envs/isaacsim/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310
export PYTHONPATH="$USDLIB"
export LD_LIBRARY_PATH="$USDLIB/bin:$USDLIB/lib:$LD_LIBRARY_PATH"
exec python /home/aes/isaac_tb4/scripts/generate_map_usd.py "$@"
