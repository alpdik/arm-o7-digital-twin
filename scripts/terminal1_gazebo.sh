#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# Mesa may otherwise fall back to llvmpipe on WSLg. Keep native Ubuntu
# untouched. Do not assume a GPU brand: Mesa chooses automatically unless
# ARM_O7_GPU_ADAPTER is explicitly set (for example, NVIDIA, AMD or Intel).
if [[ -e /dev/dxg ]] && grep -qi microsoft /proc/sys/kernel/osrelease; then
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
  if [[ -n "${ARM_O7_GPU_ADAPTER:-}" ]]; then
    export MESA_D3D12_DEFAULT_ADAPTER_NAME="$ARM_O7_GPU_ADAPTER"
  fi
  echo "[graphics] WSLg backend=$GALLIUM_DRIVER adapter=${MESA_D3D12_DEFAULT_ADAPTER_NAME:-auto}"
fi

source /opt/ros/jazzy/setup.bash
source "$project_dir/install/setup.bash"
set -u

exec ros2 launch arm_o7_bringup gazebo.launch.py
