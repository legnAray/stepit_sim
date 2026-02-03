#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/run_sim.sh -r <robot> -s <scene.xml> [stepit args] [-- <plugin args>]

Required:
  -r, --robot       Robot name (g1 | go2 | b2 | aliengo)
  -s, --scene       Scene file (e.g., scene.xml)

Optional:
  -ros2            Use ROS2 stepit binary (requires ROS2 env pre-sourced)

Notes:
  - All other arguments are passed to stepit as-is.
  - Use "--" to pass plugin/ROS2 CLI arguments to stepit plugins.
  - STEPIT_NETIF and STEPIT_DOMAIN_ID are fixed to lo / 1 in this repo.
EOF
}

robot=""
stepit_robot=""
scene=""
use_ros2=0
stepit_args=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    -r|--robot)
      robot="${2:-}"
      if [[ -z "$robot" ]]; then
        echo "Missing value for $1" >&2
        exit 1
      fi
      stepit_robot="$robot"
      if [[ "$robot" == "g1" ]]; then
        stepit_robot="g1_29dof"
      fi
      stepit_args+=("$1" "$stepit_robot")
      shift 2
      ;;
    -s|--scene)
      scene="${2:-}"
      if [[ -z "$scene" ]]; then
        echo "Missing value for $1" >&2
        exit 1
      fi
      shift 2
      ;;
    -ros2)
      use_ros2=1
      shift
      ;;
    --)
      stepit_args+=("--")
      shift
      stepit_args+=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      stepit_args+=("$1")
      shift
      ;;
  esac
done

if [[ -z "$robot" || -z "$scene" ]]; then
  usage
  exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_BIN="${ROOT_DIR}/third_party/unitree_mujoco/simulate/build/unitree_mujoco"
if [[ "$use_ros2" -eq 1 ]]; then
  STEPIT_BIN="${ROOT_DIR}/ros2_ws/install/stepit_ros2/lib/stepit_ros2/stepit"
  if [[ -z "${ROS_DISTRO:-}" ]]; then
    echo "Warning: ROS2 environment not detected. Please source /opt/ros/<distro>/setup.zsh and ros2_ws/install/setup.zsh." >&2
  fi
else
  STEPIT_BIN="${ROOT_DIR}/build/stepit/bin/stepit"
fi

if [[ ! -x "$SIM_BIN" ]]; then
  echo "unitree_mujoco not found at: $SIM_BIN" >&2
  exit 1
fi

if [[ ! -x "$STEPIT_BIN" ]]; then
  echo "stepit not found at: $STEPIT_BIN" >&2
  exit 1
fi

export STEPIT_NETIF="lo"
export STEPIT_DOMAIN_ID="1"

added_ip=0
cleanup() {
  if [[ -n "${sim_pid:-}" ]]; then
    kill "$sim_pid" 2>/dev/null || true
    wait "$sim_pid" 2>/dev/null || true
  fi
  if [[ "$added_ip" -eq 1 ]]; then
    sudo ip addr del 192.168.123.10/24 dev lo 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "$robot" == "aliengo" ]]; then
  export STEPIT_ALIENGO_RAW_UDP=1
  if ! ip addr show lo | grep -q "192.168.123.10/24"; then
    sudo ip addr add 192.168.123.10/24 dev lo
    added_ip=1
  fi
fi

"$SIM_BIN" -r "$robot" -s "$scene" -i 1 -n lo &
sim_pid=$!
sleep 1

"$STEPIT_BIN" "${stepit_args[@]}"
