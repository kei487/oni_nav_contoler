#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble not found at /opt/ros/humble/setup.bash" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash

if [[ ! -f "$SCRIPT_DIR/install/setup.bash" ]]; then
  echo "Workspace not built. Run:" >&2
  echo "  source /opt/ros/humble/setup.bash" >&2
  echo "  colcon build --symlink-install" >&2
  exit 1
fi

source "$SCRIPT_DIR/install/setup.bash"

ACADOS_INSTALL_DIR="${SCRIPT_DIR}/deps/acados_install"
if [[ ! -f "${ACADOS_INSTALL_DIR}/lib/libacados.so" ]]; then
  echo "Acados not found at ${ACADOS_INSTALL_DIR}." >&2
  echo "Build Acados first (see doc/specification.md)." >&2
  exit 1
fi

export ACADOS_INSTALL_DIR
export LD_LIBRARY_PATH="${ACADOS_INSTALL_DIR}/lib:${LD_LIBRARY_PATH:-}"

usage() {
  cat <<'EOF'
Usage:
  ./exec.bash                 Launch nav_controller_node (production)
  ./exec.bash mock [mode]       Launch with mock sensors (mode: chase|idle|lost|stop)

Examples:
  ./exec.bash
  ./exec.bash mock chase
EOF
}

CMD="${1:-}"
case "$CMD" in
  -h|--help|help)
    usage
    exit 0
    ;;
  mock)
    MODE="${2:-chase}"
    exec ros2 launch oni_nav_controller mock_nav_test.launch.py "mode:=${MODE}"
    ;;
  "")
    exec ros2 launch oni_nav_controller nav_controller.launch.py
    ;;
  chase|idle|lost|stop)
    exec ros2 launch oni_nav_controller mock_nav_test.launch.py "mode:=${CMD}"
    ;;
  *)
    echo "Unknown argument: ${CMD}" >&2
    usage
    exit 1
    ;;
esac
