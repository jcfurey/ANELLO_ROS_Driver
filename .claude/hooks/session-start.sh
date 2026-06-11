#!/bin/bash
# SessionStart hook: set up a ROS2 Jazzy build environment for Claude Code on
# the web so `colcon build` and `colcon test` work in remote sessions.
set -euo pipefail

# Only needed in remote (web) sessions; local machines manage their own ROS.
if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
  exit 0
fi

export DEBIAN_FRONTEND=noninteractive
ROS_DISTRO_NAME=jazzy

# ROS2 Jazzy on Ubuntu 24.04 requires Python 3.12, but the sandbox image
# defaults python3 to 3.11, which breaks ament_python builds
# (AttributeError: install_layout). Point python3 at 3.12.
if [ "$(readlink -f "$(command -v python3)")" != "/usr/bin/python3.12" ]; then
  sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.12 2
  sudo update-alternatives --set python3 /usr/bin/python3.12
  if [ -L /usr/local/bin/python3 ]; then
    sudo ln -sf /usr/bin/python3.12 /usr/local/bin/python3
  fi
fi

# Add the ROS2 apt source if it isn't configured yet.
if ! ls /etc/apt/sources.list.d/ros2*.sources >/dev/null 2>&1 && \
   ! ls /etc/apt/sources.list.d/ros2*.list >/dev/null 2>&1; then
  sudo apt-get update -qq
  sudo apt-get install -y -qq curl
  ROS_APT_VERSION="1.1.0"
  CODENAME="$(. /etc/os-release && echo "$VERSION_CODENAME")"
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_VERSION}/ros2-apt-source_${ROS_APT_VERSION}.${CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt-get update -qq
fi

# Install ROS2 base, colcon, and this workspace's dependencies (idempotent;
# apt skips packages that are already installed).
if ! dpkg -s ros-${ROS_DISTRO_NAME}-ros-base >/dev/null 2>&1; then
  sudo apt-get update -qq
fi
sudo apt-get install -y -qq \
  ros-${ROS_DISTRO_NAME}-ros-base \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-pytest \
  ros-${ROS_DISTRO_NAME}-nmea-msgs \
  ros-${ROS_DISTRO_NAME}-rtcm-msgs \
  ros-${ROS_DISTRO_NAME}-diagnostic-updater \
  ros-${ROS_DISTRO_NAME}-ament-lint-auto \
  ros-${ROS_DISTRO_NAME}-ament-lint-common \
  ros-${ROS_DISTRO_NAME}-ament-cmake-copyright \
  ros-${ROS_DISTRO_NAME}-ament-flake8 \
  ros-${ROS_DISTRO_NAME}-ament-pep257

# Persist the ROS environment for the session so colcon/ros2 work without
# having to source /opt/ros/<distro>/setup.bash in every shell.
if [ -n "${CLAUDE_ENV_FILE:-}" ]; then
  set +u
  # shellcheck disable=SC1090
  source /opt/ros/${ROS_DISTRO_NAME}/setup.bash
  set -u
  for var in PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH \
             PYTHONPATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION; do
    if [ -n "${!var:-}" ]; then
      printf 'export %s="%s"\n' "$var" "${!var}" >> "$CLAUDE_ENV_FILE"
    fi
  done
fi

echo "ROS2 ${ROS_DISTRO_NAME} build environment ready"
