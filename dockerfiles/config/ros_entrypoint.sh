#!/usr/bin/env bash
set -e

# setup ros environment for harmoni
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$ROS_WS/devel/setup.bash"

exec "$@"