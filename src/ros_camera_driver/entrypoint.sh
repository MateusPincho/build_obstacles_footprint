#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -f /camera_ws/install/setup.bash ]; then
  source /camera_ws/install/setup.bash
fi

exec "$@"