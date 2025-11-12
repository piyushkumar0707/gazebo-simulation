#!/usr/bin/env bash
# Simple runner for the example world
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GAZEBO_MODEL_PATH="$ROOT_DIR/models:${GAZEBO_MODEL_PATH}"

echo "GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH"
echo "Starting Gazebo..."

# Choose installed gazebo command (gazebo or gz depending on version). Try gazebo first.
if command -v gazebo >/dev/null 2>&1; then
  gazebo "$ROOT_DIR/../worlds/simple_table_world.world" --verbose
elif command -v gz >/dev/null 2>&1; then
  gz sim -r "$ROOT_DIR/../worlds/simple_table_world.world"
else
  echo "Error: gazebo (classic) or gz (ignition) not found in PATH. Install Gazebo and retry."
  exit 1
fi

