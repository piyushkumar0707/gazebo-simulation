#!/usr/bin/env bash
# ==============================================================================
# SIMPLE STANDALONE LAUNCHER - No ROS Required!
# ==============================================================================
# This script runs the simulation without ROS - perfect for quick testing
# ==============================================================================

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}"
echo "=========================================="
echo "  🤖 SIMPLE GAZEBO LAUNCHER 🤖"
echo "=========================================="
echo -e "${NC}"
echo ""
echo -e "${GREEN}Starting standalone Gazebo simulation...${NC}"
echo ""
echo -e "${YELLOW}No ROS required!${NC}"
echo ""
echo -e "After Gazebo opens:"
echo "  1. Go to: ${GREEN}Window → Joint Control${NC}"
echo "  2. Select the robot model"
echo "  3. Use sliders to move the arm joints:"
echo "     - ${BLUE}joint_base${NC} (rotate base)"
echo "     - ${BLUE}joint_shoulder${NC} (move shoulder)"
echo "     - ${BLUE}joint_elbow${NC} (move elbow)"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop the simulation${NC}"
echo ""

# Kill any existing Gazebo
killall -9 gzserver gzclient 2>/dev/null || true
sleep 1

# Set Gazebo model path
export GAZEBO_MODEL_PATH="$ROOT_DIR/models:${GAZEBO_MODEL_PATH}"

# Check if gazebo is installed
if ! command -v gazebo >/dev/null 2>&1; then
    echo -e "${RED}Error: Gazebo not found!${NC}"
    echo ""
    echo "Please install Gazebo:"
    echo "  sudo apt install gazebo11"
    echo ""
    exit 1
fi

echo -e "${GREEN}Launching Gazebo...${NC}"
echo ""

# Run Gazebo
gazebo "$ROOT_DIR/worlds/simple_table_world.world" --verbose
