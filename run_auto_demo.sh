#!/usr/bin/env bash
# ==============================================================================
# FULLY AUTOMATED DEMO - One command does everything!
# ==============================================================================
# Starts Gazebo and automatically runs pick-and-place demonstration
# No manual control needed - sit back and watch!
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
echo "  🤖 FULLY AUTOMATED DEMO 🤖"
echo "=========================================="
echo -e "${NC}"
echo ""
echo -e "${GREEN}This will:${NC}"
echo "  1. Start Gazebo simulation"
echo "  2. Wait for everything to load"
echo "  3. Automatically run pick-and-place demo"
echo "  4. Show the robot moving the red cube!"
echo ""
echo -e "${YELLOW}No manual control needed - just watch!${NC}"
echo ""

read -p "Press ENTER to start, or Ctrl+C to cancel..."

# Kill any existing Gazebo
echo ""
echo -e "${BLUE}Cleaning up old processes...${NC}"
killall -9 gzserver gzclient 2>/dev/null || true
sleep 2

# Set model path
export GAZEBO_MODEL_PATH="$ROOT_DIR/models:${GAZEBO_MODEL_PATH}"

echo ""
echo -e "${GREEN}Starting Gazebo in background...${NC}"
echo ""

# Start Gazebo in background
gazebo "$ROOT_DIR/worlds/simple_table_world.world" --verbose &
GAZEBO_PID=$!

echo ""
echo -e "${YELLOW}Waiting for Gazebo to initialize (15 seconds)...${NC}"
echo ""

# Wait for Gazebo to fully start
for i in {15..1}; do
    echo -ne "  Starting demo in ${i} seconds...\r"
    sleep 1
done

echo ""
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Starting Automated Demonstration!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Run the automated demo
python3 "$ROOT_DIR/scripts/auto_demo.py" 1

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Demo Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "The simulation is still running."
echo "You can:"
echo "  • Run another demo: python3 scripts/auto_demo.py"
echo "  • Close this window to stop Gazebo"
echo "  • Or press Ctrl+C now"
echo ""

# Keep Gazebo running
echo "Press Ctrl+C to stop Gazebo..."
wait $GAZEBO_PID
