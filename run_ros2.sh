#!/usr/bin/env bash
# ==============================================================================
# ROS 2 HUMBLE LAUNCHER - Easy Start for ROS2
# ==============================================================================

set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  3-DOF Robot Arm - ROS 2 Humble${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if ROS 2 Humble is installed
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    print_error "ROS 2 Humble not found!"
    echo ""
    echo "Please install ROS 2 Humble:"
    echo "  sudo apt install ros-humble-desktop"
    echo "  sudo apt install ros-humble-gazebo-ros-pkgs"
    echo ""
    exit 1
fi

# Source ROS 2
print_info "Sourcing ROS 2 Humble..."
source /opt/ros/humble/setup.bash

# Check if workspace exists
WORKSPACE="$HOME/ros2_ws"

if [ ! -d "$WORKSPACE/install" ]; then
    print_warning "Workspace not built. Building now..."
    
    # Create workspace
    mkdir -p "$WORKSPACE/src"
    
    # Create symlink if it doesn't exist
    if [ ! -L "$WORKSPACE/src/simple_3dof_arm" ]; then
        print_info "Creating symlink to package..."
        ln -sf "$ROOT_DIR" "$WORKSPACE/src/simple_3dof_arm"
    fi
    
    # Build
    print_info "Building workspace..."
    cd "$WORKSPACE"
    colcon build --packages-select simple_3dof_arm
    
    if [ $? -ne 0 ]; then
        print_error "Build failed!"
        exit 1
    fi
    
    print_success "Build complete!"
else
    print_success "Workspace found!"
fi

# Source workspace
if [ -f "$WORKSPACE/install/setup.bash" ]; then
    print_info "Sourcing workspace..."
    source "$WORKSPACE/install/setup.bash"
fi

# Kill existing Gazebo
print_info "Cleaning up existing Gazebo processes..."
killall -9 gzserver gzclient 2>/dev/null || true
sleep 1

# Show instructions
echo ""
print_success "Ready to launch!"
echo ""
echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}  Control Instructions:${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""
echo "After Gazebo opens:"
echo "  1. Click on the robot arm"
echo "  2. Go to: ${GREEN}Window → Joint Control${NC}"
echo "  3. Move the sliders to control:"
echo "     • ${BLUE}joint_base${NC} - Base rotation"
echo "     • ${BLUE}joint_shoulder${NC} - Shoulder pitch"
echo "     • ${BLUE}joint_elbow${NC} - Elbow bend"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop${NC}"
echo ""

# Launch
print_info "Launching Gazebo simulation..."
echo ""

ros2 launch simple_3dof_arm gazebo_sim.launch.py
