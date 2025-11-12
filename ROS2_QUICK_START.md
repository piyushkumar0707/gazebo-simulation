# ROS 2 Quick Start Guide

## For ROS 2 Humble (Ubuntu 22.04)

### 1. Build the Package

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Create/use workspace
mkdir -p ~/ros2_ws/src
ln -s ~/Documents/"gazebo simulation"/simple_3dof_arm ~/ros2_ws/src/

# Build
cd ~/ros2_ws
colcon build --packages-select simple_3dof_arm
source install/setup.bash
```

### 2. Launch Gazebo Simulation

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch simple_3dof_arm gazebo_sim.launch.py
```

### 3. Control the Robot

#### Option A: Use Gazebo GUI
- In Gazebo: **Window → Joint Control**
- Move the sliders for each joint

#### Option B: Use ROS 2 Topics (Coming Soon)
The full ROS 2 control integration is being developed. For now, use the GUI method.

### 4. View Camera Feed

```bash
# List camera topics
ros2 topic list | grep camera

# View image (requires image_view or rqt_image_view)
ros2 run rqt_image_view rqt_image_view
```

## Alternative: Run Without ROS

If you don't need ROS integration, just run:

```bash
cd ~/Documents/"gazebo simulation"/simple_3dof_arm
./run_gazebo.sh
```

This works perfectly for assignments and demos!

## Installation Requirements

```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install gazebo
```

## Next Steps

- Full ros2_control integration (in progress)
- Python control node for ROS 2
- MoveIt 2 integration

For more details, see the main README.md
