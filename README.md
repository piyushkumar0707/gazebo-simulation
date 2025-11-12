# Simple 3-DOF Arm Gazebo Example

This workspace contains a minimal Gazebo simulation of a simple 3-axis robot arm (base rotation, shoulder, elbow), a wrist-mounted camera, and a simple two-finger gripper (fixed). There's also a table and a small cube to interact with.

## Features

- ✅ **3-DOF Robot Arm**: Base rotation, shoulder, and elbow joints
- ✅ **Camera Sensor**: Wrist-mounted RGB camera (640x480)
- ✅ **2-Finger Gripper**: Simple parallel gripper
- ✅ **Realistic Environment**: Wooden table with multiple objects
- ✅ **ROS Control**: Full integration with ROS for programmatic control
- ✅ **Professional Visuals**: Studio lighting, shadows, and materials

## Files Structure

```
simple_3dof_arm/
├── models/
│   └── simple_3dof_arm/
│       ├── model.config       # Model metadata
│       └── model.sdf          # Robot definition with ROS plugins
├── worlds/
│   └── simple_table_world.world  # Scene with table and objects
├── config/
│   └── controllers.yaml       # ROS controller configuration
├── launch/
│   └── robot_control.launch   # ROS launch file
├── scripts/
│   └── control_arm.py         # Python control script
├── CMakeLists.txt             # Catkin build configuration
├── package.xml                # ROS package manifest
├── run_gazebo.sh              # Standalone Gazebo launcher
├── README.md                  # This file
├── ROS_CONTROL_GUIDE.md       # Detailed ROS control guide
└── ASSIGNMENT_GUIDE.md        # Assignment presentation guide
```

## Quick Start (Standalone - No ROS)

1. Install Gazebo Classic:
```bash
sudo apt install gazebo11
```

2. Run the simulation:
```bash
cd simple_3dof_arm
./run_gazebo.sh
```

3. Control joints using Gazebo GUI:
   - **Window → Joint Control** to move joints with sliders

## Quick Start (With ROS Control)

See **[ROS_CONTROL_GUIDE.md](ROS_CONTROL_GUIDE.md)** for detailed instructions.

### Prerequisites
```bash
sudo apt install ros-noetic-desktop-full
sudo apt install ros-noetic-gazebo-ros-control
sudo apt install ros-noetic-joint-state-controller
sudo apt install ros-noetic-position-controllers
```

### Setup
```bash
# Create/use catkin workspace
mkdir -p ~/catkin_ws/src
ln -s $(pwd) ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Launch with ROS
```bash
roslaunch simple_3dof_arm robot_control.launch
```

### Control the Arm
```bash
# Run demo sequence
rosrun simple_3dof_arm control_arm.py demo

# Or control individual joints
rostopic pub /simple_3dof_arm/base_position_controller/command std_msgs/Float64 "data: 1.57"
```

## Documentation

- **[README.md](README.md)** - This file (quick start)
- **[ROS_CONTROL_GUIDE.md](ROS_CONTROL_GUIDE.md)** - Complete ROS control documentation
- **[ASSIGNMENT_GUIDE.md](ASSIGNMENT_GUIDE.md)** - Presentation tips and usage guide

## Controlling Joints

### Method 1: Gazebo GUI (No ROS)
- Select model → **Window → Joint Control**
- Use sliders for joint_base, joint_shoulder, joint_elbow

### Method 2: ROS Topics
```bash
rostopic pub /simple_3dof_arm/base_position_controller/command std_msgs/Float64 "data: 1.57"
rostopic pub /simple_3dof_arm/shoulder_position_controller/command std_msgs/Float64 "data: 0.785"
rostopic pub /simple_3dof_arm/elbow_position_controller/command std_msgs/Float64 "data: -0.52"
```

### Method 3: Python Script
```bash
rosrun simple_3dof_arm control_arm.py demo
```

## Camera Feed

### Without ROS
- In Gazebo: **Window → Topic Visualization**
- Select camera topic

### With ROS
```bash
rosrun image_view image_view image:=/simple_3dof_arm/wrist_camera/image_raw
```

## Joint Specifications

| Joint | Type | Range | Description |
|-------|------|-------|-------------|
| joint_base | Revolute | ±180° | Base rotation (Z-axis) |
| joint_shoulder | Revolute | ±90° | Shoulder pitch (Y-axis) |
| joint_elbow | Revolute | ±90° | Elbow pitch (Y-axis) |

## Technical Details

- **Simulation**: Gazebo Classic 11.x
- **Physics**: ODE with 1ms timestep
- **ROS**: Noetic (Ubuntu 20.04)
- **Control**: ros_control with position controllers
- **Camera**: 640x480 RGB, 30Hz, 45° FOV

## Troubleshooting

**Gazebo won't start:**
- Kill existing processes: `killall -9 gzserver gzclient`
- Check Gazebo is installed: `gazebo --version`

**ROS controllers not loading:**
- Verify ROS packages installed: `rospack list | grep controller`
- Check controller status: `rosservice call /simple_3dof_arm/controller_manager/list_controllers`

**Robot falls apart:**
- Ensure transmissions are defined in model.sdf
- Check joint limits and dynamics

## Next Steps

- [ ] Add actuated gripper for pick-and-place
- [ ] Implement inverse kinematics
- [ ] Add force/torque sensors
- [ ] Integrate with MoveIt! for motion planning
- [ ] Connect to real hardware

## Contributing

Feel free to fork and improve! Suggestions:
- Better collision meshes
- More sophisticated gripper
- Example manipulation tasks
- Integration with perception pipelines

## License

MIT License - Feel free to use for educational purposes.

## Author

Created for robotics simulation and learning.

Repository: https://github.com/piyushkumar0707/gazebo-simulation

---

For detailed ROS control instructions, see **[ROS_CONTROL_GUIDE.md](ROS_CONTROL_GUIDE.md)**

Good luck with your robotics projects! 🤖✨