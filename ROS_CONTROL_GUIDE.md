# ROS Control Guide for 3-DOF Robot Arm

This guide explains how to control the robot arm using ROS (Robot Operating System).

## Prerequisites

Make sure you have ROS installed (tested with ROS Noetic on Ubuntu 20.04):

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
sudo apt install ros-noetic-joint-state-controller
sudo apt install ros-noetic-position-controllers
sudo apt install ros-noetic-robot-state-publisher
```

## Setup

### 1. Create a Catkin Workspace (if you don't have one)

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

### 2. Link or Copy this Package

```bash
# Option 1: Symlink (recommended)
ln -s ~/Documents/"gazebo simulation"/simple_3dof_arm ~/catkin_ws/src/

# Option 2: Copy
cp -r ~/Documents/"gazebo simulation"/simple_3dof_arm ~/catkin_ws/src/
```

### 3. Build the Package

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 4. Add to .bashrc (optional, for convenience)

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## Launching with ROS Control

### Start the Simulation

```bash
roslaunch simple_3dof_arm robot_control.launch
```

This will:
- Launch Gazebo with the world
- Load the robot model
- Start ROS control controllers
- Enable joint position control via topics

## Controlling the Robot

### Method 1: Command Line (rostopic pub)

Control individual joints by publishing to their command topics:

```bash
# Rotate base (90 degrees = 1.57 radians)
rostopic pub /simple_3dof_arm/base_position_controller/command std_msgs/Float64 "data: 1.57"

# Bend shoulder forward (45 degrees)
rostopic pub /simple_3dof_arm/shoulder_position_controller/command std_msgs/Float64 "data: 0.785"

# Bend elbow (30 degrees)
rostopic pub /simple_3dof_arm/elbow_position_controller/command std_msgs/Float64 "data: -0.52"
```

### Method 2: Python Script (Automated Demo)

Run the included demo script:

```bash
rosrun simple_3dof_arm control_arm.py demo
```

This will move the arm through a predefined sequence:
1. Home position (all joints at 0)
2. Rotate base 90°
3. Bend shoulder
4. Bend elbow
5. Reach forward
6. Return home

### Method 3: Custom Python Script

Create your own control script:

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

rospy.init_node('my_controller')

base_pub = rospy.Publisher('/simple_3dof_arm/base_position_controller/command', Float64, queue_size=10)
shoulder_pub = rospy.Publisher('/simple_3dof_arm/shoulder_position_controller/command', Float64, queue_size=10)
elbow_pub = rospy.Publisher('/simple_3dof_arm/elbow_position_controller/command', Float64, queue_size=10)

rospy.sleep(1)  # Wait for connection

# Move to position
base_pub.publish(Float64(1.0))
shoulder_pub.publish(Float64(0.5))
elbow_pub.publish(Float64(-0.5))
```

### Method 4: rqt GUI

Use rqt for graphical control:

```bash
rqt
```

Then:
1. Plugins → Robot Tools → Controller Manager
2. Or Plugins → Robot Tools → Joint Trajectory Controller

## Monitoring Robot State

### View Joint States

```bash
rostopic echo /simple_3dof_arm/joint_state_controller/joint_states
```

### List Available Topics

```bash
rostopic list
```

You should see:
- `/simple_3dof_arm/base_position_controller/command`
- `/simple_3dof_arm/shoulder_position_controller/command`
- `/simple_3dof_arm/elbow_position_controller/command`
- `/simple_3dof_arm/joint_state_controller/joint_states`

### View Camera Feed

```bash
# List camera topics
rostopic list | grep camera

# View image (requires image_view)
rosrun image_view image_view image:=/simple_3dof_arm/wrist_camera/image_raw
```

## Joint Limits

- **joint_base**: -π to π (-180° to 180°)
- **joint_shoulder**: -π/2 to π/2 (-90° to 90°)
- **joint_elbow**: -π/2 to π/2 (-90° to 90°)

## Troubleshooting

### Controllers not loading

```bash
# Check if controller manager is running
rosservice list | grep controller

# Manually spawn controllers
rosrun controller_manager spawner --stopped base_position_controller
```

### No joint states published

```bash
# Check if joint_state_controller is running
rostopic hz /simple_3dof_arm/joint_state_controller/joint_states
```

### Robot falling or behaving strangely

Check that transmissions are properly defined in the model.sdf file.

## Advanced: Trajectory Control

For smooth multi-joint trajectories, use the arm_controller:

```bash
# Switch to trajectory controller
rosrun controller_manager spawner arm_controller

# Send trajectory goals (requires trajectory_msgs)
# See ROS trajectory controller documentation
```

## Example Use Cases

### 1. Pick and Place Simulation
Move arm to hover over object, lower gripper, move to new location.

### 2. Workspace Analysis
Systematically move joints through their range to map reachable space.

### 3. Camera-based Tasks
Use camera feed to guide arm toward objects.

### 4. Integration with MoveIt!
For more advanced motion planning (requires additional setup).

## Next Steps

- Add inverse kinematics for Cartesian control
- Implement gripper actuation
- Add force/torque sensors
- Integrate with computer vision
- Connect to real hardware

## Resources

- [ROS Control Documentation](http://wiki.ros.org/ros_control)
- [Gazebo ROS Control Tutorial](http://gazebosim.org/tutorials?tut=ros_control)
- [Joint Trajectory Controller](http://wiki.ros.org/joint_trajectory_controller)

---

Happy robot controlling! 🤖
