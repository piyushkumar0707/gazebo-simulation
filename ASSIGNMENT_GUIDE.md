# 3-DOF Robot Arm Simulation - Assignment Guide

## Overview
Professional Gazebo simulation of a 3-axis robot manipulator with camera sensor and gripper for pick-and-place operations.

## Features Implemented
✅ **3-DOF Articulated Robot Arm**
- Base rotation joint (revolute, Z-axis)
- Shoulder joint (revolute, Y-axis, ±90°)
- Elbow joint (revolute, Y-axis, ±90°)

✅ **End Effector**
- Wrist-mounted camera sensor (640x480, 30Hz)
- Two-finger parallel gripper

✅ **Environment**
- Textured wooden table with cylindrical legs
- Multiple objects for manipulation (cube, cylinder, sphere)
- Professional 3-point lighting setup (sun, key light, fill light)
- Realistic shadows and atmospheric effects

## Quick Start

### Launch Simulation
```bash
cd ~/Documents/"gazebo simulation"/simple_3dof_arm
./run_gazebo.sh
```

### Control Joints (Method 1 - GUI)
1. In Gazebo, go to **Window → Joint Control**
2. Or right-click the robot → **View → Joints**
3. Use sliders to control:
   - `joint_base` - Base rotation
   - `joint_shoulder` - Shoulder pitch
   - `joint_elbow` - Elbow pitch

### Control Joints (Method 2 - Topic)
Open a new terminal and publish joint commands:

```bash
# Example: Rotate base
gz topic -t /gazebo/default/simple_3dof_arm/joint_cmd \
  -m gazebo.msgs.JointCmd \
  -p 'name: "joint_base", position: {target: 1.57}'
```

### View Camera Feed
1. In Gazebo GUI, go to **Window → Topic Visualization**
2. Select `/gazebo/default/simple_3dof_arm/wrist_link/wrist_camera/image`
3. Or in terminal:
```bash
gz topic -e /gazebo/default/simple_3dof_arm/wrist_link/wrist_camera/image
```

## Presentation Tips

### Key Points to Highlight
1. **Kinematic Structure**: Simple serial manipulator with 3 revolute joints
2. **Workspace**: Demonstrate reachable area by moving joints
3. **Sensor Integration**: Show camera view updating as arm moves
4. **Physics Simulation**: Objects respond to gravity and collisions
5. **Visual Quality**: Professional lighting, shadows, and materials

### Screenshots to Take
- [ ] Full scene overview showing entire workspace
- [ ] Close-up of robot arm in different configurations
- [ ] Camera view from wrist showing objects
- [ ] Joint control panel with sliders visible
- [ ] Robot manipulating/reaching for objects

### Camera Tips for Best Views
In Gazebo:
- **Orbit**: Hold middle mouse button and drag
- **Pan**: Hold Shift + middle mouse button and drag
- **Zoom**: Scroll wheel
- **Reset view**: Press 'R' key

Good camera angles:
1. Isometric view (45° from side and above)
2. Side view to show shoulder/elbow motion
3. Top view to show base rotation
4. Close-up of gripper and camera

## Technical Specifications

### Robot Parameters
- **Total Height**: ~0.8m (extended)
- **Base Diameter**: 0.24m
- **Shoulder Link**: 0.36m
- **Elbow Link**: 0.32m
- **Total Mass**: ~9kg

### Joint Limits
- Base: ±180° (unlimited rotation)
- Shoulder: ±90°
- Elbow: ±90°

### Sensor Specifications
- Camera Resolution: 640×480 pixels
- FOV: 45° (0.785 rad)
- Update Rate: 30 Hz
- Format: RGB8

### World Settings
- Physics Engine: ODE
- Time Step: 1ms
- Real-time Factor: 1.0
- Shadows: Enabled
- Lighting: 3-point setup

## Troubleshooting

**If simulation runs slow:**
1. Go to **View → Shadows** (toggle off)
2. Reduce physics update rate in world file
3. Simplify collision meshes

**If joints don't move:**
1. Check joint control panel is connected
2. Verify model is not set to static
3. Reset simulation (Ctrl+R)

**If camera shows black screen:**
1. Wait a few seconds for initialization
2. Check Topic Visualization settings
3. Restart simulation

## Extensions (Optional)

If you want to demonstrate more:
1. **ROS Integration**: Control via ROS topics
2. **Inverse Kinematics**: Calculate joint angles for target position
3. **Trajectory Planning**: Smooth motion between waypoints
4. **Gripper Control**: Make fingers actuated
5. **Object Detection**: Process camera feed with OpenCV

## File Structure
```
simple_3dof_arm/
├── models/
│   └── simple_3dof_arm/
│       ├── model.config       # Model metadata
│       └── model.sdf          # Robot definition
├── worlds/
│   └── simple_table_world.world  # Scene setup
├── run_gazebo.sh             # Launch script
├── README.md                 # Basic instructions
└── ASSIGNMENT_GUIDE.md       # This file
```

## Credits
- Simulation Framework: Gazebo Classic
- Physics Engine: Open Dynamics Engine (ODE)
- Robot Design: Custom 3-DOF manipulator

Good luck with your assignment! 🎓🤖
