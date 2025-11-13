# 🤖 3-DOF Robot Arm - Gazebo Simulation

Professional robotic arm simulation with automated pick-and-place demonstrations.

## ⚡ QUICK START

### 🚀 Run Automated Demo (Recommended!)
```bash
./run_auto_demo.sh
```
**The robot will automatically pick up and move objects!**

### 🎮 Manual Control
```bash
./run_simple.sh
```
Then in Gazebo: **Window → Joint Control** to move joints manually.

### 🔧 With ROS 2 Humble
```bash
./run_ros2.sh
```

---

## ✨ Features

- ✅ **Fully Automated Demos** - Pick-and-place, wave, scan
- ✅ **Large Colorful Robot** - Easy to see all parts (1.5m tall)
- ✅ **3-DOF Arm** - Base rotation, shoulder, elbow joints
- ✅ **Wrist Camera** - RGB camera (640x480)
- ✅ **Two-Finger Gripper** - Parallel gripper
- ✅ **Professional Visuals** - Studio lighting and shadows
- ✅ **ROS 2 Compatible** - Full integration available

## 📁 Project Structure

```
simple_3dof_arm/
├── models/simple_3dof_arm/    # Robot model (SDF format)
├── worlds/                     # Simulation world with table & objects
├── scripts/auto_demo.py        # Automated demonstration script
├── launch/                     # ROS 2 launch files
├── config/                     # ROS controller configuration
├── run_auto_demo.sh            # ⭐ ONE-CLICK automated demo
├── run_simple.sh               # Simple Gazebo launcher
├── run_ros2.sh                 # ROS 2 Humble launcher
├── README.md                   # This file
├── ROS2_QUICK_START.md         # ROS 2 specific guide
├── ROS_CONTROL_GUIDE.md        # Advanced ROS control
└── ASSIGNMENT_GUIDE.md         # Presentation tips
```

## 🎬 Automated Demonstrations

The robot performs complete automated sequences:

### Available Demos:
1. **Pick and Place** - Grabs red cube and moves it
2. **Wave** - Friendly waving motion (3 waves)
3. **Scan** - Workspace scanning left-right
4. **All Demos** - Runs all sequences

### Run a Specific Demo:
```bash
# After starting Gazebo with ./run_simple.sh
python3 scripts/auto_demo.py 1    # Pick and place
python3 scripts/auto_demo.py 2    # Wave
python3 scripts/auto_demo.py 3    # Scan
python3 scripts/auto_demo.py 4    # All demos
```

## 🎨 Robot Appearance

The robot features color-coded parts for easy identification:
- 🔴 **Red cylinder** - Base rotation joint
- 🟡 **Yellow arm** - Upper arm segment (0.7m)
- 🔵 **Blue cylinder** - Elbow joint
- 🟢 **Green arm** - Forearm (0.6m)
- 🟣 **Purple cylinder** - Wrist joint
- 🟠 **Orange sphere** - Wrist ball
- ⚪ **White fingers** - Gripper
- ⚫ **Black box** - Camera housing

## 📖 Documentation

- **README.md** - This file (overview & quick start)
- **ROS2_QUICK_START.md** - ROS 2 Humble setup guide
- **ROS_CONTROL_GUIDE.md** - Advanced ROS control
- **ASSIGNMENT_GUIDE.md** - Presentation tips

## 🎮 Manual Control

### Option 1: Gazebo GUI (No ROS Required)
1. Start simulation: `./run_simple.sh`
2. Click on robot arm
3. **Window → Joint Control**
4. Move sliders for each joint

### Option 2: ROS 2 Topics (Advanced)
```bash
# After starting with ./run_ros2.sh
ros2 topic list
ros2 topic echo /joint_states
```

## 📸 Camera Feed

View the wrist camera:
```bash
# In Gazebo GUI
Window → Topic Visualization → Select camera

# With ROS 2
ros2 run rqt_image_view rqt_image_view
```

## 🔧 Technical Specifications

### Robot Dimensions:
- **Total Height**: ~1.5 meters
- **Base Diameter**: 0.5 meters
- **Upper Arm**: 0.7 meters
- **Forearm**: 0.6 meters
- **Gripper Width**: 0.28 meters

### Joints:
| Joint | Type | Range | Axis |
|-------|------|-------|------|
| joint_base | Revolute | ±180° | Z (rotation) |
| joint_shoulder | Revolute | ±90° | Y (pitch) |
| joint_elbow | Revolute | ±90° | Y (bend) |

### System:
- **Simulation**: Gazebo Classic 11+
- **Physics**: ODE (1ms timestep)
- **ROS**: ROS 2 Humble (Ubuntu 22.04)
- **Camera**: 640x480 RGB, 30Hz, 45° FOV

## 🛠️ Installation

### Basic Requirements:
```bash
sudo apt install gazebo11
```

### For ROS 2 Support:
```bash
sudo apt install ros-humble-desktop
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install python3-colcon-common-extensions
```

## 🔧 Troubleshooting

| Problem | Solution |
|---------|----------|
| Gazebo won't start | `killall -9 gzserver gzclient` then restart |
| Demo not moving | Wait 15-20 seconds for Gazebo to fully load |
| "Command not found" | Make scripts executable: `chmod +x *.sh` |
| Robot far from table | Already fixed! Robot at (0.15, 0, 0) |

## 📚 Learn More

- **ROS2_QUICK_START.md** - Complete ROS 2 setup
- **ROS_CONTROL_GUIDE.md** - Advanced control techniques
- **ASSIGNMENT_GUIDE.md** - Tips for presentations

## 🎓 Perfect For

- ✅ Robotics learning
- ✅ Project demonstrations
- ✅ Assignment submissions
- ✅ Algorithm testing
- ✅ Computer vision integration

## 📝 License

MIT License - Free for educational use

## 🔗 Repository

**GitHub**: https://github.com/piyushkumar0707/gazebo-simulation

---

**Quick Start**: `./run_auto_demo.sh` 🚀