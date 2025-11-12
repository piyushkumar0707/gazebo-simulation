# Simple 3-DOF Arm Gazebo Example

This workspace contains a minimal Gazebo simulation of a simple 3-axis robot arm (base rotation, shoulder, elbow), a wrist-mounted camera, and a simple two-finger gripper (fixed). There's also a table and a small cube to interact with.

Files added:
- `models/simple_3dof_arm/model.config` - model metadata
- `models/simple_3dof_arm/model.sdf` - SDF model of the arm with joints and camera sensor
- `worlds/simple_table_world.world` - world that includes ground_plane, sun, table, the robot (model://simple_3dof_arm) and a cube
- `run_gazebo.sh` - helper script to run Gazebo with the correct model path

Quick start (Linux):

1. Install Gazebo (classic) if you don't have it. On Ubuntu this typically means `sudo apt install gazebo11` (version may vary).

2. From a terminal in the `simple_3dof_arm` folder run:

```bash
./run_gazebo.sh
```

This sets `GAZEBO_MODEL_PATH` to the `models` folder and launches Gazebo with the provided world.

Controlling joints (easy):
- Once the world is open, use Gazebo's GUI "Model" tools. In recent Gazebo versions you can select the model and open the "Joints" or "Joint Control" panel to move joints with sliders.
- Alternatively, right-click the model in the scene and use the interactive tools to rotate joints.

Camera:
- The camera is mounted on the wrist. In the Gazebo GUI you can open the camera view from the Sensors panel (or it may appear automatically in the left pane). You can also right-click the camera in the scene list and view it.

Notes & next steps:
- The gripper fingers are currently fixed. If you want actuated fingers we can change those joints to revolute or prismatic and add a simple controller plugin.
- If you want ROS integration (for programmatic control via topics/services), I can add a `ros` launch file and the small `gazebo_ros` plugins for joint control.

If you want, I can now:
- Add simple actuated gripper joints and a tiny plugin to drive them, or
- Add a small Python/ROS example to publish target joint angles, or
- Convert to a ROS package with ros_control controllers.

Tell me which next step you'd like.