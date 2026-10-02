# sorting_arm Ros2 package


This repository contains a description for . 

## Installation

1. Source ROS2 Jazzy:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

2. Clone the repository into your ROS2 workspace:
   ```bash
   cd /path/to/your_ros2_ws/src
   git clone --recurse-submodules https://github.com/dhanush014/Robotic-Sorting-Arm.git
   ```
   (Already cloned without it? Run `git submodule update --init`.)

3. Install dependencies using rosdep:
   ```bash
   rosdep update && rosdep install -i --from-path . --rosdistro jazzy -y
   ```

4. Build the packages (dh_gripper_driver is skipped for now):
   ```bash
   colcon build --symlink-install --packages-skip dh_gripper_driver --allow-overriding ur_description
   ```

5. Source the setup files:
   ```bash
   source install/setup.bash
   ```

## Configuration


## Usage
Launch the full sorting cell (UR10e + AG95 on a pedestal, table, bins, objects and overhead RGB camera on `/sorting_camera/image`):
```bash
ros2 launch sorting_arm_gazebo sorting_sim.launch.py
```

Launch the UR10e arm in gazebo and rviz with ag-95 gripper attached, and also launch corresponding controllers:
```bash
ros2 launch sorting_arm sorting_arm_launch.py
```

To command the gripper to close, open another terminal and run this command:
```bash
ros2 topic pub --once /gripper_controller/commands std_msgs/msg/Float64MultiArray   "{data: [0.93, 0.93, 0.93, 0.93, 0.93, 0.93]}"
```

Similarly, to open the gripper, use:
```bash
ros2 topic pub --once /gripper_controller/commands std_msgs/msg/Float64MultiArray   "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

