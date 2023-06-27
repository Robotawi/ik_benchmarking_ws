# ROS Workspace for IK Benchmarking

This workspace contains robot models and Inverse Kinematics (IK) benchmarking programs for performance comparison.

## Contents

1. **Robot Models**: Contains MoveIt config packages for various robot models
    - Kuka iiwa (7 DOF) arm

    - Universal Robot UR5 (6DOF) arm

    - NASA Valkyrie (44 DOF) humanoid

2. **IK Benchmarking Programs [WIP]**: Programs to measure and compare the performance of the IK solvers.
    - This [node](src/ik_benchmarks//src//run_ik_benchmarks.cpp) calculates average success rate and solving time.
    

## Getting Started

Clone the repo:

```
git clone https://github.com/Robotawi/ik_benchmarking_ws.git
```


Install dependencies and build the workspace:

```
cd ik_benchmarking_ws

rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

colcon build
```

Source the workspace:

```
source install/setup.bash
```

Launch an example robot arm demo:

```
ros2 launch iiwa_moveit_config demo.launch.py
```

Run a performance evaluation node (WIP)

```
ros2 run ik_benchmarks run_ik_benchmarks
```


## Running the benchmarks (WIP)

Run the following command to start the benchmarking program
```
ros2 launch package_name launch_file
```

