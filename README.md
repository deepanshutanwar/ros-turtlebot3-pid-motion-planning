# ROS TurtleBot3 PID Motion Planning

A closed-loop PID motion controller for the **TurtleBot3 Waffle Pi** robot, simulated in **Gazebo** using **ROS**. The robot navigates from its current pose to any user-specified target pose `(x, y, θ)` using two independently tuned PID controllers:

- Angular velocity controller
- Linear velocity controller

Both **sequential** and **simultaneous** control modes are implemented, tested, and verified against position and angle error bounds.

---

## Demo

Demo Video: https://youtu.be/RZjk_768i4s

---

## System Architecture

Two ROS nodes communicate over three topics:

```
motion_planner.py  --/reference_pose-->  pid_controller.py  --/cmd_vel-->  Gazebo
                   <-----------------------/odom-------------------------------
```

| Topic             | Type                  | Description                       |
|-------------------|-----------------------|-----------------------------------|
| `/reference_pose` | `Float64MultiArray`   | Target pose `[xr, yr, θr, mode]`  |
| `/odom`           | `nav_msgs/Odometry`   | Current robot pose `(x, y, θ)`    |
| `/cmd_vel`        | `geometry_msgs/Twist` | Linear + angular velocity commands|

---

## Control Modes

### Mode 0 — Sequential (Cleaner Path)

1. Turn to face the target point
2. Drive straight to the target
3. Turn to the final desired orientation

### Mode 1 — Simultaneous (Faster)

1. Drive and turn at the same time using both PIDs
2. Linear speed is scaled by `cos(angle_error)` to prevent the robot from arcing off course when heading error is large
3. Turn to the final desired orientation

---

## PID Gains

| Controller| Kp  | Ki    | Kd   | Output                       |
|-----------|-----|-------|------|------------------------------|
| Angular   | 3.0 | 0.002 | 0.8  | `angular.z` (max ±1.5 rad/s) |
| Linear    | 0.8 | 0.001 | 0.15 | `linear.x` (max 0.20 m/s)    |

Tuning method: Set Ki = Kd = 0 → increase Kp until oscillation → raise Kd to reduce overshoot → add small Ki to eliminate steady-state error.

---

## Project Structure

```
ros-turtlebot3-pid-motion-planning/
├── pid_controller.py       # PID controller node (angular + linear)
├── motion_planner.py       # Terminal UI node (goal input + live progress)
└── README.md
```

---

## Getting Started

### Prerequisites

- ROS Noetic (Ubuntu 20.04)
- TurtleBot3 packages
- Gazebo

```bash
sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-gazebo
export TURTLEBOT3_MODEL=waffle_pi
```

### Setup

```bash
# Create workspace and package
mkdir -p ~/ros-turtlebot3-pid-motion-planning/src
cd ~/ros-turtlebot3-pid-motion-planning/src
catkin_create_pkg pid_controller rospy std_msgs geometry_msgs nav_msgs

# Copy nodes
cp pid_controller.py pid_controller/src/
cp motion_planner.py pid_controller/src/

# Make executable
chmod +x pid_controller/src/pid_controller.py
chmod +x pid_controller/src/motion_planner.py

# Build
cd ..
catkin_make
source devel/setup.bash
```

### Run

**Terminal 1** — Launch Gazebo:
```bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

**Terminal 2** — Start the PID controller:
```bash
rosrun pid_controller pid_controller.py
```

**Terminal 3** — Start the motion planner:
```bash
rosrun pid_controller motion_planner.py
```

Enter a target `(x, y, angle, mode)` in the terminal and the robot will navigate to it.

---

## 🛠️ Tech Stack

- **ROS Noetic** — middleware and node communication
- **Gazebo** — robot simulation environment
- **Python 3** — controller and planner implementation
- **TurtleBot3 Waffle Pi** — differential drive robot model

---

## 👨‍💻 Author

**Deepanshu Tanwar**
