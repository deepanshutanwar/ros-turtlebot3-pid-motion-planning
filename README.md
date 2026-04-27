# ros-turtlesim-swim-to-goal

Ever wondered how a robot figures out how to get from point A to point B on its own?

This project answers that using ROS and TurtleSim. You give the turtle a target coordinate, and it navigates there by itself — no manual control, no hardcoded path. Just math, sensors, and a control loop running 50 times per second.

---

## 🎥 Demo

Video walkthrough: [ADD YOUR VIDEO LINK HERE]

---

## 💡 The Idea

The core idea is simple: **the turtle always knows where it is, and it always knows where it wants to go — so it just keeps correcting itself until it gets there.**

At every timestep the node asks two questions:

- **How far am I from the goal?** → drive faster if far, slower if close
- **Am I facing the right direction?** → turn to correct the heading

This is called **proportional control** — the bigger the error, the bigger the correction. As the turtle gets closer, both errors shrink, and so do the velocities. The turtle naturally decelerates and stops right at the goal.

---

## ⚙️ How It Works

### 1. The turtle reads its own position

The node subscribes to `/turtle1/pose` which gives the current `(x, y, θ)` in real time.

### 2. Two errors are computed every loop

```
Error_position = sqrt((x_goal - x)² + (y_goal - y)²)
Error_angle    = atan2(sin(goal_angle - θ), cos(goal_angle - θ))
```

`Error_position` is the straight-line distance to the goal.
`Error_angle` is how far off the turtle's heading is from the direction of the goal.

### 3. Velocities are set proportional to the errors

```
Linear_velocity  = K_x * Error_position * cos(Error_angle)
Angular_velocity = K_z * Error_angle
```

The `cos(Error_angle)` term is the key design choice — it scales down the forward speed when the turtle is still turning. Without it, the turtle would drive forward even while facing the wrong way and arc off course. With it, the turtle turns first, then drives — producing a clean, direct path.

### 4. The loop runs until the turtle arrives

When `Error_position < 0.5`, the turtle stops and immediately asks for the next goal.

### Tuned Gains

| Gain | Value | Effect |
|---|---|---|
| `K_x` | 1.5 | Controls forward speed |
| `K_z` | 6.0 | Controls turning speed |

---

## 🔁 ROS Topics

| Topic | Type | Role |
|---|---|---|
| `/turtle1/pose` | `turtlesim/Pose` | Reads current turtle position |
| `/turtle1/cmd_vel` | `geometry_msgs/Twist` | Sends velocity commands |

---

## 🚀 Run It Yourself

### Requirements

- Ubuntu 20.04
- ROS Noetic
- TurtleSim

```bash
sudo apt install ros-noetic-turtlesim
```

### Setup

```bash
git clone https://github.com/YOUR-USERNAME/ros-turtlesim-swim-to-goal.git
cd ros-turtlesim-swim-to-goal
catkin_make
source devel/setup.bash
```

### Launch

Open three terminals:

```bash
# Terminal 1
roscore

# Terminal 2
rosrun turtlesim turtlesim_node

# Terminal 3
rosrun autoturtle swim_to_goal.py
```

Then enter a goal when prompted:
```
Enter x_goal (0-11): 7.0
Enter y_goal (0-11): 5.0
```

The turtle navigates there and asks for the next goal automatically.

---

## 📁 Project Structure

```
ros-turtlesim-swim-to-goal/
├── devel/
├── src/
│   ├── pid_controller/
│   │   ├── src/
│   │   │   ├── motion_planner
│   │   │   └── pid_controller
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── CMakeLists.txt
│
└── README.md
```

---

## 👨‍💻 Author

**Deepanshu Tanwar**
Autonomous Systems — Spring 2026

[LinkedIn](https://www.linkedin.com/in/YOUR-PROFILE) | [GitHub](https://github.com/YOUR-USERNAME)