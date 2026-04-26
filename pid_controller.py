#!/usr/bin/env python3

"""
pid_controller.py
-----------------
Subscribes to:
    /reference_pose  (std_msgs/Float64MultiArray)  [xr, yr, theta_r, mode]
    /odom            (nav_msgs/Odometry)            current pose
Publishes to:
    /cmd_vel         (geometry_msgs/Twist)          linear + angular velocity

Mode 0: Turn to face target → drive to target → turn to final angle  (sequential)
Mode 1: Drive + turn simultaneously until (xr,yr) reached, then orient to theta_r
"""

import rospy
import math
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


# ─────────────────────────────────────────────
#  Tuneable PID gains
# ─────────────────────────────────────────────
# Angular controller  (controls /cmd_vel.angular.z)
KP_ANG = 3.0
KI_ANG = 0.002
KD_ANG = 0.8

# Linear controller   (controls /cmd_vel.linear.x)
KP_LIN = 0.8
KI_LIN = 0.001
KD_LIN = 0.15

# Tolerances
TOL_POS   = 0.05   # metres  – position error threshold
TOL_ANGLE = 0.05   # radians – angle error threshold

# Safety clamps
MAX_LINEAR  = 0.20   # m/s
MAX_ANGULAR = 1.5    # rad/s


# ─────────────────────────────────────────────
#  Helpers
# ─────────────────────────────────────────────
def normalize_angle(angle):
    """Wrap angle into (-pi, pi]."""
    while angle >  math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle


class PID:
    """Generic discrete PID controller."""

    def __init__(self, kp, ki, kd, max_output=None, min_output=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.reset()

    def reset(self):
        self.prev_error  = 0.0
        self.integral    = 0.0
        self.prev_time   = None

    def compute(self, error, current_time=None):
        if current_time is None:
            current_time = rospy.get_time()

        if self.prev_time is None:
            dt = 0.05
        else:
            dt = current_time - self.prev_time
            if dt <= 0:
                dt = 0.05

        self.integral   += error * dt
        derivative       = (error - self.prev_error) / dt
        output           = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_error  = error
        self.prev_time   = current_time

        # Clamp
        if self.max_output is not None:
            output = min(self.max_output, max(self.min_output, output))
        return output


# ─────────────────────────────────────────────
#  Main controller node
# ─────────────────────────────────────────────
class PIDController:

    def __init__(self):
        rospy.init_node('pid_controller', anonymous=False)

        # Current pose
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # Reference pose + mode
        self.xr     = None
        self.yr     = None
        self.theta_r = None
        self.mode   = 0
        self.active = False        # becomes True when a goal arrives

        # Internal FSM for Mode 0
        # States: 'idle', 'turn_to_target', 'drive', 'turn_to_final'
        self.state = 'idle'

        # PID objects
        self.ang_pid = PID(KP_ANG, KI_ANG, KD_ANG,
                           max_output= MAX_ANGULAR,
                           min_output=-MAX_ANGULAR)
        self.lin_pid = PID(KP_LIN, KI_LIN, KD_LIN,
                           max_output= MAX_LINEAR,
                           min_output= 0.0)         # never drive backwards

        # Publishers / Subscribers
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/reference_pose', Float64MultiArray,
                         self.reference_callback)

        self.rate = rospy.Rate(20)   # 20 Hz control loop
        rospy.loginfo("PID Controller ready.")

    # ── Callbacks ────────────────────────────

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def reference_callback(self, msg):
        data = msg.data
        if len(data) < 4:
            rospy.logwarn("reference_pose needs 4 values: xr yr theta_r mode")
            return
        self.xr      = data[0]
        self.yr      = data[1]
        self.theta_r = data[2]
        self.mode    = int(data[3])
        self.active  = True

        # Reset PIDs and FSM for new goal
        self.ang_pid.reset()
        self.lin_pid.reset()
        if self.mode == 0:
            self.state = 'turn_to_target'
        else:
            self.state = 'drive_mode1'
        rospy.loginfo(
            f"New goal → ({self.xr:.2f}, {self.yr:.2f}, "
            f"{math.degrees(self.theta_r):.1f}°)  mode={self.mode}"
        )

    # ── Control helpers ──────────────────────

    def pos_error(self):
        return math.hypot(self.xr - self.x, self.yr - self.y)

    def bearing_to_target(self):
        return math.atan2(self.yr - self.y, self.xr - self.x)

    def stop(self):
        self.vel_pub.publish(Twist())

    # ── Mode 0 steps ─────────────────────────

    def step_turn_to_target(self):
        """Rotate until robot faces the target point."""
        angle_err = normalize_angle(self.bearing_to_target() - self.theta)
        if abs(angle_err) < TOL_ANGLE:
            self.stop()
            self.ang_pid.reset()
            self.state = 'drive'
            rospy.loginfo("Mode0: facing target → start driving")
            return
        twist          = Twist()
        twist.angular.z = self.ang_pid.compute(angle_err)
        self.vel_pub.publish(twist)

    def step_drive(self):
        """Drive straight until position error is within tolerance."""
        dist = self.pos_error()
        if dist < TOL_POS:
            self.stop()
            self.lin_pid.reset()
            self.ang_pid.reset()
            self.state = 'turn_to_final'
            rospy.loginfo("Mode0: at target → orient to final angle")
            return
        # Keep heading locked on target while driving
        angle_err       = normalize_angle(self.bearing_to_target() - self.theta)
        twist           = Twist()
        twist.linear.x  = self.lin_pid.compute(dist)
        twist.angular.z = self.ang_pid.compute(angle_err)
        self.vel_pub.publish(twist)

    def step_turn_to_final(self):
        """Rotate to the desired final orientation."""
        angle_err = normalize_angle(self.theta_r - self.theta)
        if abs(angle_err) < TOL_ANGLE:
            self.stop()
            self.active = False
            self.state  = 'idle'
            rospy.loginfo("Mode0: goal ACHIEVED ✓")
            return
        twist           = Twist()
        twist.angular.z = self.ang_pid.compute(angle_err)
        self.vel_pub.publish(twist)

    # ── Mode 1 steps ─────────────────────────

    def step_drive_mode1(self):
        """Simultaneously control linear and angular velocity."""
        dist = self.pos_error()
        if dist < TOL_POS:
            self.stop()
            self.lin_pid.reset()
            self.ang_pid.reset()
            self.state = 'turn_to_final_m1'
            rospy.loginfo("Mode1: at target → orient to final angle")
            return
        angle_err       = normalize_angle(self.bearing_to_target() - self.theta)
        twist           = Twist()
        twist.linear.x  = self.lin_pid.compute(dist)
        # Scale linear speed down when heading is very off (avoids arcing away)
        twist.linear.x *= max(0.0, math.cos(angle_err))
        twist.angular.z = self.ang_pid.compute(angle_err)
        self.vel_pub.publish(twist)

    def step_turn_to_final_m1(self):
        """Final orientation step for Mode 1."""
        angle_err = normalize_angle(self.theta_r - self.theta)
        if abs(angle_err) < TOL_ANGLE:
            self.stop()
            self.active = False
            self.state  = 'idle'
            rospy.loginfo("Mode1: goal ACHIEVED ✓")
            return
        twist           = Twist()
        twist.angular.z = self.ang_pid.compute(angle_err)
        self.vel_pub.publish(twist)

    # ── Main loop ────────────────────────────

    def spin(self):
        while not rospy.is_shutdown():
            if self.active:
                {
                    'turn_to_target'  : self.step_turn_to_target,
                    'drive'           : self.step_drive,
                    'turn_to_final'   : self.step_turn_to_final,
                    'drive_mode1'     : self.step_drive_mode1,
                    'turn_to_final_m1': self.step_turn_to_final_m1,
                }.get(self.state, self.stop)()
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = PIDController()
        node.spin()
    except rospy.ROSInterruptException:
        pass