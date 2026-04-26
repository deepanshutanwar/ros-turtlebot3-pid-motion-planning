#!/usr/bin/env python3

"""
motion_planner.py  (user-friendly version)
-------------------------------------------
Asks the user for each value separately with clear prompts.
Subscribes to /odom, publishes to /reference_pose.
"""

import rospy
import math
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

TOL_POS   = 0.10   # metres
TOL_ANGLE = 0.10   # radians

# ── Console colours (work on most Linux terminals) ──
class C:
    RESET  = '\033[0m'
    BOLD   = '\033[1m'
    CYAN   = '\033[96m'
    GREEN  = '\033[92m'
    YELLOW = '\033[93m'
    RED    = '\033[91m'
    BLUE   = '\033[94m'
    GRAY   = '\033[90m'

def banner():
    print(f"""
{C.CYAN}{C.BOLD}
╔══════════════════════════════════════════════════╗
║          TurtleBot3 Motion Planner               ║
║          Lab 3 - PID Controller                  ║
╚══════════════════════════════════════════════════╝
{C.RESET}""")

def divider():
    print(f"{C.GRAY}--------------------------------------------------{C.RESET}")

def ask_float(prompt, default=None):
    """Ask for a float value, re-prompting on bad input."""
    while not rospy.is_shutdown():
        suffix = f" [{default}]" if default is not None else ""
        raw = input(f"  {C.YELLOW}{prompt}{suffix}: {C.RESET}").strip()
        if raw == "" and default is not None:
            return float(default)
        try:
            return float(raw)
        except ValueError:
            print(f"  {C.RED}X  Please enter a number (e.g. 2.5){C.RESET}")

def ask_mode():
    """Ask for mode 0 or 1 with explanation."""
    print(f"\n  {C.YELLOW}Select control mode:{C.RESET}")
    print(f"  {C.BLUE}[0]{C.RESET} Sequential   ->  Turn  Drive  Orient  (cleaner path)")
    print(f"  {C.BLUE}[1]{C.RESET} Simultaneous ->  Turn + Drive at the same time (faster)")
    while not rospy.is_shutdown():
        raw = input(f"  {C.YELLOW}Mode (0 or 1){C.RESET} [0]: ").strip()
        if raw == "":
            return 0
        if raw in ("0", "1"):
            return int(raw)
        print(f"  {C.RED}X  Please enter 0 or 1{C.RESET}")

def confirm(xr, yr, theta_r_deg, mode):
    """Show a summary and ask for confirmation."""
    mode_name = "Sequential  (Turn->Drive->Orient)" if mode == 0 \
                else "Simultaneous (Turn+Drive)"
    print(f"""
{C.CYAN}  +------------------------------------------+
  |  Goal Summary                            |
  |  X position : {xr:>8.3f} m                 |
  |  Y position : {yr:>8.3f} m                 |
  |  Final angle: {theta_r_deg:>8.1f} deg               |
  |  Mode       : {mode} - {mode_name:<29}|
  +------------------------------------------+{C.RESET}""")
    ans = input(f"  {C.YELLOW}Send this goal? (y/n){C.RESET} [y]: ").strip().lower()
    return ans in ("", "y", "yes")


class MotionPlanner:

    def __init__(self):
        rospy.init_node('motion_planner', anonymous=False)

        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        self.ref_pub = rospy.Publisher(
            '/reference_pose', Float64MultiArray, queue_size=10, latch=True
        )
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.sleep(1.0)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def pos_error(self, xr, yr):
        return math.hypot(xr - self.x, yr - self.y)

    def angle_error(self, theta_r):
        err = theta_r - self.theta
        while err >  math.pi: err -= 2.0 * math.pi
        while err < -math.pi: err += 2.0 * math.pi
        return abs(err)

    def goal_reached(self, xr, yr, theta_r):
        return (self.pos_error(xr, yr)    <= TOL_POS and
                self.angle_error(theta_r) <= TOL_ANGLE)

    def show_current_pose(self):
        print(f"\n  {C.GRAY}Current robot pose:  "
              f"x={self.x:.3f} m   y={self.y:.3f} m   "
              f"theta={math.degrees(self.theta):.1f} deg{C.RESET}")

    def get_goal(self):
        """Interactive per-field input. Returns (xr, yr, theta_r_rad, mode)."""
        while not rospy.is_shutdown():
            divider()
            self.show_current_pose()
            print(f"\n  {C.BOLD}Enter the target pose:{C.RESET}")

            xr          = ask_float("Target X  (metres)")
            yr          = ask_float("Target Y  (metres)")
            theta_r_deg = ask_float("Final angle (degrees,  0 = facing +X axis)")
            mode        = ask_mode()

            theta_r_rad = math.radians(theta_r_deg)

            if confirm(xr, yr, theta_r_deg, mode):
                return xr, yr, theta_r_rad, mode

            print(f"\n  {C.YELLOW}Restarting input...{C.RESET}")

    def wait_for_goal(self, xr, yr, theta_r):
        """Poll odom and print a live progress line until the goal is reached."""
        rate    = rospy.Rate(5)
        spinner = ['|', '/', '-', '\\']
        i       = 0
        print()
        while not rospy.is_shutdown():
            ep   = self.pos_error(xr, yr)
            ea   = math.degrees(self.angle_error(theta_r))
            spin = spinner[i % len(spinner)]

            print(f"\r  {C.CYAN}{spin}{C.RESET}  "
                  f"dist={ep:.3f} m   "
                  f"angle_err={ea:.1f} deg   "
                  f"robot=({self.x:.2f}, {self.y:.2f}, "
                  f"{math.degrees(self.theta):.1f} deg)   ",
                  end='', flush=True)

            if self.goal_reached(xr, yr, theta_r):
                print(f"\n\n  {C.GREEN}{C.BOLD}Goal reached!{C.RESET}")
                print(f"  {C.GREEN}Final pos error  : {ep:.4f} m  "
                      f"(limit 0.10 m)  "
                      f"{'PASS' if ep <= 0.10 else 'FAIL'}{C.RESET}")
                print(f"  {C.GREEN}Final angle error: {ea:.2f} deg  "
                      f"(limit 5.73 deg)  "
                      f"{'PASS' if ea <= 5.73 else 'FAIL'}{C.RESET}")
                return
            i += 1
            rate.sleep()

    def run(self):
        banner()
        goal_count = 0

        while not rospy.is_shutdown():
            goal_count += 1
            print(f"\n{C.BOLD}  === Goal #{goal_count} ==={C.RESET}")

            xr, yr, theta_r, mode = self.get_goal()

            msg      = Float64MultiArray()
            msg.data = [xr, yr, theta_r, float(mode)]
            self.ref_pub.publish(msg)

            mode_str = "Sequential" if mode == 0 else "Simultaneous"
            print(f"\n  {C.BLUE}Executing goal #{goal_count}  "
                  f"[{mode_str} mode]...{C.RESET}")

            self.wait_for_goal(xr, yr, theta_r)

            print(f"\n  {C.GRAY}Press Enter to set a new goal...{C.RESET}")
            input()


if __name__ == '__main__':
    try:
        planner = MotionPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass