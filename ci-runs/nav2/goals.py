"""Set the initial pose and send a few goals so the planners and controllers do real work."""
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def pose(nav, x, y):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.w = 1.0
    return p


rclpy.init()
nav = BasicNavigator()
nav.setInitialPose(pose(nav, -2.0, -0.5))
nav.waitUntilNav2Active()
print('nav2 active', flush=True)
time.sleep(10)  # idle stretch, steady-state rates
for x, y in [(1.5, -0.5), (-1.5, 0.5), (0.5, 1.0), (-2.0, -0.5)]:
    nav.goToPose(pose(nav, x, y))
    t0 = time.time()
    while not nav.isTaskComplete():
        if time.time() - t0 > 60:
            nav.cancelTask()
            break
        time.sleep(0.5)
    print(f'goal ({x}, {y}): {nav.getResult()} after {time.time() - t0:.0f}s', flush=True)
time.sleep(10)
rclpy.shutdown()
