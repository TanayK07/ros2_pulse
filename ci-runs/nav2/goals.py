"""Set the initial pose and send a few goals so the planners and controllers do real work."""
import time

import rclpy
import rclpy.time
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
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
# The loopback sim has no amcl and only starts publishing map->odom once it has seen an
# initial pose, which it can miss while it is still starting, so keep sending it.
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, nav)
while not tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
    nav.setInitialPose(pose(nav, -2.0, -0.5))
    end = time.time() + 2.0
    while time.time() < end:
        rclpy.spin_once(nav, timeout_sec=0.1)
print('map -> base_link available', flush=True)
nav.waitUntilNav2Active(navigator='bt_navigator', localizer='bt_navigator')
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
