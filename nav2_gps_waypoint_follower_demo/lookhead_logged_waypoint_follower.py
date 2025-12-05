import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
import time
from robot_localization.srv import FromLL
from rclpy.node import Node
from nav2_gps_waypoint_follower_demo.utils.gps_utils import latLonYaw2Geopose
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
import tf2_ros
from rclpy.time import Time
from rclpy.duration import Duration

def distance_2d(p1, p2):
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    return math.hypot(dx, dy)

class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return gepose_wps


class GpsWpCommander(Node):
    """
    Class to use nav2 gps waypoint follower to follow a set of waypoints logged in a yaml file
    """

    def __init__(self, wps_file_path):
        super().__init__('minimal_client_async')
        self.navigator = BasicNavigator("basic_navigator")

        # waypoint parser
        self.wp_parser = YamlWaypointParser(wps_file_path)

        # 创建对 /fromLL 服务的 client，用以把 GeoPose 中的经纬度转换为地图坐标（map frame 下的 x,y,z）
        self.localizer = self.create_client(FromLL, '/fromLL')
        while not self.localizer.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # tf buffer for querying robot pose in map frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def start_wpf(self):
        # Wait until Nav2 is active
        self.navigator.waitUntilNav2Active(localizer='controller_server')

        # Convert waypoints from LLA to map frame once
        geo_wps = self.wp_parser.get_wps()
        self.wpl = []
        for wp in geo_wps:
            req = FromLL.Request()
            req.ll_point.longitude = wp.position.longitude
            req.ll_point.latitude = wp.position.latitude
            req.ll_point.altitude = wp.position.altitude
            future = self.localizer.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position = future.result().map_point
            ps.pose.orientation = wp.orientation
            self.wpl.append(ps)

        num_points = len(self.wpl)
        if num_points == 0:
            self.get_logger().warn('No waypoints loaded, exiting')
            return

        # distance-triggered next-goal send using TF
        # threshold (meters) to decide when to issue the next goal
        distance_threshold = 2.0

        # start from first index and keep it across iterations so we cover all points
        index = 0
        self.get_logger().info("Starting waypoint loop...")

        while rclpy.ok():
            # ensure a goal is active for the current index
            self.get_logger().info(f"Sending waypoint index {index}")
            sent = self.navigator.followWaypoints([self.wpl[index]])
            self.get_logger().info(f"followWaypoints returned: {sent}")

            while not self.navigator.isTaskComplete() and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

                # lookup robot pose in map frame via tf
                try:
                    trans = self.tf_buffer.lookup_transform(
                        'map', 'base_link', Time(), timeout=Duration(seconds=0.1))
                    robot_x = trans.transform.translation.x
                    robot_y = trans.transform.translation.y
                except Exception:
                    # tf not available yet
                    continue

                # current goal position
                goal_pos = self.wpl[index].pose.position
                dx = robot_x - goal_pos.x
                dy = robot_y - goal_pos.y
                dist = math.hypot(dx, dy)

                # if within threshold, send next goal immediately
                if dist < distance_threshold:
                    index = (index + 1) % num_points
                    self.navigator.followWaypoints([self.wpl[index]])

            result = self.navigator.getResult()
            if result == 0:  # FollowWaypoints.Result.SUCCEEDED
                self.get_logger().info("Completed one loop, restarting...")
            else:
                self.get_logger().warn(f"Waypoint follower failed with code: {result}")
                # 等 1 秒再重试
                rclpy.spin_once(self, timeout_sec=1.0)

def main():
    rclpy.init()

    # allow to pass the waypoints file as an argument
    default_yaml_file_path = os.path.join(get_package_share_directory(
        "nav2_gps_waypoint_follower_demo"), "config", "gps_waypoints.yaml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    gps_wpf = GpsWpCommander(yaml_file_path)
    gps_wpf.start_wpf()


if __name__ == "__main__":
    main()