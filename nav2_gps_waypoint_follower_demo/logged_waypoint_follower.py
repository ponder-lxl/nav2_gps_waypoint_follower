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
from geometry_msgs.msg import PoseStamped


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
        self.wp_parser = YamlWaypointParser(wps_file_path)
        # 创建对 /fromLL 服务的 client，用以把 GeoPose 中的经纬度转换为地图坐标（map frame 下的 x,y,z）
        self.localizer = self.create_client(FromLL,  '/fromLL')
        while not self.localizer.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def start_wpf(self):
        """
        Function to start the waypoint following
        """
        # 确保 nav2 的 action servers（controller_server 等）运行并可用于接收导航任务。此函数阻塞直到 Nav2 活跃
        self.navigator.waitUntilNav2Active(localizer='controller_server')
        # 得到 GeoPose 列表（经纬度 + orientation）
        wps = self.wp_parser.get_wps()

        wpl = [] # 存储导航的地图坐标点
        for wp in wps:
            self.req = FromLL.Request()
            self.req.ll_point.longitude = wp.position.longitude
            self.req.ll_point.latitude = wp.position.latitude
            self.req.ll_point.altitude = wp.position.altitude

            log = 'long{:f}, lat={:f}, alt={:f}'.format(self.req.ll_point.longitude, self.req.ll_point.latitude, self.req.ll_point.altitude)
            self.get_logger().info(log)
            # 发起异步服务调用到 /fromLL，服务会返回 map_point（以地图坐标系表示的 x,y,z）
            self.future = self.localizer.call_async(self.req)
            # 阻塞并处理节点回调直到这个 future 完成（即等待服务响应）
            rclpy.spin_until_future_complete(self, self.future)

            self.resp = PoseStamped()
            self.resp.header.frame_id = 'map'
            self.resp.header.stamp = self.get_clock().now().to_msg()
            self.resp.pose.position = self.future.result().map_point

            log = 'x={:f}, y={:f}, z={:f}'.format(self.future.result().map_point.x, self.future.result().map_point.y, self.future.result().map_point.z)
            self.get_logger().info(log)
            
            self.resp.pose.orientation = wp.orientation
            wpl += [self.resp]
        # 把 PoseStamped 列表提交给 Nav2（通过 FollowWaypoints action）。
        self.navigator.followWaypoints(wpl)
        print("wps completed successfully")

def main():
    rclpy.init()

    # allow to pass the waypoints file as an argument
    default_yaml_file_path = os.path.join(get_package_share_directory(
        "nav2_gps_waypoint_follower_demo"), "config", "demo_waypoints.yaml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    gps_wpf = GpsWpCommander(yaml_file_path)
    gps_wpf.start_wpf()


if __name__ == "__main__":
    main()