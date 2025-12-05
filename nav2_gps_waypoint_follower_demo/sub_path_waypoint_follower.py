import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import math
import tf2_ros
from rclpy.time import Time
from rclpy.duration import Duration

EARTH_RADIUS = 6378137.0

def ll_to_xy(lat, lon, ref_lat, ref_lon):
    """Convert latitude/longitude to local x/y coordinates using simple projection"""
    d_lat = math.radians(lat - ref_lat)
    d_lon = math.radians(lon - ref_lon)
    x = EARTH_RADIUS * d_lon * math.cos(math.radians(ref_lat))
    y = EARTH_RADIUS * d_lat
    return x, y

def distance_2d(p1, p2):
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    return math.hypot(dx, dy)

class GPSPathFollower(BasicNavigator):
    def __init__(self):
        super().__init__('gps_path_follower')

        # Subscribe to GPS Path - callback only saves the path
        self.create_subscription(
            Path,
            '/cmd/target_path',
            self.path_callback,
            10
        )
        
        # Publisher for converted map-path so RViz can display it
        self.map_path_pub = self.create_publisher(Path, '/map/target_path', 10)

        # Publisher for status
        self.status_pub = self.create_publisher(GoalStatus, '/nav/status', 10)

        # TF buffer and listener for looking up robot pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize state variables
        self.current_path = None
        self.wpl = []
        self.path_processed = False
    
    def publish_status(self, status_code: int):
        msg = GoalStatus()
        msg.status = status_code
        self.status_pub.publish(msg)

    def path_callback(self, msg: Path):
        """ Callback that only saves the received path """
        self.current_path = msg

    def run(self):
        """ Main processing loop """
        # Wait for Nav2 to become active
        self.get_logger().info("Waiting for Nav2 to become active...")
        self.waitUntilNav2Active(localizer="controller_server")
        self.get_logger().info("GPSPathFollower ready. Waiting for /cmd/target_path ...")

        # Wait for path to be received
        while rclpy.ok() and self.current_path is None:
            rclpy.spin_once(self, timeout_sec=0.2)

        if self.current_path is None:
            self.get_logger().error("No path received, exiting")
            return

        # Process the path
        msg = self.current_path
        num_points = len(msg.poses)
        self.get_logger().info(f"Received GPS Path with {num_points} poses")

        if num_points == 0:
            self.get_logger().warn("Received empty path!")
            return

        # Get reference point from first waypoint
        first_pose = msg.poses[0]
        ref_lon = first_pose.pose.position.x
        ref_lat = first_pose.pose.position.y
        self.get_logger().info(f"Reference point: lat={ref_lat:.7f}, lon={ref_lon:.7f}")

        waypoints = []

        # Process each GPS waypoint
        for i, p in enumerate(msg.poses):
            lon = p.pose.position.x
            lat = p.pose.position.y
            alt = p.pose.position.z

            log = f"[GPS→map] wp{i}: lon={lon:.7f}, lat={lat:.7f}, alt={alt:.2f}"
            self.get_logger().info(log)

            # Convert lon/lat to map coordinates using local projection
            x, y = ll_to_xy(lat, lon, ref_lat, ref_lon)

            # log = f"[GPS→map] wp{i}: x={x:.2f}, y={y:.2f}, z=0.0"
            # self.get_logger().info(log)

            # Create PoseStamped for Nav2
            resp = PoseStamped()
            resp.header.frame_id = 'map'
            resp.header.stamp = self.get_clock().now().to_msg()
            resp.pose.position.x = x
            resp.pose.position.y = y
            resp.pose.position.z = 0.0

            # orientation from original pose
            resp.pose.orientation = p.pose.orientation

            waypoints.append(resp)

        # print(waypoints)
        # Save converted map-frame waypoints for navigation use
        self.wpl = waypoints

        # Publish the converted path so RViz can visualize the map path
        map_path = Path()
        map_path.header.frame_id = 'map'
        map_path.header.stamp = self.get_clock().now().to_msg()
        map_path.poses = waypoints
        self.map_path_pub.publish(map_path)

        distance_threshold = 2.0
        index = 0
        self.get_logger().info("Starting waypoint loop...")

        while rclpy.ok():
            self.get_logger().info(f"Sending waypoint index {index}")
            self.publish_status(3)
            sent = self.followWaypoints([self.wpl[index]])
            self.get_logger().info(f"followWaypoints returned: {sent}")

            while not self.isTaskComplete() and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

                try:
                    trans = self.tf_buffer.lookup_transform(
                        'map', 'odin1_base_link', Time(), timeout=Duration(seconds=0.1))
                    robot_x = trans.transform.translation.x
                    robot_y = trans.transform.translation.y
                except Exception:
                    continue

                # current goal position
                goal_pos = self.wpl[index].pose.position
                dx = robot_x - goal_pos.x
                dy = robot_y - goal_pos.y
                dist = math.hypot(dx, dy)

                # if within threshold, send next goal immediately
                if dist < distance_threshold:
                    index = (index + 1) % num_points
                    self.followWaypoints([self.wpl[index]])
                    self.publish_status(4)

            result = self.getResult()
            if result == 0:  # FollowWaypoints.Result.SUCCEEDED
                self.publish_status(4)
                self.get_logger().info("Completed one loop, restarting...")
            else:
                self.publish_status(6)
                self.get_logger().warn(f"Waypoint follower failed with code: {result}")
                # 等 1 秒再重试
                rclpy.spin_once(self, timeout_sec=1.0)


def main():
    rclpy.init()
    navigator = GPSPathFollower()
    navigator.run()
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
