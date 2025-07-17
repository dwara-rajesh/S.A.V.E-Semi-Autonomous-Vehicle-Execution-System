#!/home/dwarakesh/ROS2Projects/ros2_venv/bin/python3
import rclpy, math,os
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
import pandas as pd

class WaypointFollower(Node):
    def __init__(self,csv_file):
        super().__init__('waypointfollower')
        self.get_logger().info(f'Way point CSV file path: {csv_file}')
        self.goal_tolerance = 0.5
        df = pd.read_csv(csv_file, names = ['x','y'])
        img_width = 1920
        img_height = 1080
        target_min = -25
        target_max = 25
        target_range = target_max - target_min

        df['x_scaled'] = (df['x'] / img_width) * target_range + target_min
        df['y_scaled'] = target_max - (df['y'] / img_height) * target_range

        self.waypoints = df[['x_scaled', 'y_scaled']].to_dict('records')
        self.current_index = 0

        self.current_pose = None

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose2D, '/robot_pose', self.pose_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints.')

    def pose_callback(self, msg: Pose2D):
        self.current_pose = msg

    def control_loop(self):
        if self.current_pose is None:
            return

        if self.current_index >= len(self.waypoints):
            self.stop_robot()
            self.get_logger().info('All Waypoints reached, stopping')
            self.timer.cancel()
            return

        target = self.waypoints[self.current_index]
        pose = self.current_pose

        dx = target['x_scaled'] - pose.x
        dy = target['y_scaled'] - pose.y
        distance = math.hypot(dx, dy)

        if distance < self.goal_tolerance:
            self.get_logger().info(f'Waypoint {self.current_index} reached.')
            self.current_index += 1
            return

        target_theta = math.atan2(dy, dx)
        angle_error = target_theta - pose.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # Normalize angle

        K_linear = 0.5
        K_angular = 2.0
        max_linear = 1.0
        max_angular = 3.0

        linear_vel = K_linear * distance
        angular_vel = K_angular * angle_error

        linear_vel = max(-max_linear, min(max_linear, linear_vel))
        angular_vel = max(-max_angular, min(max_angular, angular_vel))

        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

def get_recent_csv(folder):
    files = [os.path.join(folder, f) for f in os.listdir(folder) if os.path.isfile(os.path.join(folder, f))]
    if not files:
        return None
    return max(files, key=os.path.getctime)

def main(args=None):
    rclpy.init(args=args)
    folderpath = 'waypointcsvs'
    node = WaypointFollower(get_recent_csv(folderpath))
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
