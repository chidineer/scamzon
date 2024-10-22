import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
import json
import os

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(PoseArray, '/scam_goals', 10)
        self.timer = self.create_timer(1.0, self.publish_goals)  # Periodically publish goals
        self.goals_file = "goals.json"  # Path to the JSON file

    def load_goals_from_json(self):
        # Load the JSON file with goals
        if not os.path.exists(self.goals_file):
            self.get_logger().error(f"Goals file {self.goals_file} not found!")
            return None

        with open(self.goals_file, 'r') as f:
            try:
                data = json.load(f)
                return data['poses']
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Failed to decode JSON: {e}")
                return None

    def publish_goals(self):
        # Load goals from JSON file
        poses_data = self.load_goals_from_json()
        if poses_data is None:
            return

        # Create a PoseArray message
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'map'

        # Fill the PoseArray with poses from the JSON file
        for pose_data in poses_data:
            pose = Pose()
            pose.position.x = pose_data['position']['x']
            pose.position.y = pose_data['position']['y']
            pose.position.z = pose_data['position']['z']
            pose.orientation.x = pose_data['orientation']['x']
            pose.orientation.y = pose_data['orientation']['y']
            pose.orientation.z = pose_data['orientation']['z']
            pose.orientation.w = pose_data['orientation']['w']
            pose_array.poses.append(pose)

        # Publish the PoseArray message
        self.publisher_.publish(pose_array)
        self.get_logger().info(f"Published {len(pose_array.poses)} goals to /scam_goals")

def main(args=None):
    rclpy.init(args=args)
    goal_publisher = GoalPublisher()
    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
