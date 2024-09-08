import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.pyplot as plt
import csv
import time
import os
import math

class YawListener(Node):
    """
    A ROS2 node that listens to /sml_yaw, /amcl_pose, and /odom topics, records yaw data,
    and generates a CSV file and a graph.

    Attributes:
    ----------
    yaw_data : list
        Stores yaw data over time from the /sml_yaw, /amcl_pose, and /odom topics.
    start_time : float
        The timestamp when recording started.
    csv_file : file
        The CSV file object where yaw data is recorded.
    csv_writer : csv.writer
        CSV writer object to write yaw data to a file.
    output_dir : str
        Directory where the CSV and graph will be saved.
    """

    def __init__(self):
        """
        Initializes the YawListener node, sets up the subscribers, and prepares to record data.
        """
        super().__init__('yaw_listener')

        # Create output directory if it doesn't exist
        self.output_dir = os.path.join(os.path.dirname(__file__), 'output')
        os.makedirs(self.output_dir, exist_ok=True)

        # Initialize subscribers to /sml_yaw, /amcl_pose, and /odom topics
        self.subscription_sml_yaw = self.create_subscription(
            Float64,
            '/sml_yaw',
            self.yaw_callback_sml,
            10)
        self.subscription_amcl_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.yaw_callback_amcl,
            10)
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.yaw_callback_odom,
            10)

        # Variables to store data
        self.yaw_data = []
        self.start_time = time.time()

        # File to save CSV data
        self.csv_file = open(os.path.join(self.output_dir, 'yaw_data.csv'), 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time (s)', 'Yaw Angle (SML)', 'Yaw Angle (AMCL)', 'Yaw Angle (Odom)'])

        # Initialize yaw values
        self.yaw_sml = None
        self.yaw_amcl = None
        self.yaw_odom = None

        # Set a timer to stop after 60 seconds
        self.timer = self.create_timer(1.0, self.check_time)

    def yaw_callback_sml(self, msg):
        """
        Callback function for the /sml_yaw topic.
        
        Parameters:
        ----------
        msg : Float64
            Message received from the /sml_yaw topic, containing the yaw angle in degrees.
        """
        self.yaw_sml = msg.data
        self.record_yaw()

    def yaw_callback_amcl(self, msg):
        """
        Callback function for the /amcl_pose topic. Converts quaternion to yaw angle.
        
        Parameters:
        ----------
        msg : PoseWithCovarianceStamped
            Message received from the /amcl_pose topic, containing the pose with orientation.
        """
        orientation_q = msg.pose.pose.orientation
        self.yaw_amcl = self.quaternion_to_yaw(orientation_q)
        self.record_yaw()

    def yaw_callback_odom(self, msg):
        """
        Callback function for the /odom topic. Converts quaternion to yaw angle.
        
        Parameters:
        ----------
        msg : Odometry
            Message received from the /odom topic, containing the orientation of the robot.
        """
        orientation_q = msg.pose.pose.orientation
        self.yaw_odom = self.quaternion_to_yaw(orientation_q)
        self.record_yaw()

    def quaternion_to_yaw(self, q):
        """
        Converts a quaternion to a yaw angle in degrees.
        
        Parameters:
        ----------
        q : geometry_msgs.msg.Quaternion
            Quaternion representing the orientation.

        Returns:
        -------
        float
            Yaw angle in degrees.
        """
        # Convert quaternion to yaw angle
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw)

    def record_yaw(self):
        """
        Records the yaw data to the CSV file if all yaw values are available.
        """
        if self.yaw_sml is not None and self.yaw_amcl is not None and self.yaw_odom is not None:
            elapsed_time = time.time() - self.start_time
            self.yaw_data.append((elapsed_time, self.yaw_sml, self.yaw_amcl, self.yaw_odom))
            self.csv_writer.writerow([elapsed_time, self.yaw_sml, self.yaw_amcl, self.yaw_odom])

    def check_time(self):
        """
        Checks if 60 seconds have passed and stops the recording. Saves the data and plots the graph.
        """
        elapsed_time = time.time() - self.start_time
        if elapsed_time >= 60:
            self.get_logger().info('Finished recording yaw data for 60 seconds.')

            # Close CSV file and stop recording
            self.csv_file.close()

            # Plot the yaw data and save the plot
            self.plot_yaw_data()

            # Shutdown the node after saving data and plotting
            rclpy.shutdown()

    def plot_yaw_data(self):
        """
        Generates and saves a plot of yaw angles over time using the recorded data.
        """
        if len(self.yaw_data) > 0:
            times, yaw_sml, yaw_amcl, yaw_odom = zip(*self.yaw_data)

            # Plot the yaw angles vs time
            plt.figure()
            plt.plot(times, yaw_sml, label='Yaw Angle (SML)', color='blue')
            plt.plot(times, yaw_amcl, label='Yaw Angle (AMCL)', color='green')
            plt.plot(times, yaw_odom, label='Yaw Angle (Odom)', color='red')
            plt.xlabel('Time (s)')
            plt.ylabel('Yaw Angle (degrees)')
            plt.title('Yaw Angle vs Time')
            plt.ylim([-180, 180])  # Set y-axis limits to -180 to 180 degrees
            plt.legend()
            plt.grid(True)

            # Save the plot as an image file
            plt.savefig(os.path.join(self.output_dir, 'yaw_plot.png'))
            plt.show()

def main(args=None):
    """
    Main function to initialize and spin the ROS2 node.
    
    Parameters:
    ----------
    args : list, optional
        Arguments passed to the node.
    """
    rclpy.init(args=args)
    yaw_listener = YawListener()

    # Spin the node to keep receiving messages
    rclpy.spin(yaw_listener)

if __name__ == '__main__':
    main()
