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
import subprocess

class YawListener(Node):
    """
    A ROS2 node that listens to 6 different topics: AMCL, Odom, and 4 SML yaw topics 
    (one for each student except Charlize), records yaw data, and generates a single graph with all values.

    Attributes:
    ----------
    yaw_data : list
        Stores yaw data over time from the 6 topics.
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

        # Set the directory path where the output will be saved (this can be changed)
        self.output_dir = '/home/student/output'  # Change this path as necessary
        
        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)

        # Initialize subscribers for each student's SML yaw, AMCL, and Odom topics
        self.subscription_sml_yaw_jarred = self.create_subscription(
            Float64,
            '/sml_yaw_jarred',  # Replace with the actual topic name
            self.yaw_callback_jarred,
            10)

        self.subscription_sml_yaw_sohail = self.create_subscription(
            Float64,
            '/sml_yaw_sohail',  # Replace with the actual topic name
            self.yaw_callback_sohail,
            10)

        self.subscription_sml_yaw_chidalu = self.create_subscription(
            Float64,
            '/sml_yaw_chidalu',  # Replace with the actual topic name
            self.yaw_callback_chidalu,
            10)

        self.subscription_sml_yaw_tejas = self.create_subscription(
            Float64,
            '/sml_yaw_tejas',  # Replace with the actual topic name
            self.yaw_callback_tejas,
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
        self.csv_writer.writerow(['Time (s)', 'Yaw (Jarred)', 'Yaw (Sohail)',
                                  'Yaw (Chidalu)', 'Yaw (Tejas)', 'Yaw (AMCL)', 'Yaw (Odom)'])

        # Initialize yaw values for each topic
        self.yaw_jarred = None
        self.yaw_sohail = None
        self.yaw_chidalu = None
        self.yaw_tejas = None
        self.yaw_amcl = None
        self.yaw_odom = None

        # Set a timer to stop after 60 seconds
        self.timer = self.create_timer(1.0, self.check_time)

    def yaw_callback_jarred(self, msg):
        """Callback for Jarred's SML yaw topic."""
        self.yaw_jarred = msg.data
        self.record_yaw()

    def yaw_callback_sohail(self, msg):
        """Callback for Sohail's SML yaw topic."""
        self.yaw_sohail = msg.data
        self.record_yaw()

    def yaw_callback_chidalu(self, msg):
        """Callback for Chidalu's SML yaw topic."""
        self.yaw_chidalu = msg.data
        self.record_yaw()

    def yaw_callback_tejas(self, msg):
        """Callback for Tejas's SML yaw topic."""
        self.yaw_tejas = msg.data
        self.record_yaw()

    def yaw_callback_amcl(self, msg):
        """Callback function for the /amcl_pose topic. Converts quaternion to yaw angle."""
        orientation_q = msg.pose.pose.orientation
        self.yaw_amcl = self.quaternion_to_yaw(orientation_q)
        self.record_yaw()

    def yaw_callback_odom(self, msg):
        """Callback function for the /odom topic. Converts quaternion to yaw angle."""
        orientation_q = msg.pose.pose.orientation
        self.yaw_odom = self.quaternion_to_yaw(orientation_q)
        self.record_yaw()

    def quaternion_to_yaw(self, q):
        """Converts a quaternion to a yaw angle in degrees."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw)

    def record_yaw(self):
        """
        Records the yaw data to the CSV file if all yaw values are available.
        """
        if (self.yaw_jarred is not None and self.yaw_sohail is not None and 
            self.yaw_chidalu is not None and self.yaw_tejas is not None and 
            self.yaw_amcl is not None and self.yaw_odom is not None):

            elapsed_time = time.time() - self.start_time

            # Record all yaw values to CSV
            self.yaw_data.append((elapsed_time, self.yaw_jarred, self.yaw_sohail,
                                  self.yaw_chidalu, self.yaw_tejas, self.yaw_amcl, self.yaw_odom))
            self.csv_writer.writerow([elapsed_time, self.yaw_jarred, self.yaw_sohail,
                                      self.yaw_chidalu, self.yaw_tejas, self.yaw_amcl, self.yaw_odom])

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
        """Generates and saves a single plot with all yaw values over time."""
        if len(self.yaw_data) > 0:
            times, yaw_jarred, yaw_sohail, yaw_chidalu, yaw_tejas, yaw_amcl, yaw_odom = zip(*self.yaw_data)

            # Plot all six yaw values on one plot
            plt.figure()
            plt.plot(times, yaw_jarred, label='Jarred Deluca - SML Yaw', color='blue')
            plt.plot(times, yaw_sohail, label='Sohail Tariq - SML Yaw', color='red')
            plt.plot(times, yaw_chidalu, label='Chidalu Chukwuneke - SML Yaw', color='orange')
            plt.plot(times, yaw_tejas, label='Tejas Bhuta - SML Yaw', color='purple')
            plt.plot(times, yaw_amcl, label='AMCL Yaw', color='cyan', linestyle='--')
            plt.plot(times, yaw_odom, label='Odom Yaw', color='magenta', linestyle='--')
            plt.xlabel('Time (s)')
            plt.ylabel('Yaw Angle (degrees)')
            plt.title('Yaw Angle vs Time (All Students, AMCL, and Odom)')
            plt.ylim([-180, 180])
            plt.legend()
            plt.grid(True)

            # Save the plot
            plot_file = os.path.join(self.output_dir, 'all_yaw_values.png')
            print(f"Saving plot to {plot_file}")
            plt.savefig(plot_file)
            plt.close()

            # Check if the file exists
            if os.path.exists(plot_file):
                # Try opening the PNG file using subprocess for better compatibility
                try:
                    subprocess.run(['xdg-open', plot_file], check=True)
                except Exception as e:
                    print(f"Failed to open image: {e}")
            else:
                print("File not found, could not open the PNG file.")

def main(args=None):
    """Main function to initialize and spin the ROS2 node."""
    rclpy.init(args=args)
    yaw_listener = YawListener()

    # Spin the node to keep receiving messages
    rclpy.spin(yaw_listener)

if __name__ == '__main__':
    main()
