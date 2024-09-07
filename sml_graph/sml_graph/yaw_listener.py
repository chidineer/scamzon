"""
@package sml_graph
Documentation for the 'sml_graph' package.

This package subscribes to the /sml_yaw topic and records yaw data for 60 seconds, saving the results
to a CSV file and generating a graph of yaw over time.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import csv
import time
import os

class YawListener(Node):
    """
    A ROS2 node that listens to the /sml_yaw topic, records yaw data, and generates a CSV file and a graph.

    Attributes:
    ----------
    yaw_data : list
        Stores yaw data over time.
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
        Initializes the YawListener node, sets up the subscriber, and prepares to record data.
        """
        super().__init__('yaw_listener')

        # Create output directory if it doesn't exist
        self.output_dir = os.path.join(os.path.dirname(__file__), 'output')
        os.makedirs(self.output_dir, exist_ok=True)

        # Initialize subscriber to /sml_yaw topic
        self.subscription = self.create_subscription(
            Float64,
            '/sml_yaw',
            self.yaw_callback,
            10)

        # Variables to store data
        self.yaw_data = []
        self.start_time = time.time()

        # File to save CSV data
        self.csv_file = open(os.path.join(self.output_dir, 'yaw_data.csv'), 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time (s)', 'Yaw Angle (degrees)'])

        # Set a timer to stop after 60 seconds
        self.timer = self.create_timer(1.0, self.check_time)

    def yaw_callback(self, msg):
        """
        Callback function for the /sml_yaw topic.
        
        Parameters:
        ----------
        msg : Float64
            Message received from the /sml_yaw topic, containing the yaw angle in degrees.
        """
        elapsed_time = time.time() - self.start_time

        # Store the yaw data
        self.yaw_data.append((elapsed_time, msg.data))

        # Write to the CSV file
        self.csv_writer.writerow([elapsed_time, msg.data])

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
        Generates and saves a plot of yaw angle over time using the recorded data.
        """
        if len(self.yaw_data) > 0:
            times, yaw_angles = zip(*self.yaw_data)

            # Plot the yaw angle vs time
            plt.figure()
            plt.plot(times, yaw_angles, label='Yaw Angle (degrees)')
            plt.xlabel('Time (s)')
            plt.ylabel('Yaw Angle (degrees)')
            plt.title('Yaw Angle vs Time')
            plt.ylim([-180, 180])  # Set y-axis limits to -180 to 180 degrees
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
