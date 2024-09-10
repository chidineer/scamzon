#ifndef SCAN_TO_IMAGE_NODE_HPP
#define SCAN_TO_IMAGE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>  // Include the Float64 message type
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>  // Correct type for AMCL pose

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // For converting quaternion to yaw

#include <vector>
#include <string>

class ScanToImageNode : public rclcpp::Node {
public:
    // Constructor that accepts the map YAML file path
    ScanToImageNode(const std::string& map_yaml_path);

private:
    // Function to load the map from a YAML file
    void loadMapManually(const std::string& yaml_file);

    // Callback function for processing laser scans
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // Callback function for processing odometry data (if needed)
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Function to convert laser scan data to an OpenCV image
    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan);

    // Function to calculate the yaw angle change between two images
    void calculateYawChange();

    // Function to rotate the robot
    void rotateRobot();

    // Function to move the robot forward
    void moveRobot();


    // Function to detect and match features between two images
    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                                std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints);

    // Function to extract a section of the map around the robot (for comparison)
    cv::Mat processMapSection(const cv::Point2d& robot_pose);

    // Function to extract edges from the map section
    cv::Mat detectEdges(const cv::Mat& image);

    // Function to save images (for debugging and visualization)
    void saveImages(const cv::Mat& first_image_edges, const cv::Mat& second_image);

    double quaternionToYaw(const geometry_msgs::msg::Quaternion &quat);
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // ROS2 subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr yaw_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr yaw_publisher_tru_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr amcl_yaw_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr rmse_publisher_;


    // Variables to store map data
    cv::Mat map_image_;   // Store the map image
    double map_resolution_;  // Store the map resolution
    cv::Point2d map_origin_; // Store the origin of the map
    cv::Point2d initial_pose_;  // Variable to store the initial robot pose

    // Variables for processing images
    cv::Mat first_image_, second_image_;
    cv::Mat first_image_edges_;  // Stores the edges of the first image
    bool first_image_captured_;
    bool second_image_captured_;

    // Variables for tracking orientation
    double angle_difference_;  // Stores the yaw angle difference
    double relative_orientation_ = 0.0;  // Stores the relative orientation

    // Flag to prevent multiple calls to moveForward
    bool move_called_;

    double true_yaw_;
};

#endif  // SCAN_TO_IMAGE_NODE_HPP
