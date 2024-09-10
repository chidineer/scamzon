#include <rclcpp/rclcpp.hpp>  // includes rclcpp for creating nodes
#include "scan_matching_localizer/scan_to_image_node.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>  // includes message type for LaserScan
#include <geometry_msgs/msg/twist.hpp>  // includes message type for Twist (for robot motion)
#include <opencv2/opencv.hpp>  // includes OpenCV library for image processing
#include <yaml-cpp/yaml.h>  // includes the YAML-CPP library
#include <nav_msgs/msg/odometry.hpp>  // includes message type for Odometry
#include <fstream>  // includes fstream for file handling
#include <vector>  // includes vector container
#include <std_msgs/msg/float64.hpp>  // Include the Float64 message type
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>  // Correct type for AMCL pose




// class ScanToImageNode : public rclcpp::Node {  // defines the ScanToImageNode class inheriting from Node
// public:
ScanToImageNode::ScanToImageNode(const std::string& map_yaml_path) 
        : Node("scan_to_image_node"), angle_difference_(0.0), relative_orientation_(0.0), move_called_(false), first_image_captured_(false), second_image_captured_(false) {
        // Set up subscriber for laser scans
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanToImageNode::scanCallback, this, std::placeholders::_1));
        // Set up subscriber for AMCL pose
        amcl_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10, std::bind(&ScanToImageNode::amclPoseCallback, this, std::placeholders::_1));
        // Set up subscriber for odometry
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ScanToImageNode::odomCallback, this, std::placeholders::_1));
        
        // yaw publishers set up - mine, ground truth, amcl
         yaw_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/my_yaw", 10);
         yaw_publisher_tru_= this->create_publisher<geometry_msgs::msg::Vector3>("/true_yaw", 10);
         rmse_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/rmse_yaw", 10);
        // Create a publisher for yaw (Vector3 message)
        amcl_yaw_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/amcl_yaw_pub", 10);
        // Robot vel
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // Load the map from the specified YAML file
        loadMapManually(map_yaml_path);

        // Set robot initial pose
        initial_pose_ = cv::Point2d(map_origin_.x, map_origin_.y);
    }

    // Load the map from the specified YAML file
void ScanToImageNode::loadMapManually(const std::string& yaml_file) {
    // Open the YAML file as a standard text file
    std::ifstream file_stream(yaml_file);
    if (!file_stream.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Unable to open YAML file: %s", yaml_file.c_str());
        return;
    }

    std::string image_file;
    double resolution = 0.0;
    std::vector<double> origin(3, 0.0);  // Initialize with zeros (x, y, theta)

    // Read the YAML file line by line - while iterates thru lines
    std::string line;
    while (std::getline(file_stream, line)) {
        // Parse each line manually
        if (line.find("image:") != std::string::npos) {
            std::istringstream(line.substr(line.find(":") + 1)) >> image_file;
        } else if (line.find("resolution:") != std::string::npos) {
            std::istringstream(line.substr(line.find(":") + 1)) >> resolution;
        } else if (line.find("origin:") != std::string::npos) {
            std::istringstream line_stream(line.substr(line.find("[") + 1, line.find("]") - line.find("[") - 1));
            for (double& val : origin) {
                line_stream >> val;
            }
        }
    }

    // Close the file stream
    file_stream.close();

    // Manually check if the image file exists
    std::ifstream image_check(image_file);
    if (!image_check.good()) {
        RCLCPP_ERROR(this->get_logger(), "Map image file not found: %s", image_file.c_str());
        return;
    }

    // Load the map image
    map_image_ = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
    if (map_image_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load the map image from %s", image_file.c_str());
        return;
    }

    // Assign the resolution and origin
    map_resolution_ = resolution;
    map_origin_ = cv::Point2d(origin[0], origin[1]);

    RCLCPP_INFO(this->get_logger(), "Map successfully loaded from %s with resolution %f and origin (%f, %f)", yaml_file.c_str(), resolution, origin[0], origin[1]);
}


void ScanToImageNode::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    // Extract quaternion
    auto orientation = msg->pose.pose.orientation;
    
    // Convert quaternion to yaw
    tf2::Quaternion quat(orientation.x, orientation.y, orientation.z, orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw); //conv quat to matrix and then rpy

    // Publish the yaw
    geometry_msgs::msg::Vector3 yaw_msg;
    yaw_msg.x = yaw;  // Store yaw in x
    amcl_yaw_publisher_->publish(yaw_msg);

    // Log the yaw value
    RCLCPP_INFO(this->get_logger(), "Published AMCL Yaw: %f radians", yaw);
}


    // Callback function to handle incoming laser scans
    void ScanToImageNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Convert LaserScan to cv::Mat (polar coordinates to Cartesian)
    cv::Mat img = laserScanToMat(msg);  //laser scan msg used to create img C

    if (!first_image_captured_) {
        
        first_image_=img.clone();
        first_image_edges_ = processMapSection(initial_pose_);  // creates image B by edge extraction
        first_image_captured_ = true;
        // Display the first image and first_image_edges
        cv::imshow("First Image", first_image_);
        cv::imshow("First Image Edges", first_image_edges_);
        cv::waitKey(1); // Process GUI events and update the window
    } else if (!second_image_captured_) {
        second_image_ = img.clone();  // Store the laser scan as the second image (Image C)
        second_image_captured_ = true;
        cv::imshow("Second Image (Laser Scan)", second_image_); // show laser scan img
        cv::waitKey(1);
        // Calculate the yaw change between Image Band Image C - same as lab
        calculateYawChange();
        saveImages(first_image_edges_, second_image_);
        first_image_ = second_image_.clone(); //update the img B and get edges
        first_image_edges_ = detectEdges(first_image_);
        moveRobot();
    } else {
        // Update first and second images with the new laser scan and map section
        first_image_ = second_image_.clone();  // old laser scan
        second_image_ = img.clone();  // current laser scan
        
        // Display the new image C laser scn
        cv::imshow("Second Image (Laser Scan)", second_image_);
        cv::waitKey(1);

        calculateYawChange();
        saveImages(first_image_edges_, second_image_); // save images and then move
        moveRobot();

    }
    relative_orientation_ += angle_difference_;

    geometry_msgs::msg::Vector3 yaw;
    yaw.x = relative_orientation_;  // Use x to store the yaw
    yaw_publisher_->publish(yaw);

    // Log the total relative orientation in degrees
    RCLCPP_INFO(this->get_logger(), "Relative Orientation: %f degrees", relative_orientation_ * 180.0 / CV_PI);
}

double ScanToImageNode::quaternionToYaw(const geometry_msgs::msg::Quaternion &quat) {
    tf2::Quaternion q(
        quat.x,
        quat.y,
        quat.z,
        quat.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}


cv::Mat ScanToImageNode::processMapSection(const cv::Point2d& robot_pose) {
    int size = 100;  // Define the size of the section to extract (in pixels)
    int half_size = size / 2;  // Half the size for easier calculations

    // Ensure the region stays within the bounds of the map image
    int x_min = std::max(0, static_cast<int>(robot_pose.x) - half_size);
    int y_min = std::max(0, static_cast<int>(robot_pose.y) - half_size);
    int x_max = std::min(map_image_.cols, static_cast<int>(robot_pose.x) + half_size);
    int y_max = std::min(map_image_.rows, static_cast<int>(robot_pose.y) + half_size);

    // Create a new image section with the defined region
    cv::Mat map_section(size, size, map_image_.type(), cv::Scalar(255));  // Initialize with white background

    // Copy the pixel values manually by iterating through the map image within bounds
    for (int i = x_min; i < x_max; ++i) {
        for (int j = y_min; j < y_max; ++j) {
            int new_i = i - x_min;  // Local coordinate in map_section
            int new_j = j - y_min;  // Local coordinate in map_section
            map_section.at<uchar>(new_j, new_i) = map_image_.at<uchar>(j, i);  // Copy pixel value into map_section cv Mat
        }
    }

    // Apply Laplacian operator for edge detection
    cv::Mat edges;
    cv::Laplacian(map_section, edges, CV_16S, 3);  // Laplacian filter for edge detection
    cv::convertScaleAbs(edges, edges);  // Convert back to 8-bit for display

    return edges;  // Return the edges from the extracted map section
}


    // Function to extract edges from the map section using OpenCV
cv::Mat ScanToImageNode::detectEdges(const cv::Mat& image) {
    // Apply Sobel operator to detect gradients in both x and y directions
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y, edges;

    // Calculate gradients in the x direction
    cv::Sobel(image, grad_x, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
    // Calculate gradients in the y direction
    cv::Sobel(image, grad_y, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);

    // Convert gradients to absolute values
    cv::convertScaleAbs(grad_x, abs_grad_x);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    // Combine gradients (x and y) to get overall edges
    cv::Mat combined_gradients;
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, combined_gradients);

    // Apply a binary threshold to highlight strong edges
    cv::threshold(combined_gradients, edges, 50, 255, cv::THRESH_BINARY);

    return edges;
}

    // Function to convert laser scans to an image (Image C)
cv::Mat ScanToImageNode::laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
    int img_size = 500;  // Define the size of the image (500x500 pixels)
    float max_range = scan->range_max;  // Get the maximum range of the laser scan

    // Create a blank image (all black) of size 500x500 pixels with a single channel (grayscale)
    cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

    // Iterate through all the laser scan range values
    for (size_t i = 0; i < scan->ranges.size(); i++) {
        float range = scan->ranges[i];  // Get the range (distance) for this laser scan point

        // Skip invalid or out-of-range values (below min range or above max range)
        if (range < scan->range_min || range > scan->range_max) {
            continue;
        }

        // Calculate the angle for this laser scan point
        float angle = scan->angle_min + i * scan->angle_increment;

        // Convert polar coordinates (range, angle) to Cartesian coordinates (x, y)
        int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
        int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;

        // Check if the computed coordinates are within the image bounds
        if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
            image.at<uchar>(y, x) = 255;  // Set the pixel at (x, y) to white (255) if valid
        }
    }

    return image;  // Return the resulting image representing the laser scan
}

    // Function to estimate the robot's rotation relative to the map
    void ScanToImageNode::calculateYawChange() {
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(first_image_edges_, second_image_, srcPoints, dstPoints);

        if (srcPoints.size() < 3 || dstPoints.size() < 3) {
            RCLCPP_ERROR(this->get_logger(), "Not enough points for yaw estimation.");
            return;
        }

        try {
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (!transform_matrix.empty()) {
                angle_difference_ = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
                relative_orientation_ += angle_difference_;

                RCLCPP_INFO(this->get_logger(), "Estimated yaw change: %f degrees", relative_orientation_ * 180.0 / CV_PI);
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in yaw change calculation: %s", e.what());
        }
    }

    // Rotate the robot
    void ScanToImageNode::rotateRobot() {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.angular.z = 0.5;  // Set angular velocity for rotation
        cmd_publisher_->publish(twist_msg);
        rclcpp::sleep_for(std::chrono::seconds(2));  // Rotate for 2 seconds

        // Stop rotation
        twist_msg.angular.z = 0.0;
        cmd_publisher_->publish(twist_msg);
        RCLCPP_INFO(this->get_logger(), "Rotation completed.");
    }

    // Move the robot forward
    void ScanToImageNode::moveRobot() {
        double distance_moved = 0.0;
        double linear_speed = 0.1; 
        double angular_speed =0.5  ;

        auto twist_msg = geometry_msgs::msg::Twist();

        twist_msg.linear.x = linear_speed;
        twist_msg.angular.z = angular_speed;  // going for a circle

        cmd_publisher_->publish(twist_msg);
        rclcpp::sleep_for(std::chrono::seconds(1));  // Rotate for 2 seconds

        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        cmd_publisher_->publish(twist_msg);

        // Update the robot's position on the map
        initial_pose_.x += distance_moved / map_resolution_ * cos(relative_orientation_);
        initial_pose_.y += distance_moved / map_resolution_ * sin(relative_orientation_);
}


    // Callback function to handle incoming odometry data
void ScanToImageNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

         // Extract true yaw from odometry
    double true_yaw = quaternionToYaw(msg->pose.pose.orientation);

    // Store the true yaw
    true_yaw_ = true_yaw;

    // Create and publish the true yaw as Vector3 message
    geometry_msgs::msg::Vector3 yaw_msg;
    yaw_msg.x = true_yaw_;
    yaw_publisher_tru_->publish(yaw_msg);

    // Compute the error between the true yaw and estimated yaw (relative_orientation_)
    double error = relative_orientation_ - true_yaw_;
    
    // Calculate RMSE (for this single value, it's equivalent to the absolute error)
    double rmse = std::sqrt(error * error);  // Simplified since it's a single value

    // Create and publish the RMSE as Vector3 message
    geometry_msgs::msg::Vector3 rmse_msg;
    rmse_msg.x = rmse;  // Store RMSE in x
    rmse_publisher_->publish(rmse_msg);
}

    // Function to detect and match features between two images
void ScanToImageNode::detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                                std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        for (const auto& match : goodMatches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    // Save images for debugging and visualization
    void ScanToImageNode::saveImages(const cv::Mat& first_image_edges, const cv::Mat& second_image) {
        cv::imwrite("first_image_edges.png", first_image_edges);
        cv::imwrite("second_image.png", second_image);
        RCLCPP_INFO(this->get_logger(), "Imgs saved");
    }

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Please provide the arg  (map YAML file path).");
        return -1;
    }

    std::string map_yaml_path = argv[1]; //input as arg
    rclcpp::spin(std::make_shared<ScanToImageNode>(map_yaml_path)); // spin with arg constructor
    rclcpp::shutdown();
    return 0;
}
