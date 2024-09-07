#include <rclcpp/rclcpp.hpp>  // includes rclcpp for creating nodes
#include <sensor_msgs/msg/laser_scan.hpp>  // includes message type for LaserScan
#include <geometry_msgs/msg/twist.hpp>  // includes message type for Twist (for robot motion)
#include <opencv2/opencv.hpp>  // includes OpenCV library for image processing
#include <yaml-cpp/yaml.h>  // includes the YAML-CPP library
#include <nav_msgs/msg/odometry.hpp>  // includes message type for Odometry
#include <std_msgs/msg/float64.hpp>  // includes message type for Float64 (for yaw)
#include <fstream>  // includes fstream for file handling
#include <vector>  // includes vector container

class ScanMatcher : public rclcpp::Node {  // defines the ScanMatcher class inheriting from Node
public:
    ScanMatcher(const std::string& map_yaml_path) 
        : Node("scan_matcher"), angle_difference_(0.0), relative_orientation_(0.0), move_called_(false) {
        // Constructor initializing the node and variables

        // Set up subscriber for laser scans
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanMatcher::scanCallback, this, std::placeholders::_1));
        
        // Set up subscriber for odometry
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ScanMatcher::odomCallback, this, std::placeholders::_1));

        // Set up publisher for robot velocity commands
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Set up publisher for yaw in degrees
        yaw_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/sml_yaw", 10);

        // Load the map from the specified YAML file
        loadMapFromFile(map_yaml_path);

        // Set the initial robot pose to a known location of the map (assuming it's the origin)
        initial_pose_ = cv::Point2d(map_origin_.x, map_origin_.y);

        RCLCPP_INFO(this->get_logger(), "Scan Matcher Node started with map %s.", map_yaml_path.c_str());
    }

private:
    // Load the map from the specified YAML file
    void loadMapFromFile(const std::string& yaml_file) {
        YAML::Node map_config = YAML::LoadFile(yaml_file);

        std::string image_file = map_config["image"].as<std::string>();
        double resolution = map_config["resolution"].as<double>();
        std::vector<double> origin = map_config["origin"].as<std::vector<double>>();

        // Load the PGM image associated with the map
        map_image_ = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
        if (map_image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map image from %s", image_file.c_str());
            return;
        }

        map_resolution_ = resolution;
        map_origin_ = cv::Point2d(origin[0], origin[1]);

        RCLCPP_INFO(this->get_logger(), "Map loaded successfully from %s", yaml_file.c_str());
    }

    // Callback function to handle incoming laser scans
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Convert LaserScan to cv::Mat (polar coordinates to Cartesian)
        cv::Mat img = laserScanToMat(msg);  // Converts the laser scan to an image (Image C)

        if (!first_image_captured_) {
            // Set the initial robot pose to a known location of the map
            first_image_ = extractMapSection(initial_pose_);  // Extracts a section of the map around the robot (Image A)
            first_image_captured_ = true;

            // Extract edges from Image A using OpenCV (Image B)
            first_image_edges_ = extractEdges(first_image_);  // Applies edge detection to Image A
            
            // Display the first image and its edges
            cv::imshow("First Image", first_image_);
            cv::imshow("First Image Edges", first_image_edges_);
            cv::waitKey(1); // Process GUI events and update the window
        } else if (!second_image_captured_) {
            second_image_ = img.clone();  // Store the laser scan as the second image (Image C)
            second_image_captured_ = true;

            // Display the second image (laser scan)
            cv::imshow("Second Image (Laser Scan)", second_image_);
            cv::waitKey(1);

            // Calculate the yaw change between Image B (edges of map section) and Image C (laser scan)
            calculateYawChange();

            // Rotate and move the robot based on the yaw difference
            rotateRobot();
            moveForward();

            // Save images for debugging/visualization
            saveImages(first_image_edges_, second_image_);

            // Update first image with the second image for the next step
            first_image_ = second_image_.clone();
            first_image_edges_ = extractEdges(first_image_);
        } else {
            // If images are captured, we update the position and orientation

            // Rotate and move the robot
            rotateRobot();
            moveForward();

            // Update first and second images with the new laser scan and map section
            first_image_ = second_image_.clone();  // Use the previous second image as the new first image
            second_image_ = img.clone();  // Update the second image with the current laser scan
            
            // Display the new second image
            cv::imshow("Second Image (Laser Scan)", second_image_);
            cv::waitKey(1);

            // Calculate yaw change again
            calculateYawChange();

            // Save images after moving and updating the robot’s pose
            saveImages(first_image_edges_, second_image_);
        }

        // Update total relative orientation after each iteration
        relative_orientation_ += angle_difference_;
        relative_orientation_ = normalizeAngle(relative_orientation_);  // Normalize the total orientation

        // Log and publish the total relative orientation in degrees
        double relative_orientation_degrees = relative_orientation_ * 180.0 / CV_PI;
        RCLCPP_INFO(this->get_logger(), "Relative Orientation: %f degrees", relative_orientation_degrees);

        // Publish the yaw to /sml_yaw
        std_msgs::msg::Float64 yaw_msg;
        yaw_msg.data = relative_orientation_degrees;
        yaw_publisher_->publish(yaw_msg);
    }

    // Function to extract a section of the map around the robot
    cv::Mat extractMapSection(const cv::Point2d& robot_pose) {
        int size = 100;  // Define the size of the section to extract (in pixels)
        int x_start = std::max(0, int(robot_pose.x - size / 2));
        int y_start = std::max(0, int(robot_pose.y - size / 2));
        int x_end = std::min(map_image_.cols, int(robot_pose.x + size / 2));
        int y_end = std::min(map_image_.rows, int(robot_pose.y + size / 2));

        cv::Rect region(x_start, y_start, x_end - x_start, y_end - y_start);
        return map_image_(region).clone();  // Extract and return the section of the map
    }

    // Function to extract edges from the map section using OpenCV
    cv::Mat extractEdges(const cv::Mat& image) {
        cv::Mat edges;
        cv::Canny(image, edges, 50, 150);  // Apply Canny edge detection
        return edges;
    }

    // Function to convert laser scans to an image (Image C)
    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        int img_size = 500;
        float max_range = scan->range_max;
        cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            if (range < scan->range_min || range > scan->range_max) {
                continue;
            }

            float angle = scan->angle_min + i * scan->angle_increment;
            int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
            int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;

            if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                image.at<uchar>(y, x) = 255;
            }
        }

        return image;
    }

    // Function to estimate the robot's rotation relative to the map
    void calculateYawChange() {
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
                relative_orientation_ = normalizeAngle(relative_orientation_);
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in yaw change calculation: %s", e.what());
        }
    }

    // Rotate the robot
    void rotateRobot() {
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
    void moveForward() {
        double distance_moved = 0.0;
        double target_distance = 6.28; // Circumference of the circle (2 * PI * radius, with radius = 1m)
        double linear_speed = 0.1;  // Linear speed in m/s
        double angular_speed = linear_speed / 1.0;  // Angular speed, keeping radius of 1 meter
        double time_step = 0.1;  // Time step for the control loop

        auto twist_msg = geometry_msgs::msg::Twist();
        rclcpp::Rate rate(1.0 / time_step);

        twist_msg.linear.x = linear_speed;
        twist_msg.angular.z = angular_speed;  // Apply angular velocity for circular motion

        cmd_publisher_->publish(twist_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(time_step * 1000)));
        distance_moved += linear_speed * time_step;  // Update distance traveled

        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        cmd_publisher_->publish(twist_msg);

        // Update the robot's position on the map
        initial_pose_.x += distance_moved / map_resolution_ * cos(relative_orientation_);
        initial_pose_.y += distance_moved / map_resolution_ * sin(relative_orientation_);
    }

    // Function to normalize angles to the range [-π, π]
    double normalizeAngle(double theta) {
        while (theta > M_PI) {
            theta -= 2.0 * M_PI;
        }
        while (theta < -M_PI) {
            theta += 2.0 * M_PI;
        }
        return theta;
    }

    // Callback function to handle incoming odometry data
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    }

    // Function to detect and match features between two images
    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
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
    void saveImages(const cv::Mat& first_image_edges, const cv::Mat& second_image) {
        cv::imwrite("first_image_edges.png", first_image_edges);
        cv::imwrite("second_image.png", second_image);
        RCLCPP_INFO(this->get_logger(), "Images saved: first_image_edges.png and second_image.png");
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_publisher_;

    cv::Mat map_image_;  // Store the map image
    double map_resolution_;  // Store the map resolution
    cv::Point2d map_origin_;  // Store the origin of the map
    cv::Point2d initial_pose_;  // Store the initial pose of the robot

    cv::Mat first_image_, second_image_;  // Stores the first and second images
    cv::Mat first_image_edges_;  // Stores the edges of the first image
    bool first_image_captured_ = false;
    bool second_image_captured_ = false;

    double angle_difference_;  // Stores the yaw angle difference
    double relative_orientation_ = 0.0;  // Stores the relative orientation

    bool move_called_;  // Flag to indicate if the moveForward function has been called
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Please provide the map YAML file path as an argument.");
        return -1;
    }

    std::string map_yaml_path = argv[1];
    rclcpp::spin(std::make_shared<ScanMatcher>(map_yaml_path));
    rclcpp::shutdown();
    return 0;
}
