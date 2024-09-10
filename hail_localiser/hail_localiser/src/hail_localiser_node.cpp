#include <rclcpp/rclcpp.hpp>  
#include <sensor_msgs/msg/laser_scan.hpp>  
#include <geometry_msgs/msg/twist.hpp>  
#include <opencv2/opencv.hpp>  
#include <yaml-cpp/yaml.h>  
#include <nav_msgs/msg/odometry.hpp>  
#include <std_msgs/msg/float64.hpp>  
#include <fstream>  
#include <vector>  

class LaserScanLocalizer : public rclcpp::Node {
public:
    LaserScanLocalizer(const std::string& yaml_path) 
        : Node("laser_scan_localizer"), yaw_variation_(0.0), total_orientation_(0.0), motion_initiated_(false) {
        // Subscribe to LaserScan data
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanLocalizer::processLaserScan, this, std::placeholders::_1));

        // Subscribe to Odometry data
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&LaserScanLocalizer::processOdom, this, std::placeholders::_1));

        // Publisher for movement commands
        command_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Publisher for yaw angle in degrees
        yaw_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/calculated_yaw", 10);

        // Load the map from the YAML file
        loadMapData(yaml_path);

        // Set initial position (assuming map origin as starting point)
        current_position_ = cv::Point2d(map_origin_.x, map_origin_.y);

        RCLCPP_INFO(this->get_logger(), "Node initialized with map at: %s", yaml_path.c_str());
    }

private:
    // Load map information from YAML file
    void loadMapData(const std::string& file) {
        YAML::Node map_file = YAML::LoadFile(file);

        std::string image_file = map_file["image"].as<std::string>();
        double resolution = map_file["resolution"].as<double>();
        std::vector<double> origin = map_file["origin"].as<std::vector<double>>();

        // Load PGM map file as grayscale image
        map_image_ = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
        if (map_image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map: %s", image_file.c_str());
            return;
        }

        map_resolution_ = resolution;
        map_origin_ = cv::Point2d(origin[0], origin[1]);

        RCLCPP_INFO(this->get_logger(), "Map loaded from: %s", file.c_str());
    }

    // Callback for processing laser scans
    void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        cv::Mat laser_image = convertScanToImage(scan_msg);  // Convert scan to image

        if (!initial_scan_done_) {
            // Initialize with first laser scan and map region
            initial_scan_ = extractMapRegion(current_position_);
            initial_scan_done_ = true;
            initial_edges_ = applyEdgeDetection(initial_scan_);
            
            // Display initial images
            cv::imshow("Initial Map Region", initial_scan_);
            cv::imshow("Initial Edges", initial_edges_);
            cv::waitKey(1);
        } else if (!new_scan_done_) {
            new_scan_ = laser_image.clone();
            new_scan_done_ = true;

            cv::imshow("Laser Scan Image", new_scan_);
            cv::waitKey(1);

            calculateYawChange();  // Compare edges and laser scan

            // Control robot based on yaw change
            rotateRobot();
            moveRobotForward();

            saveDebugImages(initial_edges_, new_scan_);

            // Prepare for next scan
            initial_scan_ = new_scan_.clone();
            initial_edges_ = applyEdgeDetection(initial_scan_);
        } else {
            rotateRobot();
            moveRobotForward();

            initial_scan_ = new_scan_.clone();
            new_scan_ = laser_image.clone();

            cv::imshow("Laser Scan Image", new_scan_);
            cv::waitKey(1);

            calculateYawChange();
            saveDebugImages(initial_edges_, new_scan_);
        }

        // Update total orientation based on yaw changes
        total_orientation_ += yaw_variation_;
        total_orientation_ = adjustAngle(total_orientation_);

        // Log and publish the yaw value
        double yaw_degrees = total_orientation_ * 180.0 / CV_PI;
        RCLCPP_INFO(this->get_logger(), "Total Yaw: %f degrees", yaw_degrees);

        // Publish the yaw angle
        std_msgs::msg::Float64 yaw_msg;
        yaw_msg.data = yaw_degrees;
        yaw_publisher_->publish(yaw_msg);
    }

    // Callback for processing odometry data
    void processOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Handle odometry data here if needed
    }

    // Extract a section of the map around the robot's current position
    cv::Mat extractMapRegion(const cv::Point2d& pose) {
        int section_size = 100;
        int x_start = std::max(0, static_cast<int>(pose.x - section_size / 2));
        int y_start = std::max(0, static_cast<int>(pose.y - section_size / 2));
        int x_end = std::min(map_image_.cols, static_cast<int>(pose.x + section_size / 2));
        int y_end = std::min(map_image_.rows, static_cast<int>(pose.y + section_size / 2));

        cv::Rect map_area(x_start, y_start, x_end - x_start, y_end - y_start);
        return map_image_(map_area).clone();
    }

    // Apply edge detection on an image using OpenCV
    cv::Mat applyEdgeDetection(const cv::Mat& img) {
        cv::Mat edges;
        cv::Canny(img, edges, 50, 150);
        return edges;
    }

    // Convert laser scans to an OpenCV image (Image from scan data)
    cv::Mat convertScanToImage(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        int img_size = 500;
        float max_scan_range = scan->range_max;
        cv::Mat laser_img = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float distance = scan->ranges[i];
            if (distance < scan->range_min || distance > scan->range_max) {
                continue;
            }

            float angle = scan->angle_min + i * scan->angle_increment;
            int x = static_cast<int>((distance * cos(angle)) * img_size / (2 * max_scan_range)) + img_size / 2;
            int y = static_cast<int>((distance * sin(angle)) * img_size / (2 * max_scan_range)) + img_size / 2;

            if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                laser_img.at<uchar>(y, x) = 255;
            }
        }

        return laser_img;
    }

    // Calculate yaw change by matching features between images
    void calculateYawChange() {
        std::vector<cv::Point2f> keypoints_map, keypoints_scan;
        matchFeaturesBetweenImages(initial_edges_, new_scan_, keypoints_map, keypoints_scan);

        if (keypoints_map.size() < 3 || keypoints_scan.size() < 3) {
            RCLCPP_ERROR(this->get_logger(), "Insufficient points to calculate yaw.");
            return;
        }

        try {
            cv::Mat affine_matrix = cv::estimateAffinePartial2D(keypoints_map, keypoints_scan);
            if (!affine_matrix.empty()) {
                yaw_variation_ = atan2(affine_matrix.at<double>(1, 0), affine_matrix.at<double>(0, 0));
                total_orientation_ += yaw_variation_;
                total_orientation_ = adjustAngle(total_orientation_);
            }
        } catch (const cv::Exception& ex) {
            RCLCPP_ERROR(this->get_logger(), "Yaw calculation error: %s", ex.what());
        }
    }

    // Rotate the robot based on calculated yaw change
    void rotateRobot() {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.angular.z = 0.5;
        command_publisher_->publish(twist_msg);
        rclcpp::sleep_for(std::chrono::seconds(2));

        // Stop rotation
        twist_msg.angular.z = 0.0;
        command_publisher_->publish(twist_msg);
        RCLCPP_INFO(this->get_logger(), "Rotation complete.");
    }

    // Move the robot forward after rotation
    void moveRobotForward() {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.1;
        command_publisher_->publish(twist_msg);
        rclcpp::sleep_for(std::chrono::seconds(1));

        // Stop movement
        twist_msg.linear.x = 0.0;
        command_publisher_->publish(twist_msg);
        RCLCPP_INFO(this->get_logger(), "Move forward completed.");
    }

    // Match features between the map and the laser scan images
    void matchFeaturesBetweenImages(const cv::Mat& img1, const cv::Mat& img2,
                                    std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2) {
        cv::Ptr<cv::ORB> orb_detector = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb_detector->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb_detector->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        cv::BFMatcher bf_matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> feature_matches;
        bf_matcher.match(descriptors1, descriptors2, feature_matches);

        std::sort(feature_matches.begin(), feature_matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        size_t top_matches = static_cast<size_t>(feature_matches.size() * 0.15);
        for (size_t i = 0; i < top_matches; ++i) {
            points1.push_back(keypoints1[feature_matches[i].queryIdx].pt);
            points2.push_back(keypoints2[feature_matches[i].trainIdx].pt);
        }
    }

    // Normalize the angle to stay within [-π, π]
    double adjustAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // Save the images for debugging purposes
    void saveDebugImages(const cv::Mat& edges_img, const cv::Mat& laser_img) {
        cv::imwrite("map_edges.png", edges_img);
        cv::imwrite("laser_image.png", laser_img);
        RCLCPP_INFO(this->get_logger(), "Images saved.");
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_publisher_;

    cv::Mat map_image_;  
    double map_resolution_;  
    cv::Point2d map_origin_;  
    cv::Point2d current_position_;  

    cv::Mat initial_scan_, new_scan_;  
    cv::Mat initial_edges_;  
    bool initial_scan_done_ = false;
    bool new_scan_done_ = false;

    double yaw_variation_;  
    double total_orientation_;  
    bool motion_initiated_;  
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Please provide the path to the map YAML file.");
        return -1;
    }

    std::string map_path = argv[1];
    rclcpp::spin(std::make_shared<LaserScanLocalizer>(map_path));
    rclcpp::shutdown();
    return 0;
}
