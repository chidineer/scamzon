#include "cylinder_detector.hpp"
#include <cmath>

CylinderDetector::CylinderDetector() : Node("cylinder_detector_node")
{
    clustering_threshold_ = 0.4; // Threshold for gradient change (tunable)
    cylinder_diameter_ = 0.3;    // Expected cylinder diameter in meters
    place_markers_ = true;       // Flag to control marker placement

    // Create a subscriber for the laser scan topic
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&CylinderDetector::scanCallback, this, std::placeholders::_1));

    // Create a publisher for visualizing the detected cylinder on the map (RViz)
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

    RCLCPP_INFO(this->get_logger(), "Cylinder Detector Node has started.");
}

// Callback function for laser scan messages
void CylinderDetector::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    // Detect the cylinder using the gradient method
    Point2D cylinder_position = detectCylinder(scan_msg);

    if (cylinder_position.x < 1e10 && cylinder_position.y < 1e10) // If a cylinder is detected
    {
        RCLCPP_INFO(this->get_logger(), "Cylinder detected at (%.2f, %.2f)", cylinder_position.x, cylinder_position.y);
        if (place_markers_)
        {
            publishMarker(cylinder_position);
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "No cylinder detected.");
    }
}

// Function to publish a marker for visualization in RViz
void CylinderDetector::publishMarker(const Point2D &cylinder_position)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_scan"; // Adjust frame_id according to your setup
    marker.header.stamp = this->now();
    marker.ns = "cylinder_detector";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the position of the cylinder
    marker.pose.position.x = cylinder_position.x;
    marker.pose.position.y = cylinder_position.y;
    marker.pose.position.z = 0.0; // Assume cylinder is at ground level

    // Set the cylinder's orientation and scale
    marker.pose.orientation.w = 1.0;
    marker.scale.x = cylinder_diameter_; // Width (diameter)
    marker.scale.y = cylinder_diameter_; // Depth (diameter)
    marker.scale.z = 1.0;                // Height of the cylinder

    // Set color to make it visible in RViz
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0; // Full opacity

    marker.lifetime = builtin_interfaces::msg::Duration(); // Marker lasts forever
    marker_publisher_->publish(marker);
    RCLCPP_INFO(this->get_logger(), "Marker Published at x: %f, y: %f", cylinder_position.x, cylinder_position.y);
}

// Detect the Cylinder Location using Gradient Method
CylinderDetector::Point2D CylinderDetector::detectCylinder(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    Point2D no_cylinder_found;
    no_cylinder_found.x = 1e10;
    no_cylinder_found.y = 1e10;

    // Add error handling to check scan message
    if (!scan_msg || scan_msg->ranges.empty())
    {
        return no_cylinder_found;
    }

    std::vector<Point2D> points;

    // Convert ranges to 2D points in the robot's coordinate frame
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
    {
        double range = scan_msg->ranges[i];
        // Skip invalid or infinite ranges
        if (range < scan_msg->range_min || range > scan_msg->range_max)
            continue;

        // Calculate the angle corresponding to the current range
        double angle = scan_msg->angle_min + i * scan_msg->angle_increment;

        // Convert to 2D point and store
        points.push_back(rangeToPoint(range, angle, i));
    }

    // Calculate gradients between consecutive points
    std::vector<double> gradients;
    for (size_t i = 1; i < points.size(); ++i)
    {
        double gradient = calculateDistance(points[i], points[i - 1]);
        gradients.push_back(gradient);
    }

    // Identify the sections where the gradient is relatively small, indicating a cylinder
    std::vector<Point2D> cylinder_points;
    for (size_t i = 0; i < gradients.size(); ++i)
    {
        if (gradients[i] < clustering_threshold_) // Small gradient indicates smooth surface
        {
            cylinder_points.push_back(points[i]);
        }
        else if (!cylinder_points.empty())
        {
            // If we have collected cylinder points, check if it matches the expected width
            double width = calculateClusterWidth(cylinder_points);
            if (std::abs(width - cylinder_diameter_) < 0.05) // Allow 5 cm tolerance
            {
                Point2D cylinder_center = calculateClusterCenter(cylinder_points);
                return cylinder_center;
            }
            cylinder_points.clear();
        }
    }

    // Check at the end in case the last set of points form a cylinder
    if (!cylinder_points.empty())
    {
        double width = calculateClusterWidth(cylinder_points);
        if (std::abs(width - cylinder_diameter_) < 0.05) // Allow 5 cm tolerance
        {
            Point2D cylinder_center = calculateClusterCenter(cylinder_points);
            return cylinder_center;
        }
    }

    return no_cylinder_found;
}

// Function to convert a range into a 2D point in the robot's coordinate frame
CylinderDetector::Point2D CylinderDetector::rangeToPoint(double range, double angle, int index)
{
    Point2D point;
    point.x = range * std::cos(angle);
    point.y = range * std::sin(angle);
    point.index = index;
    return point;
}

// Function to calculate the distance between two 2D points
double CylinderDetector::calculateDistance(const Point2D &p1, const Point2D &p2)
{
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

// Function to calculate the width of a cluster (distance between first and last point)
double CylinderDetector::calculateClusterWidth(const std::vector<Point2D> &cluster)
{
    if (cluster.size() < 2)
        return 0.0;

    const Point2D &first_point = cluster.front();
    const Point2D &last_point = cluster.back();
    return calculateDistance(first_point, last_point);
}

// Function to calculate the center of a cluster in the robot's frame
CylinderDetector::Point2D CylinderDetector::calculateClusterCenter(const std::vector<CylinderDetector::Point2D> &cluster)
{
    Point2D center = {0.0, 0.0, 0};

    for (const auto &point : cluster)
    {
        center.x += point.x;
        center.y += point.y;
    }

    // Compute the average (center) of the points
    center.x /= cluster.size();
    center.y /= cluster.size();

    return center;
}

// Main function
int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create a shared pointer to the CylinderDetector node
    auto node = std::make_shared<CylinderDetector>();

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
