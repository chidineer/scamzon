#ifndef CYLINDER_DETECTOR_HPP
#define CYLINDER_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <string>

class CylinderDetector : public rclcpp::Node
{
public:
    CylinderDetector();

    struct Point2D
    {
        double x;
        double y;
        int index; // To keep track of the original range index
    };

    struct Pose2D
    {
        double x;
        double y;
        double theta;
    } robot_pose_;

private:
    // Callback for laser scan messages
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    // Function to publish the detected cylinder marker
    void publishMarker(const Point2D &cylinder_position);

    // Cylinder detection using gradient method
    Point2D detectCylinder(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    // Helper functions
    Point2D rangeToPoint(double range, double angle, int index);
    Point2D calculateClusterCenter(const std::vector<Point2D> &cluster);
    double calculateDistance(const Point2D &p1, const Point2D &p2);
    double calculateClusterWidth(const std::vector<Point2D> &cluster);

    // ROS2 subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    // Parameters for detection
    double clustering_threshold_;
    double cylinder_diameter_;
    bool place_markers_;
};

#endif // CYLINDER_DETECTOR_HPP
