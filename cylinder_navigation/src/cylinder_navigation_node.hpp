#ifndef CYLINDER_NAVIGATION_NODE_HPP
#define CYLINDER_NAVIGATION_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <visualization_msgs/msg/marker.hpp>
#include <mutex>
#include <cmath> 
#include <chrono>
#include <thread>
using namespace std::placeholders;

class CylinderNavigationNode : public rclcpp::Node
{
public:
    CylinderNavigationNode();

private:

    enum class State {
        WAITING_FOR_GOAL,
        WAITING_FOR_CYLINDER,
        STOP_NAVIGATION,
        CREATE_WAYPOINTS,
        SEND_WAYPOINTS,
        MONITOR_WAYPOINTS,
        RETURN_TO_ORIGINAL_GOAL,
    };

    State state_ = State::WAITING_FOR_GOAL;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr cylinder_detected_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    

    bool first_detection_flag_ = true;
    bool waypoints_finished_ = false;
    std::mutex mtx_;
    std::mutex feedback_mutex_;
    geometry_msgs::msg::Pose cylinder_pose_;
    double distance_from_cylinder_; //adjust as necessary
    geometry_msgs::msg::Pose pose_when_detected_;
    geometry_msgs::msg::Pose last_amcl_pose_;
    sensor_msgs::msg::LaserScan scan_msg_;
    geometry_msgs::msg::Pose original_goal_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback_;
    double threshold_; //adjust as necessary
    std::chrono::steady_clock::time_point start_time_;
    int waypoint_index_ = 0;
    int num_waypoints_; //adjust as necessary (goes back to first waypoint)
    std::shared_ptr<nav_msgs::msg::Path> plan_msg_;
    bool first_plan_flag_ = true;
    bool amcl_updated_flag_ = false;

    void cylinderDetectedCallback(const visualization_msgs::msg::Marker::SharedPtr msg);

    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    void planCallback(const nav_msgs::msg::Path::SharedPtr msg);

    void timer_callback();

    void updateState();

    void stopNavigation();

    void publishOriginalGoal(float x, float y, float yaw);

    void createWaypointsAroundCylinder();

    void sendWaypointsToNav2();

    void goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle);

    void feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,  
                            const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);

    void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);    

    void checkWaypointsFinished();

    geometry_msgs::msg::Quaternion createQuaternionFromYaw(float yaw);

    double quatToYaw(const geometry_msgs::msg::Quaternion& quaternion);

    double linearDistance(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2);

    geometry_msgs::msg::Pose newPoseAtDistance(const geometry_msgs::msg::Pose& input_pose, double distance);

    geometry_msgs::msg::Pose robotToMapPose(geometry_msgs::msg::Pose robot_frame_pose);


};

#endif //CYLINDER_NAVIGATION_NODE_HPP