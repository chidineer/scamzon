#include "cylinder_navigation_node.hpp"

#include <mutex>
#include <cmath> 
#include <chrono>
#include <thread>
using namespace std::placeholders;

CylinderNavigationNode::CylinderNavigationNode() : Node("cylinder_navigation_node")
{
    //Adjustable variables
    distance_from_cylinder_ = 0.5; //adjust as necessary
    threshold_ = 0.3; //adjust as necessary (changes how close to waypoint it reaches)
    num_waypoints_ = 4; //adjust as necessary (goes back to first waypoint)

    // Subscribers
    cylinder_detected_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "/visualization_marker", 10,
        std::bind(&CylinderNavigationNode::cylinderDetectedCallback, this, std::placeholders::_1));

    amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        std::bind(&CylinderNavigationNode::amclPoseCallback, this, std::placeholders::_1));
    
    plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10, 
        std::bind(&CylinderNavigationNode::planCallback, this, std::placeholders::_1));

    // Publishers
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints", 10);

    // Action client for navigation
    client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    // Timer callback for running state machine
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&CylinderNavigationNode::timer_callback, this));


}

void CylinderNavigationNode::cylinderDetectedCallback(const visualization_msgs::msg::Marker::SharedPtr msg)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    // Only run once when cylinder is detected first time AND navigation has started(or this will continue happening as it navigates)
    if (first_detection_flag_ && (state_ == State::WAITING_FOR_CYLINDER) && amcl_updated_flag_)
    {
        cylinder_pose_ = robotToMapPose(msg->pose); //For pose that is in robot ref frame
        // cylinder_pose_ = msg->pose;
    
        RCLCPP_INFO(this->get_logger(), "cylinder_pose_: (x: %.2f, y: %.2f)",
                    cylinder_pose_.position.x, cylinder_pose_.position.y);
                
        state_ = State::STOP_NAVIGATION;
        RCLCPP_INFO(this->get_logger(), "STOP_NAVIGATION");
        first_detection_flag_ = false; //this stays false so navigation isn't triggered again after multiple detections
    }   

    
}

void CylinderNavigationNode::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    amcl_updated_flag_ = true;
    last_amcl_pose_ = msg->pose.pose; // Store the last AMCL pose
    //Debugging
    RCLCPP_INFO(this->get_logger(), "last_amcl_pose:  (x: %.2f, y: %.2f, yaw: %.2f)", 
                last_amcl_pose_.position.x, last_amcl_pose_.position.y, quatToYaw(last_amcl_pose_.orientation));
}

void CylinderNavigationNode::planCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    // std::lock_guard<std::mutex> lock(mtx_);
    plan_msg_ = msg; //to implement cylinder detection later

    std::vector<geometry_msgs::msg::PoseStamped> plan_poses = msg->poses;
    //Debugging
    // RCLCPP_INFO(this->get_logger(), "plan_msgs_ last pose:  (x: %.2f, y: %.2f)", 
    //             plan_poses.back().pose.position.x, plan_poses.back().pose.position.y);

    if (first_plan_flag_)
    {
        first_plan_flag_ = false; //won't be triggered again
        // save the first plan's final pose
        std::vector<geometry_msgs::msg::PoseStamped> first_plan_poses = msg->poses;
        original_goal_ = first_plan_poses.back().pose;

        //Debugging
        RCLCPP_INFO(this->get_logger(), "original_goal_:  (x: %.2f, y: %.2f)", 
                    original_goal_.position.x, original_goal_.position.y);

        //Start to navigate to goal and waits for a cylinder pose
        state_ = State::WAITING_FOR_CYLINDER;
        updateState();
    }
    
}

void CylinderNavigationNode::timer_callback() {
    updateState();
}

void CylinderNavigationNode::updateState() {
    RCLCPP_INFO(this->get_logger(), "Updating state!");

    switch (state_) {

        case State::WAITING_FOR_GOAL:
            RCLCPP_INFO(this->get_logger(), "WAITING_FOR_GOAL");
            // Remain in this state until a goal/plan is created
            break;

        case State::WAITING_FOR_CYLINDER:
            RCLCPP_INFO(this->get_logger(), "WAITING_FOR_CYLINDER");
            // Remain in this state until a new detection occurs
            break;

        case State::STOP_NAVIGATION:
            //Stop navigating to goal
            stopNavigation();

            // Save pose of the turtlebot when cylinder gets detected
            pose_when_detected_ = last_amcl_pose_;

            // Save cylinder pose
            // for now lets say the cylinder_pose_ is 1m in front of turtlebot this will be changed later
            // cylinder_pose_ = newPoseAtDistance(pose_when_detected_, 1.0);
            // cylinder_pose_ = geometry_msgs::msg::Pose();
            // cylinder_pose_.position.x = 5.99;
            // cylinder_pose_.position.y = 7.69;
            // cylinder_pose_.position.z = 0;
            // cylinder_pose_.orientation.x = 0;
            // cylinder_pose_.orientation.y = 0;
            // cylinder_pose_.orientation.z = 0;
            // cylinder_pose_.orientation.w = 1;

            // Create waypoints around cylinder
            state_ = State::CREATE_WAYPOINTS;
            RCLCPP_INFO(this->get_logger(), "CREATE_WAYPOINTS");
            break;

        case State::CREATE_WAYPOINTS:
            createWaypointsAroundCylinder();
            state_ = State::SEND_WAYPOINTS;
            RCLCPP_INFO(this->get_logger(), "SEND_WAYPOINTS");
            break;

        case State::SEND_WAYPOINTS:
            // Send waypoints to nav2
            sendWaypointsToNav2();

            // Monitor if waypoints are finished
            start_time_ = std::chrono::steady_clock::now();
            state_ = State::MONITOR_WAYPOINTS;
            RCLCPP_INFO(this->get_logger(), "MONITOR_WAYPOINTS");
            break;

        case State::MONITOR_WAYPOINTS:
            if (waypoints_finished_) {
                stopNavigation();
                //publish original goal to return to navigation
                publishOriginalGoal(original_goal_.position.x, original_goal_.position.y, quatToYaw(original_goal_.orientation));
                state_ = State::RETURN_TO_ORIGINAL_GOAL;
                RCLCPP_INFO(this->get_logger(), "RETURN_TO_ORIGINAL_GOAL");
            } else {
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time_);
                if (elapsed_time.count() >= 1000) {
                    checkWaypointsFinished();
                    start_time_ = current_time; // Reset the start time
                }
            }
            break;
        
        case State::RETURN_TO_ORIGINAL_GOAL:
            double distance_to_goal = linearDistance(original_goal_, last_amcl_pose_);
            if (distance_to_goal <= threshold_)
            {
                //Reset for new goal/objects
                first_detection_flag_ = true;
                first_plan_flag_ = false;
                waypoints_finished_ = false;
                //Go back to waiting for goal
                state_ = State::WAITING_FOR_GOAL;
                RCLCPP_INFO(this->get_logger(), "WAITING_FOR_GOAL");
            }
            
            break;
    }
}

void CylinderNavigationNode::stopNavigation(){

    auto goal_msg = geometry_msgs::msg::PoseStamped();
    goal_msg.header.stamp = this->now();
    goal_msg.header.frame_id = "map";  // Make sure to set the correct frame
    goal_msg.pose = newPoseAtDistance(last_amcl_pose_, 0.2); //a little ahead of current pose to make up for delay
    goal_pub_->publish(goal_msg);
        RCLCPP_INFO(this->get_logger(), "Stopping Navigation at: (x: %.2f, y: %.2f)",
                    goal_msg.pose.position.x, goal_msg.pose.position.y);

    //Wait for turtlebot to stop or waypoints won't be published
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

void CylinderNavigationNode::publishOriginalGoal(float x, float y, float yaw)
{
    original_goal_.position.x = x;
    original_goal_.position.y = y;
    original_goal_.orientation = createQuaternionFromYaw(yaw);
    auto original_pose_msg = geometry_msgs::msg::PoseStamped();
    original_pose_msg.header.stamp = this->now();
    original_pose_msg.header.frame_id = "map";
    original_pose_msg.pose = original_goal_;

    goal_pub_->publish(original_pose_msg);
        RCLCPP_INFO(this->get_logger(), "Published goal to /goal_pose: (x: %.2f, y: %.2f)",
                    original_pose_msg.pose.position.x, original_pose_msg.pose.position.y);

}

void CylinderNavigationNode::createWaypointsAroundCylinder()
{
    waypoints_.clear();

    //Debugging
    RCLCPP_INFO(this->get_logger(), "Waypoints created at:");

    // Create waypoints starting from closest point to robot
    // double robot_yaw = quatToYaw(pose_when_detected_.orientation); 
    double x_diff = pose_when_detected_.position.x - cylinder_pose_.position.x;
    double y_diff = pose_when_detected_.position.y - cylinder_pose_.position.y;
    double angle_to_cylinder = atan(y_diff/x_diff);

    if (x_diff < 0)
    {
        angle_to_cylinder += M_PI;
    }
    
    
    // RCLCPP_INFO(this->get_logger(), "x_diff: %.2f, y_diff: %.2f, angle_to_cylinder: %.2f", x_diff, y_diff, angle_to_cylinder);
    

    for (int i = 0; i < num_waypoints_; ++i) // Create waypoints
    {
        double angle;
        if (i == (num_waypoints_-1))
        {
            angle = angle_to_cylinder; // last one in same spot as first
        }
        else
        {
            angle = (i * ((2*M_PI)/(num_waypoints_-1))) + angle_to_cylinder; // degrees apart
        }
        
        geometry_msgs::msg::PoseStamped waypoint;
        // Set the header
        waypoint.header.stamp = rclcpp::Clock().now(); // Set the current time
        waypoint.header.frame_id = "map";          // Set the frame ID
        // Assign the pose
        waypoint.pose.position.x = cylinder_pose_.position.x + distance_from_cylinder_ * cos(angle);
        waypoint.pose.position.y = cylinder_pose_.position.y + distance_from_cylinder_ * sin(angle);
        waypoint.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, sin(angle / 2), cos(angle / 2)));
        //Order waypoints into vector of PoseStamped
        waypoints_.push_back(waypoint); 

        //Debugging
        RCLCPP_INFO(this->get_logger(), "Waypoint %d: x: %.2f, y:, %.2f, angle: %.2f, cos: %.2f, sin: %.2f",
                        (i+1), waypoint.pose.position.x, waypoint.pose.position.y, angle, cos(angle), sin(angle));
    }
    
}

void CylinderNavigationNode::sendWaypointsToNav2()
{
    // Create a goal message
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();

    geometry_msgs::msg::PoseStamped goal_pose = waypoints_.at(waypoint_index_);
    //add to goal_msg
    goal_msg.pose = goal_pose;
    RCLCPP_INFO(this->get_logger(), "goal_msg Created! for waypoint %d", (waypoint_index_+1));

    // Ensure the action server is available before sending goals
    if (!client_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available!");
        return;
    }
    else
    {
        // Set up the goal response and feedback callbacks
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&CylinderNavigationNode::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&CylinderNavigationNode::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&CylinderNavigationNode::result_callback, this, _1);

        // Send the goal
        client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Goal Sent! for waypoint %d", (waypoint_index_+1));
    }
    
}

void CylinderNavigationNode::goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle)
{
    if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void CylinderNavigationNode::feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,  
                        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
    if (feedback) {
        feedback_ = feedback;
        // RCLCPP_INFO(this->get_logger(), "Feedback received: %.2f distance remaining", feedback->distance_remaining);
    } else {
        RCLCPP_WARN(this->get_logger(), "Received null feedback!");
    }
}

void CylinderNavigationNode::result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
{
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
}


void CylinderNavigationNode::checkWaypointsFinished()
{
    if (!waypoints_.empty())
    {  
        geometry_msgs::msg::PoseStamped last_pose = waypoints_.back();
        double distance = linearDistance(last_amcl_pose_, last_pose.pose);

        //Debugging
        RCLCPP_INFO(this->get_logger(), "distance vs threshold: %.2f/%.2f",
                    distance, threshold_);
        // //Debugging
        RCLCPP_INFO(this->get_logger(), "distance remaining: %.2f",
                    feedback_->distance_remaining);


        //Debugging
        RCLCPP_INFO(this->get_logger(), "Current waypoint: %d/%ld", (waypoint_index_+1), waypoints_.size());

        //Condition to send another waypoint if previous is complete and more to do
        if ((feedback_->distance_remaining < threshold_) && (waypoint_index_+1 < waypoints_.size()))
        {                
            // stopNavigation();
            state_ = State::SEND_WAYPOINTS;
            waypoint_index_++;
            waypoints_finished_ = false;
        }
        else if ((feedback_->distance_remaining < threshold_) && (waypoint_index_+1 >= waypoints_.size())) // Check if its near the last waypoint and all waypoints complete
        {
            waypoints_finished_ = true;
            waypoint_index_ = 0; //reset
        }
        else
        {
            waypoints_finished_ = false;
        }
        
        
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot check if finished: Waypoints or amcl pose information missing!");
    }
    
    //Debugging
    RCLCPP_INFO(this->get_logger(), "waypoints_finished_: %s", waypoints_finished_ ? "true" : "false");
    
}

geometry_msgs::msg::Quaternion CylinderNavigationNode::createQuaternionFromYaw(float yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.w = cos(yaw * 0.5);
    q.x = 0.0;
    q.y = 0.0;
    q.z = sin(yaw * 0.5);
    return q;
}

double CylinderNavigationNode::quatToYaw(const geometry_msgs::msg::Quaternion& quaternion) {
    // Create a tf2 Quaternion from the input quaternion
    tf2::Quaternion quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w);

    // Convert to a rotation matrix
    tf2::Matrix3x3 mat(quat);

    // Extract Roll, Pitch, Yaw
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    // Return only the Yaw value
    return yaw;
}

double CylinderNavigationNode::linearDistance(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2)
{
    // Extract positions from the poses
    double x1 = pose1.position.x;
    double y1 = pose1.position.y;
    double x2 = pose2.position.x;
    double y2 = pose2.position.y;

    // Calculate the linear distance using the Euclidean distance formula
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

geometry_msgs::msg::Pose CylinderNavigationNode::newPoseAtDistance(const geometry_msgs::msg::Pose& input_pose, double distance) 
{
    geometry_msgs::msg::Pose new_pose = input_pose;

    // Extract the orientation as a quaternion
    double x = input_pose.position.x;
    double y = input_pose.position.y;

    // Convert quaternion to yaw (z-axis rotation)
    double roll, pitch, yaw;
    tf2::Quaternion quat(input_pose.orientation.x, input_pose.orientation.y,
                        input_pose.orientation.z, input_pose.orientation.w);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // Calculate the new position using the variable distance
    new_pose.position.x = x + distance * cos(yaw);
    new_pose.position.y = y + distance * sin(yaw);

    // The orientation remains the same
    new_pose.orientation = input_pose.orientation;

    return new_pose;
}

geometry_msgs::msg::Pose CylinderNavigationNode::robotToMapPose(geometry_msgs::msg::Pose robot_frame_pose){
    geometry_msgs::msg::Pose map_frame_pose = robot_frame_pose;
    // Extract the orientation from the robot frame pose
    // Bug: if last_amcl_pose_ has no time to update this will be wrong
    
    double robot_x = last_amcl_pose_.position.x;
    double robot_y = last_amcl_pose_.position.y;
    double robot_theta = quatToYaw(last_amcl_pose_.orientation); // Get the yaw from the quaternion
    RCLCPP_INFO(this->get_logger(), "robot_theta: %.2f", robot_theta);

    // Calculate the new position in the map frame
    map_frame_pose.position.x = robot_x + (robot_frame_pose.position.x * cos(robot_theta)) - (robot_frame_pose.position.y * sin(robot_theta));
    map_frame_pose.position.y = robot_y + (robot_frame_pose.position.x * sin(robot_theta)) + (robot_frame_pose.position.y * cos(robot_theta));

    return map_frame_pose;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    //Debugging
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Spinning Node!");

    rclcpp::spin(std::make_shared<CylinderNavigationNode>());
    rclcpp::shutdown();
    return 0;
}