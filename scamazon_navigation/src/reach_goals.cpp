// reach_goals.cpp

#include "reach_goals.hpp"
#include <cmath>

ReachGoalsNode::ReachGoalsNode() : Node("reach_goals"), current_state_(State::IDLE), current_goal_index_(0), total_goals_(0), started_(true), tsp_(false)
{
    // Declare a parameter to toggle TSP usage
    this->declare_parameter("tsp", false);
    tsp_ = this->get_parameter("tsp").as_bool();

    // Subscriber to /scam_goals
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/scam_goals", 10, std::bind(&ReachGoalsNode::goalCallback, this, std::placeholders::_1));

    // Subscriber to /amcl_pose
    amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10, std::bind(&ReachGoalsNode::amclCallback, this, std::placeholders::_1));

    // Publisher to /goal_pose
    nav2_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
}

void ReachGoalsNode::goalCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    if (!msg->poses.empty())
    {
        goals_ = msg->poses;
        total_goals_ = goals_.size();
        RCLCPP_INFO(this->get_logger(), "Received %lu goals", goals_.size());
        current_goal_index_ = 0;
        current_state_ = State::RUNNING;  // Switch to RUNNING state

        if(tsp_){
             // Initialize the PathOptimizer and compute the shortest path
            PathOptimizer optimizer;
            goals_ = optimizer.computeShortestPath(msg->poses);
            RCLCPP_INFO(this->get_logger(), "TSP optimization applied.");
        }
        
    }

}

void ReachGoalsNode::amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(mtx_);
    pose_ = msg->pose.pose;
}


bool ReachGoalsNode::isGoalReached(const geometry_msgs::msg::Pose &pose)
{
    // Tolerances
    double position_tolerance = 0.5;  // Position tolerance (meters)
    double orientation_tolerance = 1.0;  // Orientation tolerance (radians)

    // Current goal
    const auto &goal = goals_.front();

    // Position check
    double dx = pose.position.x - goal.position.x;
    double dy = pose.position.y - goal.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // // Print current pose quaternion for debugging
    // RCLCPP_INFO(this->get_logger(), "Current Pose Quaternion: x: %f, y: %f, z: %f, w: %f",
    //             pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    // // Print goal pose quaternion for debugging
    // RCLCPP_INFO(this->get_logger(), "Goal Pose Quaternion: x: %f, y: %f, z: %f, w: %f",
    //             goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w);

    // Orientation check: Convert quaternion to yaw for both current pose and goal pose
    double current_yaw = std::atan2(2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
                                    1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z));

    double goal_yaw = std::atan2(2.0 * (goal.orientation.w * goal.orientation.z + goal.orientation.x * goal.orientation.y),
                                 1.0 - 2.0 * (goal.orientation.y * goal.orientation.y + goal.orientation.z * goal.orientation.z));

    // Calculate the yaw difference and normalize it to [-pi, pi]
    double yaw_diff = goal_yaw - current_yaw;

    // Normalize to the range [-pi, pi]
    if (yaw_diff > M_PI) {
        yaw_diff -= 2 * M_PI;
    } else if (yaw_diff < -M_PI) {
        yaw_diff += 2 * M_PI;
    }

    // Always print yaw values for debugging
    // RCLCPP_INFO(this->get_logger(), "Distance to Goal = %f", distance);
    // RCLCPP_INFO(this->get_logger(), "Yaw Difference = %f", yaw_diff);

    // Check if the distance is within the position tolerance
    if (distance > position_tolerance) {
        return false;
    }

    // Check if the yaw difference is within the orientation tolerance
    if (std::abs(yaw_diff) > orientation_tolerance) {
        return false;
    }

    // If both position and orientation are within tolerance, the goal is reached
    return true;
}




void ReachGoalsNode::publishNextGoal()
{
    if (goals_.size() > 0)
    {
        auto goal_msg = geometry_msgs::msg::PoseStamped();
        goal_msg.header.stamp = this->now();
        goal_msg.header.frame_id = "map";  // Make sure to set the correct frame
        goal_msg.pose = goals_.front();

        nav2_goal_pub_->publish(goal_msg);
        RCLCPP_INFO(this->get_logger(), "Published goal to /goal_pose: (x: %f, y: %f)",
                    goal_msg.pose.position.x, goal_msg.pose.position.y);

        current_state_ = State::RUNNING;  // Set state to RUNNING
    }
}

// State functions
void ReachGoalsNode::runIdleState()
{
    RCLCPP_INFO(this->get_logger(), "State: IDLE - Waiting for goals");
}

void ReachGoalsNode::runRunningState()
{

    if (goals_.size() > 0)
    {
        if(started_)
        {
            publishNextGoal();
            RCLCPP_INFO(this->get_logger(), "State: RUNNING");
        }
        started_ = false;
        std::unique_lock<std::mutex> lock(mtx_);
        if (isGoalReached(pose_))
        {
            RCLCPP_INFO(this->get_logger(), "Goal reached! Goal # %d of %d.", current_goal_index_ + 1, total_goals_);
            current_state_ = State::TASKED;
        lock.unlock();
    }
    }
    else
    {
        current_state_ = State::IDLE;  // No goals left
    }
    // RCLCPP_INFO(this->get_logger(), "Current state: %d", static_cast<int>(current_state_));
    
}

void ReachGoalsNode::runTaskedState()
{
    RCLCPP_INFO(this->get_logger(), "State: TASKED - Reached goal, waiting for tasks");

    // If there are no tasks for this goal, move to the next goal
    current_goal_index_++;
    current_state_ = State::RUNNING;
    goals_.erase(goals_.begin());
    started_ = true;
}
