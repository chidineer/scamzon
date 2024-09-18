// reach_goals.hpp

#ifndef REACH_GOALS_HPP_
#define REACH_GOALS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "scamazon_navigation/path_optimizer.hpp"
#include <vector>
#include <mutex>

using namespace std;

class ReachGoalsNode : public rclcpp::Node
{
public:
    ReachGoalsNode();

    enum class State
    {
        IDLE,
        RUNNING,
        TASKED
    };

    // Public methods for each state
    void runIdleState();
    void runRunningState();
    void runTaskedState();

    // Getter for current state
    State getCurrentState() const { return current_state_; }

private:
    // Subscriber and Publisher
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav2_goal_pub_;

    // Callbacks
    void goalCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // State management
    State current_state_;
    std::vector<geometry_msgs::msg::Pose> goals_;
    bool isGoalReached(const geometry_msgs::msg::Pose &pose);
    void publishNextGoal();

    int current_goal_index_;
    int total_goals_;
    geometry_msgs::msg::Pose pose_;
    bool started_;
    bool tsp_;
    mutex mtx_;
};

#endif  // REACH_GOALS_HPP_
