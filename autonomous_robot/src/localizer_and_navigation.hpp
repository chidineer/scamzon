#ifndef LOCALIZER_AND_NAVIGATION_HPP
#define LOCALIZER_AND_NAVIGATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <mutex>
#include <vector>

class LocalizerAndNavigation : public rclcpp::Node
{
public:
    LocalizerAndNavigation(float x, float y, float yaw);

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
    void publishInitialPose(float x, float y, float yaw);
    void goalCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    bool isGoalReached(const geometry_msgs::msg::Pose &pose);
    void publishNextGoal();
    geometry_msgs::msg::Quaternion createQuaternionFromYaw(float yaw);

    void stateMachine();


    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

    std::vector<geometry_msgs::msg::Pose> goals_;
    int current_goal_index_;
    bool started_;

    State current_state_;

    geometry_msgs::msg::Pose current_pose_;
    int total_goals_;
    std::mutex mtx_;
};

#endif // LOCALIZER_AND_NAVIGATION_HPP
