#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class InitialPosePublisher : public rclcpp::Node
{
public:
    InitialPosePublisher()
        : Node("initial_pose_publisher"), count_(0)
    {
        // Create a publisher for the 2D pose estimate
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

        // Define the timer to publish at 1 Hz
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&InitialPosePublisher::publish_initial_pose, this));
    }

private:
    void publish_initial_pose()
    {
        if (count_ >= 5)
        {
            RCLCPP_INFO(this->get_logger(), "Published initial pose 5 times, stopping.");
            rclcpp::shutdown();  // Shutdown after publishing 5 times
            return;
        }

        // Define the initial pose
        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg;
        initial_pose_msg.header.stamp = this->now();
        initial_pose_msg.header.frame_id = "map";  // Frame ID should be "map" for localization

        // Set the known position
        initial_pose_msg.pose.pose.position.x = 0.0;
        initial_pose_msg.pose.pose.position.y = 0.0;
        initial_pose_msg.pose.pose.position.z = 0.0;

        // Set the known orientation (quaternion)
        initial_pose_msg.pose.pose.orientation.x = 0.0;
        initial_pose_msg.pose.pose.orientation.y = 0.0;
        initial_pose_msg.pose.pose.orientation.z = 0.0;
        initial_pose_msg.pose.pose.orientation.w = 1.0;

        // Set the covariance (optional, usually zero for initialization)
        for (size_t i = 0; i < 36; i++) {
            initial_pose_msg.pose.covariance[i] = 0.0;
        }

        // Publish the message
        publisher_->publish(initial_pose_msg);

        RCLCPP_INFO(this->get_logger(), "Published initial pose (x: %f, y: %f)", 
                    initial_pose_msg.pose.pose.position.x, initial_pose_msg.pose.pose.position.y);

        count_++;
    }

    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
