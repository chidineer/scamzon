#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <random>

class MoveRobotNode : public rclcpp::Node
{
public:
    MoveRobotNode() : Node("move_robot_node"), initialized_(false), destination_reached_(false)
    {
        // Initialize publishers and subscribers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_noisy_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_noisy", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MoveRobotNode::odomCallback, this, std::placeholders::_1));
        
        // Initialize parameters
        this->declare_parameter<double>("linear_speed", 0.1);
        this->declare_parameter<double>("angular_speed", 0.0);
        this->declare_parameter<double>("distance", 10.0);
        this->declare_parameter<bool>("forward", true);

        // Initialize dead reckoning values
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        flag_ = false;

        // Timer to control the motion
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MoveRobotNode::moveRobot, this));
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!initialized_)
        {
            // Initialize starting position and orientation
            initial_x_ = msg->pose.pose.position.x;
            initial_y_ = msg->pose.pose.position.y;
            tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, initial_theta_);

            last_x_ = initial_x_;
            last_y_ = initial_y_;
            theta_ = initial_theta_;

            initialized_ = true;

            RCLCPP_INFO(this->get_logger(), "Initial position set. x: %.2f, y: %.2f",
                        initial_x_, initial_y_);
        }
        if(flag_)
        {
            final_x_ = msg->pose.pose.position.x;
            final_y_ = msg->pose.pose.position.y;
            RCLCPP_INFO(this->get_logger(), "Final position at. x: %.2f, y: %.2f",
                        final_x_, final_y_);
            RCLCPP_INFO(this->get_logger(), "Noisy final position at. x: %.2f, y: %.2f",
            x_, y_);
            flag_ = false;
        }
    }

    void moveRobot()
    {
        if (!initialized_)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for initial odometry data...");
            return;
        }

        // Retrieve parameters
        double linear_speed = this->get_parameter("linear_speed").as_double();
        double angular_speed = this->get_parameter("angular_speed").as_double();
        double distance = this->get_parameter("distance").as_double();
        bool forward = this->get_parameter("forward").as_bool();

        // Calculate distance traveled
        double traveled_distance = sqrt(pow(x_ - initial_x_, 2) + pow(y_ - initial_y_, 2));

        // Check if the robot has reached the destination
        if (traveled_distance >= distance && !destination_reached_)
        {
            destination_reached_ = true;
            flag_ = true;
            RCLCPP_INFO(this->get_logger(), "Destination reached.");
        }

        // Continue publishing zero velocities once the destination is reached
        if (destination_reached_)
        {
            auto stop_msg = geometry_msgs::msg::Twist();
            cmd_vel_pub_->publish(stop_msg);
        }
        else
        {
            // Determine direction
            if (!forward)
            {
                linear_speed = -linear_speed;
            }

            // Update dead reckoning position with noise
            std::normal_distribution<double> dist(0.0, 0.01);
            double delta_x = linear_speed * cos(theta_);
            double delta_y = linear_speed * sin(theta_);
            x_ += delta_x + dist(generator_);
            y_ += delta_y + dist(generator_);
            theta_ += angular_speed;

            // Create a noisy odometry message and publish it
            auto noisy_odom_msg = nav_msgs::msg::Odometry();
            noisy_odom_msg.header.stamp = this->now();
            noisy_odom_msg.header.frame_id = "odom";
            noisy_odom_msg.child_frame_id = "base_link";
            noisy_odom_msg.pose.pose.position.x = x_;
            noisy_odom_msg.pose.pose.position.y = y_;
            tf2::Quaternion q;
            q.setRPY(0, 0, theta_);
            noisy_odom_msg.pose.pose.orientation = tf2::toMsg(q);

            odom_noisy_pub_->publish(noisy_odom_msg);

            // Create a Twist message for robot movement
            auto cmd_msg = geometry_msgs::msg::Twist();
            cmd_msg.linear.x = linear_speed;
            cmd_msg.angular.z = angular_speed;

            // Publish the command
            cmd_vel_pub_->publish(cmd_msg);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_noisy_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Dead reckoning variables
    double x_, y_, theta_;
    double initial_x_, initial_y_, initial_theta_;
    double last_x_, last_y_;
    bool initialized_;
    bool destination_reached_;
    bool flag_;
    double final_x_, final_y_;

    // Random number generator for Gaussian noise
    std::default_random_engine generator_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveRobotNode>());
    rclcpp::shutdown();
    return 0;
}
