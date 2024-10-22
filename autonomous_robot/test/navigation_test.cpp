#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


TEST(LocalizerAndNavigation, TestLocalization)
{
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("autonomous_robot");
    std::string bag_filename = package_share_directory + "/ros_bags/navigation";

    rosbag2_cpp::Reader reader1;
    rclcpp::Serialization<geometry_msgs::msg::PoseWithCovarianceStamped> serialization_amcl_pose;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr amcl_pose_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    reader1.open(bag_filename);

    while (reader1.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader1.read_next();
        if (msg->topic_name != "/amcl_pose") {
            continue;
        }
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        serialization_amcl_pose.deserialize_message(&serialized_msg, amcl_pose_msg.get());
        break;
    }

    geometry_msgs::msg::Pose bag_amcl_pose = amcl_pose_msg->pose.pose;

    // Goal Position
    double goal_x = -0.7895557742337819;
    double goal_y = 11.605891467846984;
    double goal_z = 0.0;

    // Goal Orientation
    double goal_qx = 0.0;
    double goal_qy = 0.0;
    double goal_qz = 0.8157913526932744;
    double goal_qw = 0.5783463226051304;

    double tolerance = 0.5;

    ASSERT_NEAR(bag_amcl_pose.position.x, goal_x, tolerance);
    ASSERT_NEAR(bag_amcl_pose.position.y, goal_y, tolerance);
    ASSERT_NEAR(bag_amcl_pose.position.z, goal_z, tolerance);

    ASSERT_NEAR(bag_amcl_pose.orientation.x, goal_qx, tolerance);
    ASSERT_NEAR(bag_amcl_pose.orientation.y, goal_qy, tolerance);
    ASSERT_NEAR(bag_amcl_pose.orientation.z, goal_qz, tolerance);
    ASSERT_NEAR(bag_amcl_pose.orientation.w, goal_qw, tolerance);

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
