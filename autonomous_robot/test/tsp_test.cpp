#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "../src/tsp.hpp"

TEST(tsp, TestTSPSearch)
{
    // Start Position
    geometry_msgs::msg::Pose start;
    start.position.set__x(0.0);
    start.position.set__y(0.0);

    std::shared_ptr<tsp> tsp_solver = std::make_shared<tsp>(start);

    // Create PoseArray for goals
    geometry_msgs::msg::PoseArray goals;

    // Goal 1
    geometry_msgs::msg::Pose goal_1;
    goal_1.position.set__x(-0.9703071146877011);
    goal_1.position.set__y(12.334043699471234);
    goals.poses.push_back(goal_1);

    // Goal 2
    geometry_msgs::msg::Pose goal_2;
    goal_2.position.set__x(0.1438945276075762);
    goal_2.position.set__y(12.390100674478632);
    goals.poses.push_back(goal_2);

    // Goal 3
    geometry_msgs::msg::Pose goal_3;
    goal_3.position.set__x(2.5149358303842186);
    goal_3.position.set__y(12.340246738052114);
    goals.poses.push_back(goal_3);

    // Goal 4
    geometry_msgs::msg::Pose goal_4;
    goal_4.position.set__x(3.4607120709319745);
    goal_4.position.set__y(12.308612527434398);
    goals.poses.push_back(goal_4);

    // Goal 5
    geometry_msgs::msg::Pose goal_5;
    goal_5.position.set__x(5.494962481835915);
    goal_5.position.set__y(12.285629140943204);
    goals.poses.push_back(goal_5);

    // Goal 6
    geometry_msgs::msg::Pose goal_6;
    goal_6.position.set__x(6.492166561601688);
    goal_6.position.set__y(12.242804767343367);
    goals.poses.push_back(goal_6);

    // Goal 7
    geometry_msgs::msg::Pose goal_7;
    goal_7.position.set__x(8.575562344742487);
    goal_7.position.set__y(12.208734196456609);
    goals.poses.push_back(goal_7);

    // Goal 8
    geometry_msgs::msg::Pose goal_8;
    goal_8.position.set__x(9.611783738023814);
    goal_8.position.set__y(11.949035756908456);
    goals.poses.push_back(goal_8);

    // Goal 9
    geometry_msgs::msg::Pose goal_9;
    goal_9.position.set__x(11.449972616012856);
    goal_9.position.set__y(11.750152085069661);
    goals.poses.push_back(goal_9);

    // Goal 10
    geometry_msgs::msg::Pose goal_10;
    goal_10.position.set__x(12.639590379185575);
    goal_10.position.set__y(11.702032383037356);
    goals.poses.push_back(goal_10);

    // Goal 11
    geometry_msgs::msg::Pose goal_11;
    goal_11.position.set__x(14.650339893261064);
    goal_11.position.set__y(11.671225509286678);
    goals.poses.push_back(goal_11);

    // Goal 12
    geometry_msgs::msg::Pose goal_12;
    goal_12.position.set__x(15.521947500068439);
    goal_12.position.set__y(11.683563222783144);
    goals.poses.push_back(goal_12);

    // Goal 13
    geometry_msgs::msg::Pose goal_13;
    goal_13.position.set__x(12.34516667975596);
    goal_13.position.set__y(0.7770220396276165);
    goals.poses.push_back(goal_13);

    // Goal 14
    geometry_msgs::msg::Pose goal_14;
    goal_14.position.set__x(11.485715854933082);
    goal_14.position.set__y(0.6882745284326579);
    goals.poses.push_back(goal_14);

    // Goal 15
    geometry_msgs::msg::Pose goal_15;
    goal_15.position.set__x(8.322238460974097);
    goal_15.position.set__y(0.3739701007978499);
    goals.poses.push_back(goal_15);

    // Goal 16
    geometry_msgs::msg::Pose goal_16;
    goal_16.position.set__x(7.305051934059303);
    goal_16.position.set__y(0.29117914118343247);
    goals.poses.push_back(goal_16);

    // Goal 17
    geometry_msgs::msg::Pose goal_17;
    goal_17.position.set__x(4.308159565209022);
    goal_17.position.set__y(0.5175427507865094);
    goals.poses.push_back(goal_17);

    // Goal 18
    geometry_msgs::msg::Pose goal_18;
    goal_18.position.set__x(3.3925333911795397);
    goal_18.position.set__y(0.6300366270934497);
    goals.poses.push_back(goal_18);


    std::vector<unsigned int> calculated_path = tsp_solver->optimizePath(goals);

    std::cout << "Calculated Path: ";

    for(size_t i = 0; i < calculated_path.size(); i++)
    {
        std::cout << calculated_path.at(i) << ", ";
    }

    // Ground Truth

    std::vector<unsigned int> ground_truth_path = {17, 16, 15, 14, 13, 12, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 10, 11};

    for(size_t i = 0; i < calculated_path.size(); i++)
    {
        ASSERT_TRUE(calculated_path.at(i) == ground_truth_path.at(i));
    }

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
