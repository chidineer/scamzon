// main.cpp

#include "rclcpp/rclcpp.hpp"
#include "reach_goals.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ReachGoalsNode>();

    rclcpp::Rate rate(2);  // 2 Hz loop rate
    while (rclcpp::ok())
    {
        switch (node->getCurrentState())  // Use the getter to access current state
        {
        case ReachGoalsNode::State::IDLE:
            node->runIdleState();
            break;
        case ReachGoalsNode::State::RUNNING:
            node->runRunningState();
            break;
        case ReachGoalsNode::State::TASKED:
            node->runTaskedState();
            break;
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
