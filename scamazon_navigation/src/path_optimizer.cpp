#include "scamazon_navigation/path_optimizer.hpp"
#include <cmath>
#include <limits>

double PathOptimizer::euclideanDistance(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2)
{
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<geometry_msgs::msg::Pose> PathOptimizer::computeShortestPath(const std::vector<geometry_msgs::msg::Pose> &goals)
{
    if (goals.empty()) return {};

    std::vector<geometry_msgs::msg::Pose> ordered_goals;
    std::vector<bool> visited(goals.size(), false);

    // Start with the first goal
    ordered_goals.push_back(goals[0]);
    visited[0] = true;

    // Keep track of the last visited goal
    geometry_msgs::msg::Pose last_visited = goals[0];

    // Nearest neighbor algorithm
    for (size_t i = 1; i < goals.size(); ++i)
    {
        double min_distance = std::numeric_limits<double>::max();
        size_t next_goal_index = 0;

        for (size_t j = 0; j < goals.size(); ++j)
        {
            if (!visited[j])
            {
                double distance = euclideanDistance(last_visited, goals[j]);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    next_goal_index = j;
                }
            }
        }

        // Mark the next goal as visited
        visited[next_goal_index] = true;
        ordered_goals.push_back(goals[next_goal_index]);
        last_visited = goals[next_goal_index];
    }

    return ordered_goals;
}
