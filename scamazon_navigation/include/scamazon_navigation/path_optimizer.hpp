#ifndef SCAMAZON_NAVIGATION_PATH_OPTIMIZER_HPP
#define SCAMAZON_NAVIGATION_PATH_OPTIMIZER_HPP

#include <vector>
#include "geometry_msgs/msg/pose.hpp"

class PathOptimizer
{
public:
    PathOptimizer() = default;

    // Function to compute the shortest path using nearest neighbor algorithm
    std::vector<geometry_msgs::msg::Pose> computeShortestPath(const std::vector<geometry_msgs::msg::Pose> &goals);

private:
    double euclideanDistance(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2);
};

#endif  // SCAMAZON_NAVIGATION_PATH_OPTIMIZER_HPP
