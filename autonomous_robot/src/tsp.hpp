#ifndef TSP_HPP
#define TSP_HPP

#include <vector>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

class tsp {
public:
    tsp(geometry_msgs::msg::Pose start);
    
    // Returns a vector of indices representing the optimized order
    std::vector<unsigned int> optimizePath(geometry_msgs::msg::PoseArray &poses);

    // Returns a vector of poses in the optimized order
    std::vector<geometry_msgs::msg::Pose> mapGoalsToOptimizedOrder(const geometry_msgs::msg::PoseArray &poses, const std::vector<unsigned int> &optimized_indices);

private:
    double calculateDistance(const geometry_msgs::msg::Pose &a, const geometry_msgs::msg::Pose &b);

    geometry_msgs::msg::Pose start_;
};

#endif // TSP_HPP
