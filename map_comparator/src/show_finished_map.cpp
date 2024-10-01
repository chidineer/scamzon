#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <string>

class ShowFinishedMapNode : public rclcpp::Node
{
public:
    ShowFinishedMapNode() : Node("show_finished_map_node")
    {
        // Load ground truth map
        ground_truth_map_ = cv::imread("/home/student/ros2_ws/src/autonomous_robot/map/ground_truth.pgm", cv::IMREAD_GRAYSCALE);
        if (ground_truth_map_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load ground truth map.");
            rclcpp::shutdown();
        }

        // Load final map
        final_map_ = cv::imread("/home/student/ros2_ws/src/autonomous_robot/map/map.pgm", cv::IMREAD_GRAYSCALE);
        if (final_map_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load final map.");
            rclcpp::shutdown();
        }

        // Create windows for displaying images
        cv::namedWindow("Ground Truth Map", cv::WINDOW_NORMAL);
        cv::namedWindow("Final Map", cv::WINDOW_NORMAL);

        // Show both maps
        cv::imshow("Ground Truth Map", ground_truth_map_);
        cv::imshow("Final Map", final_map_);
        
        // Call waitKey to keep the window open until a key is pressed
        cv::waitKey(0); // Wait indefinitely for a keypress

    }

private:
    cv::Mat ground_truth_map_;
    cv::Mat final_map_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ShowFinishedMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
