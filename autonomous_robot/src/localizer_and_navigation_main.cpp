#include "rclcpp/rclcpp.hpp"
#include "localizer_and_navigation.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Check for valid argument count
    if (argc < 4) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: localizer_and_navigation <x> <y> <yaw>");
        return -1;
    }

    // Parse the x, y, and yaw from command-line arguments
    float x = std::stof(argv[1]);
    float y = std::stof(argv[2]);
    float yaw = std::stof(argv[3]);

    auto node = std::make_shared<LocalizerAndNavigation>(x, y, yaw);

    rclcpp::shutdown();
    return 0;
}
