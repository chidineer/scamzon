#include "localizer_and_navigation.hpp"
#include <thread>
#include <chrono>

LocalizerAndNavigation::LocalizerAndNavigation(float x, float y, float yaw)
    : Node("localizer_and_navigation"), current_goal_index_(0), started_(true), current_state_(State::IDLE)
{
    // Launch Nav2 (run in background)
    RCLCPP_INFO(this->get_logger(), "Starting Nav2...");
    std::string nav2_cmd = "ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=/home/student/ros2_ws/src/autonomous_robot/map/real_warehouse.yaml &";
    int nav2_status = system(nav2_cmd.c_str());
    if (nav2_status != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start Nav2.");
        return;
    }

    // Wait for a few seconds for the systems to initialize
    std::this_thread::sleep_for(std::chrono::seconds(10));

    // Create publishers and subscribers
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/scam_goals", 10,
                                                                         std::bind(&LocalizerAndNavigation::goalCallback, this, std::placeholders::_1));
    amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10,
                                                                                        std::bind(&LocalizerAndNavigation::amclCallback, this, std::placeholders::_1));
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    // Subscribe to the image_raw topic
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&LocalizerAndNavigation::imageCallback, this, std::placeholders::_1));

    // Declare parameters for x, y, and yaw
    this->declare_parameter<float>("x", x);
    this->declare_parameter<float>("y", y);
    this->declare_parameter<float>("yaw", yaw);

    // Publish initial pose
    publishInitialPose(x, y, yaw);

    std::this_thread::sleep_for(std::chrono::seconds(10));

    colourDetectorPtr_ = std::make_shared<ColourDetector>();
    resetStocktake();
    stateMachine();
}

void LocalizerAndNavigation::resetStocktake(){

    count_.blue = 0;
    count_.green = 0;
    count_.yellow = 0;
    count_.red = 0;
    count_.purple = 0;

}

void LocalizerAndNavigation::stateMachine(){

    rclcpp::Rate rate(2);  // 2 Hz loop rate
    while (rclcpp::ok())
    {
        switch (getCurrentState())  // Use the getter to access current state
        {
        case LocalizerAndNavigation::State::IDLE:
            runIdleState();
            break;
        case LocalizerAndNavigation::State::RUNNING:
            runRunningState();
            break;
        case LocalizerAndNavigation::State::TASKED:
            runTaskedState();
            break;
        }
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
    }

}

void LocalizerAndNavigation::publishInitialPose(float x, float y, float yaw)
{
    auto initial_pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    initial_pose_msg.header.stamp = this->now();
    initial_pose_msg.header.frame_id = "map";
    initial_pose_msg.pose.pose.position.x = x;
    initial_pose_msg.pose.pose.position.y = y;
    initial_pose_msg.pose.pose.orientation = createQuaternionFromYaw(yaw);

    initial_pose_pub_->publish(initial_pose_msg);
    RCLCPP_INFO(this->get_logger(), "Published initial pose (x: %f, y: %f, yaw: %f)", x, y, yaw);
}

geometry_msgs::msg::Quaternion LocalizerAndNavigation::createQuaternionFromYaw(float yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.w = cos(yaw * 0.5);
    q.x = 0.0;
    q.y = 0.0;
    q.z = sin(yaw * 0.5);
    return q;
}

void LocalizerAndNavigation::goalCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    if (!msg->poses.empty())
    {
        std::unique_lock<std::mutex> lock(mtx_);
        geometry_msgs::msg::Pose start = current_pose_;
        lock.unlock();
        tsp_solver_ = std::make_shared<tsp>(start);
        std::vector<unsigned int> path = tsp_solver_->optimizePath(*msg);
        goals_ = tsp_solver_->mapGoalsToOptimizedOrder(*msg, path);
        total_goals_ = goals_.size();
        RCLCPP_INFO(this->get_logger(), "Received %lu goals", goals_.size());
        current_goal_index_ = 0;
        current_state_ = State::RUNNING;  // Switch to RUNNING state
    }
}

void LocalizerAndNavigation::amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    current_pose_ = msg->pose.pose;
}

void LocalizerAndNavigation::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    image_ = msg;

    // Convert the ROS image message to an OpenCV Mat in BGR format
    cv::Mat img(cv::Size(msg->width, msg->height), CV_8UC3, const_cast<unsigned char*>(msg->data.data()), cv::Mat::AUTO_STEP);

    // Now call the color detection function
    ColourDetector::ProductColour detected_color = colourDetectorPtr_->detectBox(msg);

    // Optionally, print the detected color to console
    std::string color_name;
    switch (detected_color) {
        case ColourDetector::RED: color_name = "Red"; break;
        case ColourDetector::YELLOW: color_name = "Yellow"; break;
        case ColourDetector::BLUE: color_name = "Blue"; break;
        case ColourDetector::GREEN: color_name = "Green"; break;
        case ColourDetector::PURPLE: color_name = "Purple"; break;
        default: color_name = "None"; break;
    }

    RCLCPP_INFO(this->get_logger(), "Detected color: %s", color_name.c_str());

    // Display the image with bounding box and label (already drawn inside the detectBox function)
    cv::imshow("Camera Feed", img);  // img is still in BGR format
    cv::waitKey(1);  // Required for OpenCV to update the image window
}

bool LocalizerAndNavigation::isGoalReached(const geometry_msgs::msg::Pose &pose)
{
    // Tolerances
    double position_tolerance = 0.5;  // Position tolerance (meters)
    double orientation_tolerance = 0.5;  // Orientation tolerance (radians)

    // Current goal
    const auto &goal = goals_.front();

    // Position check
    double dx = pose.position.x - goal.position.x;
    double dy = pose.position.y - goal.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Orientation check: Convert quaternion to yaw for both current pose and goal pose
    double current_yaw = std::atan2(2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
                                    1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z));

    double goal_yaw = std::atan2(2.0 * (goal.orientation.w * goal.orientation.z + goal.orientation.x * goal.orientation.y),
                                 1.0 - 2.0 * (goal.orientation.y * goal.orientation.y + goal.orientation.z * goal.orientation.z));

    // Calculate the yaw difference and normalize it to [-pi, pi]
    double yaw_diff = goal_yaw - current_yaw;
    if (yaw_diff > M_PI) {
        yaw_diff -= 2 * M_PI;
    } else if (yaw_diff < -M_PI) {
        yaw_diff += 2 * M_PI;
    }

    // Check if the distance is within the position tolerance
    if (distance > position_tolerance) {
        return false;
    }

    // Check if the yaw difference is within the orientation tolerance
    if (std::abs(yaw_diff) > orientation_tolerance) {
        return false;
    }

    // If both position and orientation are within tolerance, the goal is reached
    return true;
}

void LocalizerAndNavigation::publishNextGoal()
{
    if (goals_.size() > 0)
    {
        auto goal_msg = geometry_msgs::msg::PoseStamped();
        goal_msg.header.stamp = this->now();
        goal_msg.header.frame_id = "map";  // Make sure to set the correct frame
        goal_msg.pose = goals_.front();

        goal_pub_->publish(goal_msg);
        RCLCPP_INFO(this->get_logger(), "Published goal to /goal_pose: (x: %f, y: %f)",
                    goal_msg.pose.position.x, goal_msg.pose.position.y);

        current_state_ = State::RUNNING;  // Set state to RUNNING
    }
}

void LocalizerAndNavigation::stocktakeReport() {
    // Hardcoded filename and path
    std::string filename = "/home/student/ros2_ws/src/autonomous_robot/stocktake/Stocktake_Report.txt";

    // Open the file in truncate mode to remove all contents
    std::ofstream outfile(filename, std::ios::trunc);

    // Check if the file is open
    if (!outfile.is_open()) {
        std::cerr << "Error: Could not open the file for writing." << std::endl;
        return;
    }

    // Write new content to the file (you can replace this with your desired content)
    std::string newContent = "Stocktake Report:\n\n";
    outfile << newContent;

    newContent = "Red Boxes = " + std::to_string(count_.red) + "\n";
    outfile << newContent;

    newContent = "Yellow Boxes = " + std::to_string(count_.yellow) + "\n";
    outfile << newContent;

    newContent = "Blue Boxes = " + std::to_string(count_.blue) + "\n";
    outfile << newContent;

    newContent = "Green Boxes = " + std::to_string(count_.green) + "\n";
    outfile << newContent;

    newContent = "Purple Boxes = " + std::to_string(count_.purple) + "\n";
    outfile << newContent;

    // Close the file after writing
    outfile.close();

}

// State functions
void LocalizerAndNavigation::runIdleState()
{
    RCLCPP_INFO(this->get_logger(), "State: IDLE - Waiting for goals");
}

void LocalizerAndNavigation::runRunningState()
{

    if (goals_.size() > 0)
    {
        if(started_)
        {
            publishNextGoal();
            RCLCPP_INFO(this->get_logger(), "State: RUNNING");
        }
        started_ = false;
        std::unique_lock<std::mutex> lock(mtx_);
        if (isGoalReached(current_pose_))
        {
            RCLCPP_INFO(this->get_logger(), "Goal reached! Goal # %d of %d.", current_goal_index_ + 1, total_goals_);
            current_state_ = State::TASKED;
        }
        lock.unlock();
    }
    else
    {
        stocktakeReport();
        resetStocktake();
        current_state_ = State::IDLE;  // No goals left
    }
    // RCLCPP_INFO(this->get_logger(), "Current state: %d", static_cast<int>(current_state_));
    
}

void LocalizerAndNavigation::runTaskedState()
{
    RCLCPP_INFO(this->get_logger(), "State: TASKED - Reached goal, performing tasks");
    
    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::unique_lock<std::mutex> lock(mtx_);
    int current_box = colourDetectorPtr_->detectBox(image_);
    lock.unlock();

    switch(current_box)
    {
        case ProductColour::RED:
            RCLCPP_INFO(this->get_logger(), "Red Box Detected");
            count_.red++;
            break;
        case ProductColour::YELLOW:
            RCLCPP_INFO(this->get_logger(), "Yellow Box Detected");
            count_.yellow++;
            break;
        case ProductColour::BLUE:
            RCLCPP_INFO(this->get_logger(), "Blue Box Detected");
            count_.blue++;
            break;
        case ProductColour::GREEN:
            RCLCPP_INFO(this->get_logger(), "Green Box Detected");
            count_.green++;
            break;
        case ProductColour::PURPLE:
            RCLCPP_INFO(this->get_logger(), "Purple Box Detected");
            count_.purple++;
            break;
    }

    // If there are no tasks left for this goal, move to the next goal
    current_goal_index_++;
    current_state_ = State::RUNNING;
    goals_.erase(goals_.begin());
    started_ = true;
}
