#include "colour_detector.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

ColourDetector::ColourDetector() {
}

ColourDetector::ProductColour ColourDetector::detectBox(const sensor_msgs::msg::Image::SharedPtr msg) {

    // Convert the ROS image message to an OpenCV image
    cv_bridge::CvImagePtr cv_image_ptr;
    try
    {
        cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        return ColourDetector::NOTHING;  // Return a default value or handle this case appropriately
    }

    // Convert the image to HSV color space
    cv::Mat hsv_image;
    cv::cvtColor(cv_image_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    // Define color ranges
    cv::Scalar red_lower(0, 100, 0), red_upper(10, 255, 255);
    cv::Scalar blue_lower(100, 150, 0), blue_upper(140, 255, 255);
    cv::Scalar yellow_lower(20, 100, 100), yellow_upper(30, 255, 255);
    cv::Scalar green_lower(40, 40, 40), green_upper(70, 255, 255);
    cv::Scalar purple_lower(130, 50, 50), purple_upper(160, 255, 255);

    // Create masks for each color
    cv::Mat red_mask, blue_mask, yellow_mask, green_mask, purple_mask;
    cv::inRange(hsv_image, red_lower, red_upper, red_mask);
    cv::inRange(hsv_image, blue_lower, blue_upper, blue_mask);
    cv::inRange(hsv_image, yellow_lower, yellow_upper, yellow_mask);
    cv::inRange(hsv_image, green_lower, green_upper, green_mask);
    cv::inRange(hsv_image, purple_lower, purple_upper, purple_mask);

    // Calculate the area for each color mask
    int red_area = cv::countNonZero(red_mask);
    int blue_area = cv::countNonZero(blue_mask);
    int yellow_area = cv::countNonZero(yellow_mask);
    int green_area = cv::countNonZero(green_mask);
    int purple_area = cv::countNonZero(purple_mask);

    // Create an array of areas for easier comparison
    int color_areas[] = { red_area, yellow_area, blue_area, green_area, purple_area };

    // Find the index of the largest area
    int max_index = std::distance(color_areas, std::max_element(color_areas, color_areas + 6));

    // Select the corresponding mask and label
    cv::Mat *selected_mask = nullptr;
    std::string detected_color;
    switch (max_index) {
        case 0: selected_mask = &red_mask; detected_color = "Red"; break;
        case 1: selected_mask = &yellow_mask; detected_color = "Yellow"; break;
        case 2: selected_mask = &blue_mask; detected_color = "Blue"; break;
        case 3: selected_mask = &green_mask; detected_color = "Green"; break;
        case 5: selected_mask = &purple_mask; detected_color = "Purple"; break;
        default: return ColourDetector::NOTHING;  // Default in case of an error
    }

    // Find contours on the selected mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(*selected_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
        // Find the largest contour
        auto largest_contour = std::max_element(contours.begin(), contours.end(),
            [](const std::vector<cv::Point> &c1, const std::vector<cv::Point> &c2) {
                return cv::contourArea(c1) < cv::contourArea(c2);
            });

        // Draw the bounding box around the largest contour
        cv::Rect bounding_box = cv::boundingRect(*largest_contour);
        cv::rectangle(cv_image_ptr->image, bounding_box, cv::Scalar(0, 255, 0), 2);  // Green bounding box
        cv::putText(cv_image_ptr->image, detected_color, cv::Point(bounding_box.x, bounding_box.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);  // Label the box with color name
    }

    // Return the detected color enum
    switch (max_index) {
        case 0: return ColourDetector::RED;
        case 1: return ColourDetector::YELLOW;
        case 2: return ColourDetector::BLUE;
        case 3: return ColourDetector::GREEN;
        case 5: return ColourDetector::PURPLE;
        default: return ColourDetector::NOTHING;
    }
}
