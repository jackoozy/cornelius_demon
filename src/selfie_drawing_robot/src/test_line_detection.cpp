// Test plan for my module:

// - dynamically adjust parameters for a desired level of detail
// - supply multiple faces for some test cases
// - supply images of differnt lighting to see adaptive contrasting and brightness
// - test aginst images of people with different facial features (eg; hair type, beard, skin tone, etc)
// - how many lines are produced, how smooth are they?
// - are the shaded regions accurate?

// main.cpp

#include <iostream>
#include <memory>
#include <ros/package.h>
#include <filesystem>
#include <chrono> // For timestamp
#include <sstream> // For string stream
#include <opencv2/opencv.hpp>


#include "line_detection.h"
// #include "GUI/GUI.h"

// std::unique_ptr<GUI> gui;
std::unique_ptr<Line_detection> line_detection;

std::string getTestImagePath()
{
    std::string package_path = ros::package::getPath("selfie_drawing_robot");
    std::string data_path = package_path + "/src/line_detect_data";
    std::string imagePath;

    int num_samples = std::distance(std::filesystem::directory_iterator(data_path + "/faces"), std::filesystem::directory_iterator{});

    int selected_face = 0;

    std::cout << "choose a value from 0 - " << (num_samples - 1) << " to select a face: ";
    std::cin >> selected_face;

    switch (selected_face)
    {
    case 0:
        imagePath = data_path + "/faces/elon_portrait.jpg";
        break;
    case 1:
        imagePath = data_path + "/faces/huberman.jpg";
        break;
    case 2:
        imagePath = data_path + "/faces/lex.jpg";
        break;
    case 3:
        imagePath = data_path + "/faces/reeves.jpg";
        break;
    case 4:
        imagePath = data_path + "/faces/zuck_HD.webp";
        break;
    case 5:
        imagePath = data_path + "/faces/group.webp";
        break;
    default:
        std::cout << "Invalid input" << std::endl;
        break;
    }

    return imagePath;
}


std::string capturePhoto() {
    std::string package_path = ros::package::getPath("selfie_drawing_robot");
    std::string data_path = package_path + "/src/line_detect_data/faces/webcam/";
    std::string path;

    cv::VideoCapture cap(0);  // Open the default video camera

    if (!cap.isOpened()) {  // Check if the camera opened successfully
        std::cerr << "Error opening the camera" << std::endl;
        return "";
    }

    std::cout << "Press any key to capture the photo..." << std::endl;

    cv::Mat frame;
    cv::namedWindow("Capture", cv::WINDOW_AUTOSIZE); // Create a window
    while (true) {
        cap >> frame; // Get a new frame from the camera

        if (frame.empty()) {
            std::cerr << "Failed to capture an image" << std::endl;
            break;
        }

        cv::imshow("Capture", frame); // Show the frame in the created window

        int key = cv::waitKey(30); // Wait for a key press for 30 milliseconds
        if (key >= 0) break; // If a key is pressed, break out of the loop
    }

    if (!frame.empty()) {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d%H%M%S");
        path = data_path + ss.str() + ".jpg";
        cv::imwrite(path, frame);
        std::cout << "Photo saved as " << path << std::endl;
    }

    cap.release(); // Release the video camera
    cv::destroyAllWindows(); // Close all OpenCV windows

    return path;
}


int main()
{
    // std::string imagePath = getTestImagePath();
    std::string imagePath = capturePhoto();
    line_detection = std::make_unique<Line_detection>();
    line_detection->begin(imagePath);
    std::cout << "test script finished!" << std::endl;
    return 0;
}
