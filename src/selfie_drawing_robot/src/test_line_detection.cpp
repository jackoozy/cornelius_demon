#include <iostream>
#include <memory>
#include <ros/package.h>
#include <ros/ros.h>
#include <filesystem>
#include <chrono>  // For timestamp
#include <sstream> // For string stream
#include <opencv2/opencv.hpp>

#include "line_detection.h"

std::unique_ptr<Line_detection> line_detection;

std::string getTestImagePath()
{
    std::string package_path = ros::package::getPath("selfie_drawing_robot");
    std::string data_path = package_path + "/src/line_detect_data/faces/samples";
    std::string imagePath;

    int num_samples = std::distance(std::filesystem::directory_iterator(data_path), std::filesystem::directory_iterator{});

    int selected_face = 0;

    std::cout << "choose a value from 0 - " << (num_samples - 1) << " to select a face: ";
    std::cin >> selected_face;

    imagePath = data_path + "/" + std::to_string(selected_face) + ".jpg";

    return imagePath;
}

std::string capturePhoto()
{
    std::string package_path = ros::package::getPath("selfie_drawing_robot");
    std::string data_path = package_path + "/src/line_detect_data/faces/webcam/";
    std::string path;

    cv::VideoCapture cap(0); // Open the default video camera

    if (!cap.isOpened())
    { // Check if the camera opened successfully
        std::cerr << "Error opening the camera" << std::endl << std::flush;
        return "";
    }

    std::cout << "Press any key in the image window to capture..." << std::endl << std::flush;

    cv::Mat frame;
    cv::namedWindow("Capture", cv::WINDOW_AUTOSIZE); // Create a window
    while (true)
    {
        cap >> frame; // Get a new frame from the camera

        if (frame.empty())
        {
            std::cerr << "Failed to capture an image" << std::endl << std::flush;
            break;
        }

        cv::imshow("Capture", frame); // Show the frame in the created window

        int key = cv::waitKey(30); // Wait for a key press for 30 milliseconds
        if (key >= 0)
            break; // If a key is pressed, break out of the loop
    }

    if (!frame.empty())
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d%H%M%S");
        path = data_path + ss.str() + ".jpg";
        cv::imwrite(path, frame);
        std::cout << "Photo saved as " << path << std::endl << std::flush;
    }

    cap.release();           // Release the video camera
    cv::destroyAllWindows(); // Close all OpenCV windows

    return path;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_line_detection");

    if (argc != 2)
    {
        std::cerr << "Usage: rosrun selfie_drawing_robot test_line_detection <mode>" << std::endl;
        std::cerr << "<mode> can be 'image' or 'camera'" << std::endl;
        return -1;
    }

    std::string mode = argv[1];
    std::string imagePath;

    if (mode == "image")
    {
        imagePath = getTestImagePath();
    }
    else if (mode == "camera")
    {
        imagePath = capturePhoto();
    }
    else
    {
        std::cerr << "Invalid mode. Please use 'image' or 'camera'" << std::endl;
        return -1;
    }

    if (imagePath.empty())
    {
        std::cerr << "No image path obtained. Exiting." << std::endl;
        return -1;
    }

    line_detection = std::make_unique<Line_detection>();
    line_detection->begin(imagePath);

    std::cout << "line script finished!" << std::endl << std::flush;
    return 0;
}
