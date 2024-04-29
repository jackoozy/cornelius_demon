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


#include "line_detection.h"
// #include "GUI/GUI.h"

// std::unique_ptr<GUI> gui;
std::unique_ptr<Line_detection> line_detection;

std::string getImagePath()
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

int main()
{
    std::string imagePath = getImagePath();
    line_detection = std::make_unique<Line_detection>();
    line_detection->begin(imagePath);
    std::cout << "test script finished!" << std::endl;
    return 0;
}
