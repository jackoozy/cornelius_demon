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

#include "line_detection.h"
// #include "GUI/GUI.h"

// std::unique_ptr<GUI> gui;
std::unique_ptr<Line_detection> line_detection;

int main() {
    line_detection = std::make_unique<Line_detection>();
    line_detection->begin();
    std::cout << "DONE!" << std::endl;
    return 0;
}
