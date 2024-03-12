// main.cpp

#include <iostream>
#include <memory>

#include "line_detection.h"
// #include "GUI/GUI.h"

// std::unique_ptr<GUI> gui;
std::unique_ptr<Line_detection> line_detection;

int main() {
    line_detection = std::make_unique<Line_detection>(5);
    line_detection->begin();
    std::cout << "DONE!" << std::endl;
    return 0;
}
