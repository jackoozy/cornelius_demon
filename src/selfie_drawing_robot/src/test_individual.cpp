// main.cpp

#include <iostream>
#include <memory>

#include "line_detection/line_detection.h"
#include "motion_planner/motion_planner.h"
#include "stroke_planner/stroke_planner.h"
#include "end_effector/end_effector.h"
#include "GUI/GUI.h"

std::unique_ptr<GUI> gui;
std::unique_ptr<Line_detection> line_detection;
std::unique_ptr<Motion_planner> motion_planner;
std::unique_ptr<Stroke_planner> stroke_planner;
std::unique_ptr<End_effector> end_effector;

int main() {
    // get user input
    std::cout << "Select a module to run <GUI(1), line_detection(2), motion_planner(3), stroke_planner(4), end_effector(5)> ";
    int x;
    std::cin >> x;

    switch(x){
        case 1:
            gui = std::make_unique<GUI>(5);
            break;
        case 2:
            line_detection = std::make_unique<Line_detection>(5);
            line_detection->begin();
            break;
        case 3:
            motion_planner = std::make_unique<Motion_planner>(5);
            break;
        case 4:
            stroke_planner = std::make_unique<Stroke_planner>(5);
            break;
        case 5:
            end_effector = std::make_unique<End_effector>(5);
            break;
    }

    // print
    std::cout << "DONE!" << std::endl;
    return 0;
}
