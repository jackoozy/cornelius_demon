#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "tinyxml2.h"
#include <ros/package.h>


struct Point
{
    float x, y;
};

struct ContourData
{
    std::vector<std::vector<Point>> contours;
    std::vector<float> strokeWidths;
    std::vector<std::vector<Point>> fillRegions;
    std::vector<std::string> fillColours;
};

ContourData svg_to_contours(const std::string& fileName_)
{
    std::string package_path = ros::package::getPath("selfie_drawing_robot");
    std::string data_path = package_path + "/src/line_detect_data";


    ContourData contourGroup_;
    tinyxml2::XMLDocument doc;
    
    std::string file_name = data_path + "/" + fileName_ + ".svg";
    if (doc.LoadFile(file_name.c_str()) != tinyxml2::XML_SUCCESS)
    {
        std::cerr << "Unable to open file: " << file_name << std::endl;
        return contourGroup_;
    }

    tinyxml2::XMLElement* svgElement = doc.FirstChildElement("svg");
    if (!svgElement)
    {
        std::cerr << "No <svg> element found in file: " << file_name << std::endl;
        return contourGroup_;
    }

    tinyxml2::XMLElement* pathElement = svgElement->FirstChildElement("path");
    while (pathElement)
    {
        const char* dAttr = pathElement->Attribute("d");
        if (!dAttr)
        {
            pathElement = pathElement->NextSiblingElement("path");
            continue;
        }

        std::string d = dAttr;
        std::stringstream ss(d);
        std::vector<Point> points;
        std::string command;
        float x, y;
        
        ss >> command; // Should be 'M'
        while (ss >> x >> y)
        {
            points.push_back({x, y});
        }

        if (std::string(pathElement->Attribute("fill")) == "none")
        {
            contourGroup_.contours.push_back(points);
            const char* strokeWidthAttr = pathElement->Attribute("stroke-width");
            contourGroup_.strokeWidths.push_back(strokeWidthAttr ? std::stof(strokeWidthAttr) : 1.0f);
        }
        else
        {
            contourGroup_.fillRegions.push_back(points);
            const char* fillColourAttr = pathElement->Attribute("fill");
            contourGroup_.fillColours.push_back(fillColourAttr ? fillColourAttr : "#000000");
        }

        pathElement = pathElement->NextSiblingElement("path");
    }

    return contourGroup_;
}

int main()
{
    std::string fileName = "example";
    ContourData data = svg_to_contours(fileName);
    
    // Debug output
    for (const auto& contour : data.contours)
    {
        for (const auto& point : contour)
        {
            std::cout << "Contour point: (" << point.x << ", " << point.y << ")\n";
        }
    }
    for (const auto& region : data.fillRegions)
    {
        for (const auto& point : region)
        {
            std::cout << "Fill region point: (" << point.x << ", " << point.y << ")\n";
        }
    }
    return 0;
}
