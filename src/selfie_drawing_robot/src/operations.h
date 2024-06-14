#ifndef OPERATIONS_H
#define OPERATIONS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/scoped_ptr.hpp>
#include <iostream>
#include <thread>
#include <iomanip>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include "tinyxml2.h"
#include <ros/package.h>
#include <GLFW/glfw3.h>
#include <GL/glut.h>
#include <visualization_msgs/MarkerArray.h>


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

class Operations {
    public:

    Operations(){}

    ~Operations(){}

    // Computes TdrawSpace and drawSpaceHeight and drawSpaceWidth. 
    // Ensure that draw-space corners are set first.
    Eigen::Affine3d calculateDrawSpaceTransformation();

    // Takes in normalised draw-space point and returns 3D point.
    Eigen::Vector3d drawSpaceToWorld(const Point& pagePoint, const double& depth);

    void renderSpatialData();

    ContourData svg_to_contours(const std::string& fileName_);

    visualization_msgs::MarkerArray generateMarkerArray();

    static std::vector<visualization_msgs::Marker> createSphereMarkers(const geometry_msgs::Point& center, double radius, int numPoints);

    visualization_msgs::MarkerArray getCornersDrawSpace();

    Eigen::Vector3d bottomLeftCorner;
    Eigen::Vector3d topLeftCorner;
    Eigen::Vector3d topRightCorner;
    Eigen::Vector3d bottomRightCorner;
    Eigen::Affine3d TdrawSpace;
    double drawSpaceHeight;
    double drawSpaceWidth;
    ContourData contourData;
    std::vector<std::vector<Eigen::Vector3d>> spatialData;
    static Operations* operationsInstance;
    static std::string frameName;
};

#endif