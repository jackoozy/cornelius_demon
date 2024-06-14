#include "operations.h"

Operations* Operations::operationsInstance = nullptr;
std::string Operations::frameName = "base_link";

// Computes TdrawSpace and drawSpaceHeight and drawSpaceWidth. 
// Ensure that draw-space corners are set first.
Eigen::Affine3d Operations::calculateDrawSpaceTransformation() {
    // Calculate the vectors for the edges of the paper page
    Eigen::Vector3d xAxis = topRightCorner - topLeftCorner;
    drawSpaceWidth = xAxis.norm();
    Eigen::Vector3d yAxis = bottomLeftCorner - topLeftCorner;
    drawSpaceHeight = yAxis.norm();

    // Normalize the vectors
    xAxis.normalize();
    yAxis.normalize();
    // Calculate the z-axis (normal to the paper plane) using the cross product of the x-axis and y-axis
    Eigen::Vector3d zAxis = xAxis.cross(yAxis);
    zAxis.normalize();
    // Construct the rotation matrix using the vectors as columns
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix.col(0) = xAxis;
    rotationMatrix.col(1) = yAxis;
    rotationMatrix.col(2) = zAxis;
    // Create an affine transformation from rotation matrix and translation vector
    Eigen::Affine3d transformation(Eigen::Translation3d(topLeftCorner) * rotationMatrix);

    TdrawSpace = transformation;

    return transformation;
}

// Takes in normalised draw-space point and returns 3D point.
Eigen::Vector3d Operations::drawSpaceToWorld(const Point& pagePoint, const double& depth) {
    Eigen::Vector2d pt(pagePoint.x, pagePoint.y);
    double dim = std::min(drawSpaceWidth, drawSpaceHeight);
    double x = pt.y()*dim;
    double y = pt.x()*dim;
    double z = depth*0.001;
    if (drawSpaceWidth > drawSpaceHeight){
        x+=(drawSpaceWidth-drawSpaceHeight)/2;
    } else {
        y+=(drawSpaceHeight-drawSpaceWidth)/2;
    }
    // Transform the 2D point into a 3D point on the plane
    Eigen::Vector3d pagePoint3D(x,y,z);
    // Apply the transformation to convert the point to the world frame
    Eigen::Vector3d worldPoint = TdrawSpace * pagePoint3D;
    return worldPoint;
}

void Operations::renderSpatialData(){
    spatialData.clear();
    std::vector<Eigen::Vector3d> strokeWorld;
    for (int stroke = 0; stroke < contourData.contours.size(); ++stroke) {
        strokeWorld.clear();
        for (auto& point : contourData.contours.at(stroke)) {
            strokeWorld.push_back(drawSpaceToWorld(point, contourData.strokeWidths.at(stroke)));
        }
        spatialData.push_back(strokeWorld);
    }
}

visualization_msgs::MarkerArray Operations::getCornersDrawSpace() {
    visualization_msgs::MarkerArray markerArray;

    // Create green sphere markers for each corner
    for (int i = 0; i < 4; ++i) {
        visualization_msgs::Marker greenSphere;
        greenSphere.header.frame_id = frameName;
        greenSphere.header.stamp = ros::Time::now();
        greenSphere.type = visualization_msgs::Marker::SPHERE;
        greenSphere.action = visualization_msgs::Marker::ADD;
        greenSphere.pose.orientation.w = 1.0;
        greenSphere.scale.x = 0.01;  // Adjust the scale as needed
        greenSphere.scale.y = 0.01;
        greenSphere.scale.z = 0.01;
        greenSphere.color.a = 1.0;  // Fully opaque
        greenSphere.color.r = 0.0;  // No red
        greenSphere.color.g = 1.0;  // Full green
        greenSphere.color.b = 0.0;  // No blue

        // Set the positions of the spheres based on the corner locations
        switch (i) {
            case 0:
                greenSphere.pose.position.x = bottomLeftCorner.x();
                greenSphere.pose.position.y = bottomLeftCorner.y();
                greenSphere.pose.position.z = bottomLeftCorner.z();
                break;
            case 1:
                greenSphere.pose.position.x = topLeftCorner.x();
                greenSphere.pose.position.y = topLeftCorner.y();
                greenSphere.pose.position.z = topLeftCorner.z();
                break;
            case 2:
                greenSphere.pose.position.x = topRightCorner.x();
                greenSphere.pose.position.y = topRightCorner.y();
                greenSphere.pose.position.z = topRightCorner.z();
                break;
            case 3:
                greenSphere.pose.position.x = bottomRightCorner.x();
                greenSphere.pose.position.y = bottomRightCorner.y();
                greenSphere.pose.position.z = bottomRightCorner.z();
                break;
            default:
                break;
        }

        // Assign a unique id to each marker
        greenSphere.id = i;

        // Add the marker to the array
        markerArray.markers.push_back(greenSphere);
    }

    return markerArray;
}

ContourData Operations::svg_to_contours(const std::string& fileName_)
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

    // Normalise the contour data
    float max = 0.0f;
    for (const auto& contour : contourGroup_.contours) 
        for (const auto& point : contour) {
            max = std::max(max, point.x);
            max = std::max(max, point.y);
        }
    for (auto& contour : contourGroup_.contours) 
        for (auto& point : contour) {
            point.x = point.x / max;
            point.y = point.y / max;
        }
    for (auto& fillRegion : contourGroup_.fillRegions) 
        for (auto& point : fillRegion) {
            point.x = point.x / max;
            point.y = point.y / max;
        }
    

    contourData = contourGroup_;

    return contourGroup_;
}

visualization_msgs::MarkerArray Operations::generateMarkerArray() {
    visualization_msgs::MarkerArray marker_array;
    double sizeMultiplier = 0.002;
    double size;
    double minDistance = 0.01; // Minimum distance between consecutive points

    // Iterate through spatialData and create markers for each point
    int marker_id = 0;

    for (int stroke = 0; stroke < spatialData.size(); ++stroke) {
        size = (contourData.strokeWidths.at(stroke) * sizeMultiplier + 0.005)/10;

        // Iterate through points in the stroke
        for (int i = 0; i < spatialData.at(stroke).size() - 1; ++i) {
            const auto& point1 = spatialData.at(stroke).at(i);
            const auto& point2 = spatialData.at(stroke).at(i + 1);

            // Calculate distance between consecutive points
            double distance = (point2 - point1).norm();

            // Insert intermediate points if distance exceeds stroke size
            int numPoints = std::max(1, static_cast<int>(2 * distance / size));
            for (int j = 0; j < numPoints; ++j) {
                double ratio = static_cast<double>(j) / numPoints;
                Eigen::Vector3d intermediatePoint = point1 + ratio * (point2 - point1);

                // Create marker for intermediate point
                visualization_msgs::Marker marker;
                marker.header.frame_id = frameName; // Change the frame_id if needed
                marker.header.stamp = ros::Time::now();
                marker.id = marker_id++;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = intermediatePoint.x();
                marker.pose.position.y = intermediatePoint.y();
                marker.pose.position.z = intermediatePoint.z();
                marker.scale.x = size; // Adjust the scale as needed
                marker.scale.y = size;
                marker.scale.z = size;
                marker.color.a = 0.6; // 60% opacity
                marker.color.r = 1.0; // Red color
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker_array.markers.push_back(marker);
            }
        }
    }

    return marker_array;
}

std::vector<visualization_msgs::Marker> Operations::createSphereMarkers(const geometry_msgs::Point& center, double radius, int numPoints) {
    std::vector<visualization_msgs::Marker> markers;
    double phiIncrement = M_PI / (numPoints - 1);
    double thetaIncrement = 2 * M_PI / numPoints;
    for (int i = 0; i < numPoints; ++i) {
        for (int j = 0; j < numPoints; ++j) {
            double phi = i * phiIncrement;
            double theta = j * thetaIncrement;
            double x = center.x + radius * sin(phi) * cos(theta);
            double y = center.y + radius * sin(phi) * sin(theta);
            double z = center.z + radius * cos(phi);
            
            visualization_msgs::Marker marker;
            marker.header.frame_id = frameName; // Set the frame ID of your choice
            marker.ns = "spheres";
            marker.id = i * numPoints + j;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            marker.scale.x = 0.02; // Adjust the scale as needed
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
            marker.color.a = 0.6; // Fully opaque
            marker.color.r = 1.0; // Red color
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            markers.push_back(marker);
        }
    }
    return markers;
}