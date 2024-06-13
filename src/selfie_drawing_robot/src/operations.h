#ifndef OPERATIONS_H
#define OPERATIONS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/scoped_ptr.hpp>
#include <iostream>
#include <thread>
#include <iomanip>

class Operations {
    public:

    Operations(){}

    ~Operations(){}

    // Computes TdrawSpace and drawSpaceHeight and drawSpaceWidth. 
    // Ensure that draw-space corners are set first.
    Eigen::Affine3d calculateDrawSpaceTransformation();

    // Takes in normalised draw-space point and returns 3D point.
    Eigen::Vector3d drawSpaceToWorld(const Eigen::Vector2d& pagePoint);

    void testFunction();

    Eigen::Vector3d bottomLeftCorner;
    Eigen::Vector3d topLeftCorner;
    Eigen::Vector3d topRightCorner;
    Eigen::Vector3d bottomRightCorner;
    Eigen::Affine3d TdrawSpace;
    double drawSpaceHeight;
    double drawSpaceWidth;

};

#endif