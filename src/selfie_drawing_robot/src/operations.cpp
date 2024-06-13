#include "operations.h"

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
Eigen::Vector3d Operations::drawSpaceToWorld(const Eigen::Vector2d& pagePoint) {
    // Transform the 2D point into a 3D point on the plane
    Eigen::Vector3d pagePoint3D(pagePoint.x()*drawSpaceWidth, pagePoint.y()*drawSpaceHeight, 0.0);
    // Apply the transformation to convert the point to the world frame
    Eigen::Vector3d worldPoint = TdrawSpace * pagePoint3D;
    return worldPoint;
}

void Operations::testFunction(){
  bottomLeftCorner = Eigen::Vector3d(3,4.5,5);
  topLeftCorner = Eigen::Vector3d(3,4.5,6.5);
  topRightCorner = Eigen::Vector3d(3,2,6.5);
  bottomRightCorner = Eigen::Vector3d(3,2,5);


  calculateDrawSpaceTransformation();

  Eigen::Vector2d testPoint(0.5, 0.5);

  Eigen::Vector3d testPointWorld = drawSpaceToWorld(testPoint);
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "Vector: " << testPointWorld.transpose() << std::endl;
}