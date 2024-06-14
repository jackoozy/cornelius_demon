#include <gtest/gtest.h>
#include <GLFW/glfw3.h>
#include <GL/glut.h>
#include "../src/operations.h"

std::vector<std::vector<Eigen::Vector3d>> POINTS;

void printContour(const std::vector<Point>& contour) {
    for (const auto& point : contour) {
        std::cout << "(" << point.x << ", " << point.y << ") ";
    }
    std::cout << std::endl;
}

// Test the capacity to map point in the normalised page coordinates to spatial coordinates on the draw-sapce
TEST(MPTest, testMappingToDrawSpace) {
    Operations ops;

    // Set the corners and dimensions of drawspace
    ops.bottomLeftCorner = Eigen::Vector3d(3,4.5,5);
    ops.topLeftCorner = Eigen::Vector3d(3,4.5,6.5);
    ops.topRightCorner = Eigen::Vector3d(3,2,6.5);
    ops.bottomRightCorner = Eigen::Vector3d(3,2,5);

    ops.calculateDrawSpaceTransformation();

    Point testPoint;
    testPoint.x = 0.5; 
    testPoint.y = 0.5;

    Eigen::Vector3d testPointWorld = ops.drawSpaceToWorld(testPoint);
    // Extract the individual components of the resulting vector
    double x = testPointWorld.x();
    double y = testPointWorld.y();
    double z = testPointWorld.z();

    // Define the expected values
    double expected_x = 3.0;
    double expected_y = 3.25;
    double expected_z = 5.75;

    // Check if the components are equal to the expected values
    EXPECT_DOUBLE_EQ(x, expected_x);
    EXPECT_DOUBLE_EQ(y, expected_y);
    EXPECT_DOUBLE_EQ(z, expected_z);
}

// Qualitative test: capability to convert SVG data into usable vector format where coordinates normalised [0, 1]
TEST(MPTest, testSVGConversion) {
    Operations ops;
    
    ContourData ctr = ops.svg_to_contours("sample_5");

    // Print out the contours
    for (size_t i = 0; i < ctr.contours.size(); ++i) {
        std::cout << "Contour " << i + 1 << ":" << std::endl;
        printContour(ctr.contours[i]);
    }
}


// Define a main function
int main(int argc, char **argv) {
    // Initialize Google Test framework
    testing::InitGoogleTest(&argc, argv);
    // Run all tests
    return RUN_ALL_TESTS();
}
