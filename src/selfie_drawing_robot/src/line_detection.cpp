/*

search for '!!!' in document, these are areas that need to be tested, improved or fixed

Ideas/improvements:
- add contrast adjustment and see how it affects the detail

*/

// line_detection.cpp
#include "line_detection.h"
#include <iostream>
#include <filesystem>

#include <ros/package.h>

// Constructor implementation
Line_detection::Line_detection()
{
    std::cout << "Line_detection constructor called" << std::endl;
}

void Line_detection::begin(std::string imagePath)
{
    // account for different image formats

    // find number of files inside data->faces folder

    input_image = cv::imread(imagePath, cv::IMREAD_COLOR);

    if (input_image.empty())
    {
        std::cerr << "Could not open or find the image" << std::endl;
        return; // Exit or handle the error appropriately
    }

    // first change contrast of image
    applyCLAHE(input_image);

    // dispaly iamge
    cv::imshow("Contrasted Image", input_image);
    cv::waitKey(0); // Wait indefinitely for a key press

    // change the dimensions of the iamge
    // cv::resize(input_image, input_image, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);

    contourData contourGroup;

    int max_length = 8000; // can still be adjusted but this value worked well with the sample images

    int kernal = 9;

    cv::Mat edgeImage;

    while (contourGroup.total_arc_length < max_length)
    {
        cv::Mat copyInput = input_image.clone();

        edgeImage = edgeDetection(copyInput, kernal);
        contourGroup = bwImageToContours(edgeImage);

        // print total arc length
        std::cout << "total arc length: " << contourGroup.total_arc_length << " Kernal size: " << kernal << std::endl;

        if (contourGroup.total_arc_length < max_length)
        {
            switch (kernal)
            {
            case 9:
                kernal = 7; // Reduce the kernel size
                break;
            case 7:
                kernal = 5; // Reduce further
                break;
            case 5:
                kernal = 3; // Reduce further
                break;
            default:
                kernal = 1; // Minimum kernel size
                break;
            }
        }
    }

    // show final image
    cv::imshow("Edge iamge", edgeImage);
    cv::waitKey(0); // Wait indefinitely for a key press

    // contourGroup.contours = bezierCurveApprox(contourGroup.contours, 39);

    // Animation of contours with ghosting
    animateContours(contourGroup);

    std::cout << "save file? (1 = yes | 0 = no)" << std::endl;
    int response;
    std::cin >> response;

    if (response == 1)
    {
        std::cout << "select name" << std::endl;
        std::string fileName;
        std::cin >> fileName;

        contours_to_svg(contourGroup, fileName);
    }
}

contourData Line_detection::bwImageToContours(cv::Mat &edgeImageBW)
{
    contourData contourGroup;

    // first check if it is black and white
    if (!isBinary(edgeImageBW))
    {
        std::cout << "Input is not black and white" << std::endl;
        return contourGroup;
    }

    // ensure there are more white pixels than black. This should be the case...
    // ...if the image has been edge extracted. Expecting lines to be black.
    uint whitePixels = colourCount(edgeImageBW, 255);
    uint blackPixels = colourCount(edgeImageBW, 0);

    if (blackPixels > whitePixels)
    {
        std::cout << "more black pixels than white" << std::endl;
        return contourGroup;
    }

    // next crop the image to the max and mins of the black pixels
    // then resize it to some universal value
    cv::Rect boundaryBox = findImageBounds(edgeImageBW);

    contourGroup.croppedImage = edgeImageBW(boundaryBox);

    cv::bitwise_not(contourGroup.croppedImage, contourGroup.croppedImage);

    // cv::imshow("Image", contourGroup.croppedImage);
    // cv::waitKey(0); // Wait indefinitely for a key press

    cv::findContours(contourGroup.croppedImage, contourGroup.contours, contourGroup.hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // A vector to hold the level of each contour
    contourGroup.levels = std::vector<int>(contourGroup.contours.size(), 0);

    // Function to recursively determine the level of each contour
    std::function<void(int, int)> determineLevels = [&](int index, int level)
    {
        if (index == -1)
            return;                         // Base case: no contour
        contourGroup.levels[index] = level; // Set the level of the current contour
        // Recurse for all children
        determineLevels(contourGroup.hierarchy[index][2], level + 1);
        // Proceed to the next contour at the same level
        determineLevels(contourGroup.hierarchy[index][0], level);
    };

    // Start the recursion with the outermost contours (those without a parent)
    determineLevels(0, 0);

    // go through each contour and determine the length of it. if its below a certain threshold, remove it
    for (int i = contourGroup.contours.size() - 1; i >= 0; --i)
    {
        double length = cv::arcLength(contourGroup.contours[i], false);

        // !!! this is arbitrary and needs to be tested
        if (length < 50)
        {
            contourGroup.contours.erase(contourGroup.contours.begin() + i);
        }
    }

    std::cout << "number of contours: " << contourGroup.contours.size() << std::endl;

    // !!! epsilon value needs to be tested on varying faces
    contourGroup.contours = douglasPeuckerReduction(contourGroup.contours, 2);

    double maxArcLength;
    double minArcLength;
    contourGroup.total_arc_length = 0;

    for (size_t i = 0; i < contourGroup.contours.size(); ++i)
    {
        double arcLength = cv::arcLength(contourGroup.contours[i], false);
        if (i == 0)
        {
            maxArcLength = arcLength;
            minArcLength = arcLength;
        }
        else
        {
            if (arcLength > maxArcLength)
            {
                maxArcLength = arcLength;
            }
            if (arcLength < minArcLength)
            {
                minArcLength = arcLength;
            }
        }
        contourGroup.total_arc_length += arcLength;
    }

    for (size_t i = 0; i < contourGroup.contours.size(); ++i)
    {
        double arcLength = cv::arcLength(contourGroup.contours[i], false);
        double strokeWidth = 5 * ((arcLength - minArcLength) / (maxArcLength - minArcLength)) + 1.0;
        contourGroup.strokeWidths.push_back(strokeWidth);
    }

    // !!! look into spline fitting to polylines

    return contourGroup;
}

void Line_detection::animateContours(contourData &contourGroup_)
{
    const int iterationPeriod = 5; // Number of frames to display
    const int n = 10;              // Number of contours to ghost
    const double fadeFactor = 0.9; // Factor to fade previous contours by, closer to 0 makes them fade faster

    cv::Mat background = cv::Mat::zeros(contourGroup_.croppedImage.size(), CV_8UC3); // Initial background

    for (int iteration = 0; iteration < iterationPeriod; ++iteration)
    {
        for (size_t i = 0; i < contourGroup_.contours.size(); ++i)
        {
            cv::Mat lineImage = cv::Mat::zeros(contourGroup_.croppedImage.size(), CV_8UC3); // Image for the current contour

            int level = contourGroup_.levels[i];
            cv::Scalar color;
            switch (level % 4)
            {
            case 0:
                color = cv::Scalar(255, 0, 0);
                break; // Blue
            case 1:
                color = cv::Scalar(0, 255, 0);
                break; // Green
            case 2:
                color = cv::Scalar(0, 0, 255);
                break; // Red
            case 3:
                color = cv::Scalar(255, 255, 0);
                break; // Cyan
            }

            // Draw the current contour with full intensity
            cv::drawContours(lineImage, contourGroup_.contours, static_cast<int>(i), color, 2, cv::LINE_8, contourGroup_.hierarchy, 0);

            // Fade the background by blending it with a black image
            background = (1 - fadeFactor) * cv::Mat::zeros(contourGroup_.croppedImage.size(), CV_8UC3) + fadeFactor * background;

            // Add the current contour onto the background
            cv::addWeighted(background, 1.0, lineImage, 1.0, 0, background);

            // Display the combined image
            cv::imshow("Contours Animation with Ghosting", background);

            if (cv::waitKey(50) >= 0)
                break; // Exit if any key is pressed
        }
    }
}

cv::Rect Line_detection::findImageBounds(cv::Mat &image)
{
    uint maxX = 0;
    uint minX = 99999999; // big boy

    uint maxY = 0;
    uint minY = 99999999; // big boy

    for (uint i = 0; i < image.rows; ++i)
    {
        for (uint j = 0; j < image.cols; ++j)
        {
            if (image.at<uchar>(i, j) == 0)
            {
                if (i > maxY)
                {
                    maxY = i;
                }
                if (i < minY)
                {
                    minY = i;
                }

                if (j > maxX)
                {
                    maxX = j;
                }
                if (j < minX)
                {
                    minX = j;
                }
            }
        }
    }

    cv::Rect rect(minX, minY, (maxX - minX), (maxY - minY));
    return rect;
}

uint Line_detection::colourCount(const cv::Mat &image, int colourValue)
{
    uint count = 0;
    for (uint i = 0; i < image.rows; ++i)
    {
        for (uint j = 0; j < image.cols; ++j)
        {
            uchar pixelValue = image.at<uchar>(i, j);
            if (pixelValue == colourValue)
            {
                count++;
            }
        }
    }
    return count;
}

bool Line_detection::isBinary(const cv::Mat &image)
{
    // Ensure it's grayscale first
    if (!isGrayscale(image))
    {
        std::cout << "image is not grayscale" << std::endl;
        return false;
    }

    for (int i = 0; i < image.rows; ++i)
    {
        for (int j = 0; j < image.cols; ++j)
        {
            uchar pixelValue = image.at<uchar>(i, j);
            if (pixelValue != 0 && pixelValue != 255)
            {
                return false; // Found a pixel that is neither black nor white
            }
        }
    }
    return true; // All pixels are either black or white
}

bool Line_detection::isGrayscale(const cv::Mat &image)
{
    // Check if image is single channel
    return image.channels() == 1;
}

// Method to set the value
cv::Mat Line_detection::edgeDetection(cv::Mat &input_image, int kernal_size)
{
    // maintain image aspect ratio
    int rows = 800;
    int cols = static_cast<int>((static_cast<double>(input_image.cols) / input_image.rows) * rows);

    // Resize the image to the new dimensions
    cv::resize(input_image, input_image, cv::Size(cols, rows), 0, 0, cv::INTER_LINEAR);

    // finds face and removes background
    input_image = backgroundSubtraction(input_image);

    // blur image
    cv::Mat image_blurred;
    GaussianBlur(input_image, image_blurred, cv::Size(kernal_size, kernal_size), 0);

    // cv::imshow("Blurred Image", image_blurred);
    // cv::waitKey(0); // Wait indefinitely for a key press

    cv::Mat gray, edge, draw;
    cv::cvtColor(image_blurred, gray, cv::COLOR_BGR2GRAY);

    cv::Canny(gray, edge, 50, 150, 3);
    edge.convertTo(draw, CV_8U);

    // maintain image aspect ratio
    rows = 500;
    cols = static_cast<int>((static_cast<double>(input_image.cols) / input_image.rows) * rows);

    // Resize the image to the new dimensions
    cv::resize(draw, draw, cv::Size(cols, rows), 0, 0, cv::INTER_LINEAR);

    cv::bitwise_not(draw, draw); // inverting the binary image and storing it in inverted_binary_image matrix//

    // ensure it is a black and white iamge
    cv::threshold(draw, draw, 200, 255, cv::THRESH_BINARY);

    return draw;
}

cv::Mat Line_detection::backgroundSubtraction(cv::Mat &input_image)
{
    cv::Mat mask;
    mask.create(input_image.size(), CV_8UC1);
    mask.setTo(cv::GC_BGD); // Background label

    // cv::Rect rectangle(100, 50, input_image.cols-200, input_image.rows-100);
    cv::Rect rectangle = FaceLocationDetection(input_image);

    // Apply GrabCut
    cv::Mat bgdModel, fgdModel;
    cv::grabCut(input_image, mask, rectangle, bgdModel, fgdModel, 5, cv::GC_INIT_WITH_RECT);

    // Modify the mask to create a binary mask of the foreground
    cv::compare(mask, cv::GC_PR_FGD, mask, cv::CMP_EQ);

    // Extract the foreground
    cv::Mat foreground(input_image.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    input_image.copyTo(foreground, mask); // Copies only where mask is non-zero

    // Crop the image to the bounding box of the foreground
    cv::Rect boundedROI = rectangle & cv::Rect(0, 0, foreground.cols, foreground.rows);
    cv::Mat croppedImage = foreground(boundedROI).clone();

    return croppedImage;
}

cv::Rect Line_detection::FaceLocationDetection(cv::Mat &input_image)
{
    // Load the Haar Cascade file for facial detection
    cv::CascadeClassifier faceCascade;
    faceCascade.load(data_path + "/haarcascade_frontalface_default.xml");

    // Check if the Haar Cascade file is loaded
    if (faceCascade.empty())
    {
        std::cout << "Failed to load Haar Cascade file." << std::endl;
        return cv::Rect(); // Return an empty rectangle
    }

    std::vector<cv::Rect> faces;

    // Detect faces
    faceCascade.detectMultiScale(input_image, faces);

    int numFaces = faces.size();

    cv::Rect face;

    if (numFaces > 1)
    {
        std::cout << numFaces << " faces detected. Returning the largest detected face." << std::endl;

        float maxArea = 0;
        int maxAreaIndex = 0;

        // find the face with the largest bounding box
        for (int i = 0; i < numFaces; i++)
        {
            if (faces[i].area() > maxArea)
            {
                maxArea = faces[i].area();
                maxAreaIndex = i;
            }
        }
        face = faces[maxAreaIndex];
    }
    else
    {
        face = faces[0];
    }

    cv::Rect expandedFaceRect = expandRectanglePercentage(input_image, face, 20);

    return expandedFaceRect;
}

cv::Rect Line_detection::expandRectanglePercentage(cv::Mat &input_image, const cv::Rect &originalRect, int percentage)
{
    int expandByX = static_cast<int>(originalRect.width * percentage / 100.0);
    int expandByY = static_cast<int>(originalRect.height * percentage / 100.0);

    cv::Rect expandedRect = originalRect;

    // Adjust the rectangle origin and dimensions
    expandedRect.x -= expandByX;
    expandedRect.y -= expandByY;
    expandedRect.width += 2 * expandByX;
    expandedRect.height += 2 * expandByY;

    // ensure the expanded rectangle is within the image bounds
    expandedRect.x = std::max(expandedRect.x, 0);
    expandedRect.y = std::max(expandedRect.y, 0);
    expandedRect.width = std::min(expandedRect.width, input_image.cols - expandedRect.x);
    expandedRect.height = std::min(expandedRect.height, input_image.rows - expandedRect.y);

    return expandedRect;
}

std::string Line_detection::contours_to_svg(contourData contourGroup_, std::string fileName_)
{
    std::string svg_content;
    std::string svg_paths;

    for (size_t i = 0; i < contourGroup_.contours.size(); ++i)
    {
        std::string path = "<path d=\"M"; // Move to the starting point
        for (size_t j = 0; j < contourGroup_.contours[i].size(); ++j)
        {
            path += std::to_string(contourGroup_.contours[i][j].x) + " " + std::to_string(contourGroup_.contours[i][j].y) + " ";
        }

        path += "Z\" fill=\"none\" stroke=\"black\" stroke-width=\"" + std::to_string(contourGroup_.strokeWidths[i]) + "\" />\n";
        svg_paths += path;
    }

    svg_content = svg_header + svg_paths + svg_footer;

    // save content to file
    std::string file_name = data_path + "/" + fileName_ + ".svg";

    std::ofstream file(file_name);
    if (file.is_open())
    {
        file << svg_content;
        file.close();
    }
    else
    {
        std::cout << "Unable to open file";
    }

    return svg_content;
}

std::vector<std::vector<cv::Point>> Line_detection::douglasPeuckerReduction(std::vector<std::vector<cv::Point>> &contours, double epsilon)
{
    std::vector<std::vector<cv::Point>> reduced_contours;

    for (size_t i = 0; i < contours.size(); ++i)
    {
        std::vector<cv::Point> reduced_contour;
        cv::approxPolyDP(contours[i], reduced_contour, epsilon, true);
        reduced_contours.push_back(reduced_contour);
    }

    return reduced_contours;
}

std::vector<std::vector<cv::Point>> Line_detection::bezierCurveApprox(const std::vector<std::vector<cv::Point>> &contours_, int segmentCount_)
{
    double minDistance = 0.1; // !!! this is arbitrary and needs to be tested, still needs to be implemented

    std::vector<std::vector<cv::Point>> bezier_contours;

    for (size_t i = 0; i < contours_.size(); ++i)
    {
        std::vector<cv::Point> bezier_contour;
        // Iterate through contour points
        for (size_t j = 0; j + 2 < contours_[i].size(); j += 3)
        {
            cv::Point P0 = contours_[i][j];
            cv::Point P1 = contours_[i][j + 1];
            cv::Point P2 = contours_[i][j + 2];

            std::vector<cv::Point> curveApprox(segmentCount_ + 1);

            for (int k = 0; k <= segmentCount_; ++k)
            {
                double t = static_cast<double>(k) / segmentCount_;
                double x = std::pow(1 - t, 2) * P0.x + 2 * (1 - t) * t * P1.x + std::pow(t, 2) * P2.x;
                double y = std::pow(1 - t, 2) * P0.y + 2 * (1 - t) * t * P1.y + std::pow(t, 2) * P2.y;
                bezier_contour.push_back(cv::Point(static_cast<int>(x), static_cast<int>(y)));
            }
        }
        bezier_contours.push_back(bezier_contour);
    }

    return bezier_contours;
}

// Adaptive Histogram Equalization (CLAHE)
void Line_detection::applyCLAHE(cv::Mat &input_image)
{
    cv::Mat lab_image;
    cv::cvtColor(input_image, lab_image, cv::COLOR_BGR2Lab);

    // Spli t the image into channels
    std::vector<cv::Mat> lab_planes;
    cv::split(lab_image, lab_planes);

    // Apply CLAHE to the lightness channel
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(1.0);
    clahe->apply(lab_planes[0], lab_planes[0]);

    // Merge the channels back
    cv::merge(lab_planes, lab_image);

    // Convert back to RGB
    cv::cvtColor(lab_image, input_image, cv::COLOR_Lab2BGR);
}
