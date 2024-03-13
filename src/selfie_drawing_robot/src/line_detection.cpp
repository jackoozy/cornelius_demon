/*

Ideas/improvements:
- add contrast adjustment and see how it affects the detail

*/

// line_detection.cpp
#include "line_detection.h"
#include <iostream>
#include <filesystem>

#include <ros/package.h>

// Constructor implementation
Line_detection::Line_detection(int initialValue) : value(initialValue)
{
    std::cout << "Line_detection constructor called" << std::endl;
}

void Line_detection::begin()
{
    // account for different image formats

    // find number of files inside data->faces folder

    std::string package_path = ros::package::getPath("selfie_drawing_robot");
    data_path = package_path + "/src/line_detect_data";

    int num_faces = std::distance(std::filesystem::directory_iterator(data_path + "/faces"), std::filesystem::directory_iterator{});

    std::cout << "choose a value from 0 - " << (num_faces - 1) << " to select a face: ";
    std::cin >> value;

    switch (value)
    {
    case 0:
        input_image = cv::imread(data_path + "/faces/elon_portrait.jpg", cv::IMREAD_COLOR);
        break;
    case 1:
        input_image = cv::imread(data_path + "/faces/huberman.jpg", cv::IMREAD_COLOR);
        break;
    case 2:
        input_image = cv::imread(data_path + "/faces/lex.jpg", cv::IMREAD_COLOR);
        break;
    case 3:
        input_image = cv::imread(data_path + "/faces/reeves.jpg", cv::IMREAD_COLOR);
        break;
    case 4:
        input_image = cv::imread(data_path + "/faces/zuck_HD.webp", cv::IMREAD_COLOR);
        break;
    case 5:
        input_image = cv::imread(data_path + "/faces/group.webp", cv::IMREAD_COLOR);
        break;
    default:
        std::cout << "Invalid input" << std::endl;
        break;
    }

    if (input_image.empty())
    {
        std::cerr << "Could not open or find the image" << std::endl;
        return; // Exit or handle the error appropriately
    }

    // change the dimensions of the iamge
    // cv::resize(input_image, input_image, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
    cv::Mat edgeImage = edgeDetection(input_image);

    std::vector<cv::Vec2f> lines = convertToNormalisedLines(edgeImage);

    // create something to store shaded regions. Use k-means clustering

    cv::imshow("Image", edgeImage);
    cv::waitKey(0); // Wait indefinitely for a key press
}

std::vector<cv::Vec2f> Line_detection::convertToNormalisedLines(cv::Mat &edgeImageBW)
{
    // std::vector<std::vector<float>> lines;
    std::vector<cv::Vec2f> lines;

    // first check if it is black and white
    if (!isBinary(edgeImageBW))
    {
        std::cout << "Input is not black and white" << std::endl;
        return lines;
    }

    // ensure there are more white pixels than black. This should be the case...
    // ...if the image has been edge extracted. Expecting lines to be black.
    uint whitePixels = colourCount(edgeImageBW, 255);
    uint blackPixels = colourCount(edgeImageBW, 0);

    if (blackPixels > whitePixels)
    {
        std::cout << "more black pixels than white" << std::endl;
        return lines;
    }

    // next crop the image to the max and mins of the black pixels
    // then resize it to some universal value
    cv::Rect boundaryBox = findImageBounds(edgeImageBW);

    cv::Mat croppedImage = edgeImageBW(boundaryBox);

    cv::bitwise_not(croppedImage, croppedImage);

    cv::imshow("Image", croppedImage);
    cv::waitKey(0); // Wait indefinitely for a key press

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(croppedImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat lineImage = cv::Mat::zeros(croppedImage.size(), CV_8UC3);

    // A vector to hold the level of each contour
    std::vector<int> levels(contours.size(), 0);

    // Function to recursively determine the level of each contour
    std::function<void(int, int)> determineLevels = [&](int index, int level)
    {
        if (index == -1)
            return;            // Base case: no contour
        levels[index] = level; // Set the level of the current contour
        // Recurse for all children
        determineLevels(hierarchy[index][2], level + 1);
        // Proceed to the next contour at the same level
        determineLevels(hierarchy[index][0], level);
    };

    // Start the recursion with the outermost contours (those without a parent)
    determineLevels(0, 0);

    // go through each contour and determine the length of it. if its below a certain threshold, remove it
    for (int i = contours.size() - 1; i >= 0; --i)
    {
        double length = cv::arcLength(contours[i], false);
        if (length < 50)
        {
            contours.erase(contours.begin() + i);
        }
    }

    std::cout << "number of contours: " << contours.size() << std::endl;

    const int n = 10;               // Number of contours to ghost
    const double fadeFactor = 0.9; // Factor to fade previous contours by, closer to 0 makes them fade faster

    cv::Mat background = cv::Mat::zeros(croppedImage.size(), CV_8UC3); // Initial background

    while (true)
    {
        for (size_t i = 0; i < contours.size(); ++i)
        {
            cv::Mat lineImage = cv::Mat::zeros(croppedImage.size(), CV_8UC3); // Image for the current contour

            int level = levels[i];
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
            cv::drawContours(lineImage, contours, static_cast<int>(i), color, 2, cv::LINE_8, hierarchy, 0);

            // Fade the background by blending it with a black image
            background = (1 - fadeFactor) * cv::Mat::zeros(croppedImage.size(), CV_8UC3) + fadeFactor * background;

            // Add the current contour onto the background
            cv::addWeighted(background, 1.0, lineImage, 1.0, 0, background);

            // Display the combined image
            cv::imshow("Contours Animation with Ghosting", background);

            if (cv::waitKey(50) >= 0)
                break; // Exit if any key is pressed
        }
    }

    // // display new image
    // cv::imshow("Image", lineImage);
    // cv::waitKey(0); // Wait indefinitely for a key press

    // select a starting point of a selected contour
    // compare it the starting and ending point of every contour
    // find the smallest value and compare it to a threshold
    // if below the threshold, joint them together and add it to the new list
    // if above the threshold, do this again but from the ending point of the contour

    // std::vector<std::deque<cv::Point>> formulatedContours; // this will be in the format required for the next module. each row should describe a continuous line

    // formulatedContours.resize(contours.size());
    // double distanceThreshold = 0.1; // threshold for euclidean distance between points
    // double distance = 0.0;          // euclidean distance between the two points

    // cv::Point currentPoint; // will fill with the last point of the current contour
    // cv::Point nextPoint;    // will fill with the first point of the next contour

    // size_t lineCount = 0;

    // while (!contours.empty())
    // {

    //     // add the first contour in the list
    //     formulatedContours[lineCount].insert(formulatedContours[lineCount].end(), contours[0].begin(), contours[0].end());

    //     // remove the contour from the list
    //     contours.erase(contours.begin());

    //     std::cout << "contours size" << contours.size() << std::endl;

    //     double minDistanceStart = 9999999;
    //     size_t minIndexStart = 0;
    //     double minDistanceEnd = 9999999;
    //     size_t minIndexEnd = 0;

    //     while (true)
    //     {
    //         currentPoint = formulatedContours[lineCount].back();

    //         for (size_t i = 0; i < contours.size(); ++i)
    //         {
    //             nextPoint = contours[i].front();
    //             distance = cv::norm(currentPoint - nextPoint);

    //             if (distance < minDistanceEnd)
    //             {
    //                 minDistanceEnd = distance;
    //                 minIndexEnd = i;
    //             }
    //         }

    //         if (minDistanceEnd < distanceThreshold)
    //         {
    //             formulatedContours[lineCount].insert(formulatedContours[lineCount].end(), contours[minIndexEnd].begin(), contours[minIndexEnd].end());
    //             contours.erase(contours.begin() + minIndexEnd);
    //         }
    //         else
    //         {
    //             break;
    //         }
    //     }

    //     while (true)
    //     {
    //         currentPoint = formulatedContours[lineCount].front();
    //         for (size_t i = 0; i < contours.size(); ++i)
    //         {
    //             nextPoint = contours[i].back();
    //             distance = cv::norm(currentPoint - nextPoint);

    //             if (distance < minDistanceStart)
    //             {
    //                 minDistanceStart = distance;
    //                 minIndexStart = i;
    //             }
    //         }

    //         if (minDistanceStart < distanceThreshold)
    //         {
    //             formulatedContours[lineCount].insert(formulatedContours[lineCount].begin(), contours[minIndexStart].begin(), contours[minIndexStart].end());
    //             contours.erase(contours.begin() + minIndexStart);
    //         }
    //         else
    //         {
    //             break;
    //         }
    //     }

    //     lineCount++;
    //     std::cout << "line count: " << lineCount << std::endl;
    // }

    // std::cout << "number of lines: " << lineCount << std::endl;

    // for (size_t i = 0; i < contours.size(); ++i)
    // {
    //     std::cout << "contour " << i << " has " << contours[i].size() << " points" << std::endl;
    //     formulatedContours[lineCount].insert(formulatedContours[lineCount].end(), contours[i].begin(), contours[i].end());
    //     currentPoint = contours[i].back();
    //     nextPoint = contours[i + 1].front(); // contours[hierarchy[i][0]].front();

    //     // compare their euclidean distance
    //     distance = cv::norm(currentPoint - nextPoint); // calcualte euclidean distance between the two points

    //     std::cout << "distance between points: " << distance << std::endl; // debugging

    //     if (distance < distanceThreshold)
    //     {
    //         // add to the same row
    //         formulatedContours[lineCount].insert(formulatedContours[lineCount].end(), contours[i + 1].begin(), contours[i + 1].end());
    //     }
    //     else
    //     {
    //         // start a new row
    //         lineCount++;
    //     }
    // }

    // display new image
    cv::imshow("Image", lineImage);
    cv::waitKey(0); // Wait indefinitely for a key press

    // use the concept of momentum. If the line is being drawn in a similar direction,...
    // ... keep it apart of the same row.
    // need to keep track of what pixels have been visited

    // each row represents a new line. keep track of the average angle of this row,
    // if the next line creates an angle greater than some threshold, start new line

    // implement Douglas-Peucker algorithm to point count on lines

    return lines;
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
cv::Mat Line_detection::edgeDetection(cv::Mat &input_image)
{
    // maintain image aspect ratio
    int rows = 800;
    int cols = static_cast<int>((static_cast<double>(input_image.cols) / input_image.rows) * rows);

    // Resize the image to the new dimensions
    cv::resize(input_image, input_image, cv::Size(cols, rows), 0, 0, cv::INTER_LINEAR);

    // finds face and removes background
    input_image = backgroundSubtraction(input_image);

    // blur image
    int kernal_size = 5;
    cv::Mat image_blurred;
    GaussianBlur(input_image, image_blurred, cv::Size(kernal_size, kernal_size), 0);

    cv::imshow("Image", image_blurred);
    cv::waitKey(0); // Wait indefinitely for a key press

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
        std::cout << numFaces << " faces detected. Returning the first detected face." << std::endl;

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

// Method to display the value
void Line_detection::displayValue() const
{
    std::cout << "Value: " << value << std::endl;
}
