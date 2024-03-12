/*

Ideas/improvements:
- add contrast adjustment and see how it affects the detail

*/


// line_detection.cpp
#include "line_detection.h"
#include <iostream>
#include <filesystem>

// Constructor implementation
Line_detection::Line_detection(int initialValue) : value(initialValue)
{
    std::cout << "Line_detection constructor called" << std::endl;
}

void Line_detection::begin()
{
    // account for different image formats

    // find number of files inside data->faces folder
    int num_faces = std::distance(std::filesystem::directory_iterator("../data/faces"), std::filesystem::directory_iterator{});

    std::cout << "choose a value from 0 - " << (num_faces - 1) << " to select a face: ";
    std::cin >> value;

    switch (value)
    {
    case 0:
        input_image = cv::imread("../data/faces/elon_portrait.jpg", cv::IMREAD_COLOR);
        break;
    case 1:
        input_image = cv::imread("../data/faces/huberman.jpg", cv::IMREAD_COLOR);
        break;
    case 2:
        input_image = cv::imread("../data/faces/lex.jpg", cv::IMREAD_COLOR);
        break;
    case 3:
        input_image = cv::imread("../data/faces/reeves.jpg", cv::IMREAD_COLOR);
        break;
    case 4:
        input_image = cv::imread("../data/faces/zuck_HD.webp", cv::IMREAD_COLOR);
        break;
    case 5:
        input_image = cv::imread("../data/faces/group.webp", cv::IMREAD_COLOR);
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

    std::vector<std::vector<float>> lines = convertToNormalisedLines(edgeImage);

    // create something to store shaded regions. Use k-means clustering

    cv::imshow("Image", edgeImage);
    cv::waitKey(0); // Wait indefinitely for a key press
}

std::vector<std::vector<float>> Line_detection::convertToNormalisedLines(cv::Mat &edgeImageBW)
{
    std::vector<std::vector<float>> lines;

    // first check if it is black and white
    if (!isBinary(edgeImageBW))
    {
        std::cout << "Input is not black and white" << std::endl;
        return lines;
    }

    // ensure there are more white pixels than black. This should be the case...
    // ...if the image has been edge extracted, contains black lines.
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

    cv::imshow("Image", croppedImage);
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
    faceCascade.load("../data/haarcascade_frontalface_default.xml");

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
