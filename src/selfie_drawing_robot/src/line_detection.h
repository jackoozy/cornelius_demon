// line_detection.h

#include <opencv2/opencv.hpp>

#ifndef LINE_DETECTION_H
#define LINE_DETECTION_H

class Line_detection
{
private:
    int value;
    cv::Mat input_image;
    cv::Mat backgroundSubtraction(cv::Mat &input_image);
    cv::Rect FaceLocationDetection(cv::Mat &input_image);
    cv::Rect expandRectanglePercentage(cv::Mat &input_image, const cv::Rect &originalRect, int percentage);

    // expects a black and white (lines are black) image that has been reduces to edges
    std::vector<cv::Vec2f> convertToNormalisedLines(cv::Mat &edgeImageBW);

    bool isBinary(const cv::Mat &image);
    bool isGrayscale(const cv::Mat &image);
    uint colourCount(const cv::Mat &image, int colourValue);

    cv::Rect findImageBounds(cv::Mat &edgeImageBW);

    std::string data_path;


public:
    Line_detection(int initialValue); // Constructor
    cv::Mat edgeDetection(cv::Mat &input_image);
    void displayValue() const;
    void begin(void);
};

#endif // LINE_DETECTION_H
