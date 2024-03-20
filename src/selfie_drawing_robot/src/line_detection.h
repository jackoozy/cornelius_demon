// line_detection.h

#include <opencv2/opencv.hpp>

#ifndef LINE_DETECTION_H
#define LINE_DETECTION_H

struct contourData
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<int> levels;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat croppedImage;
};

class Line_detection
{

public:
    Line_detection(); // Constructor
    cv::Mat edgeDetection(cv::Mat &input_image);
    void displayValue() const;
    void begin(void);

private:
    cv::Mat input_image;
    cv::Mat backgroundSubtraction(cv::Mat &input_image);
    cv::Rect FaceLocationDetection(cv::Mat &input_image);
    cv::Rect expandRectanglePercentage(cv::Mat &input_image, const cv::Rect &originalRect, int percentage);

    bool isBinary(const cv::Mat &image);
    bool isGrayscale(const cv::Mat &image);
    uint colourCount(const cv::Mat &image, int colourValue);

    cv::Rect findImageBounds(cv::Mat &edgeImageBW);

    std::string data_path;

    // contour stuff

    void animateContours(contourData &contourGroup_);
    std::vector<std::vector<cv::Point>> douglasPeuckerReduction(std::vector<std::vector<cv::Point>> &contours, double epsilon);
    // expects a black and white (lines are black) image that has been reduces to edges
    contourData bwImageToContours(cv::Mat &edgeImageBW);

    // SVG stuff
    std::string svg_header = "<svg width=\"100%\" height=\"100%\" version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\"> \n";
    std::string svg_footer = "</svg>";

    std::string contours_to_svg(std::vector<std::vector<cv::Point>> contours_, std::string fileName_);
};

#endif // LINE_DETECTION_H
