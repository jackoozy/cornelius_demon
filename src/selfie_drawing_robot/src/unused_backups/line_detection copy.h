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
    std::vector<double> strokeWidths;
    std::vector<std::vector<cv::Point>> fillRegions;
    double total_arc_length;
};

class Line_detection
{

public:
    Line_detection(); // Constructor
    cv::Mat edgeDetection(cv::Mat &input_image, int kernal_size);
    void displayValue() const;
    void begin(std::string imagePath);

private:
    cv::Mat input_image;
    cv::Mat backgroundSubtraction(cv::Mat &input_image);
    cv::Rect FaceLocationDetection(cv::Mat &input_image);
    cv::Rect expandRectanglePercentage(cv::Mat &input_image, const cv::Rect &originalRect, int percentage);
    cv::Mat changeContrast(cv::Mat &input_image, double alpha, int beta);
    void applyCLAHE(cv::Mat &input_image);

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

    std::vector<std::vector<cv::Point>> bezierCurveApprox(const std::vector<std::vector<cv::Point>> &contours_, int segmentCount_);

    void addFillRegions(contourData &contourGroup_, cv::Mat copyInput, int numGrids1D, double similarityThreshold);

    // SVG stuff
    std::string svg_header = "<svg width=\"100%\" height=\"100%\" version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\"> \n";
    std::string svg_footer = "</svg>";

    std::string contours_to_svg(contourData contours_, std::string fileName_);
};

#endif // LINE_DETECTION_H
