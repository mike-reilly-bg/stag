#include "src/Stag.h"
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <iostream>
using cv::Mat;

# define DEBUG 0

int main() {
    // load image
    cv::Mat image = cv::imread("../example.jpg");

    // set HD library
    int libraryHD = 23;

    auto corners = std::vector<std::vector<cv::Point2f>>();
    auto ids = std::vector<int>();
    auto rejectedImgPoints = std::vector<std::vector<cv::Point2f>>(); // optional, helpful for debugging

    // detect markers
    stag::detectMarkers(image, libraryHD, corners, ids, -1, rejectedImgPoints);
    //std::cout << static_cast<int>(rejectedImgPoints.size()) << std::endl;
#if DEBUG
    char a;
    std::cout << "\n\n";
    std::cin >> a;
#endif

    // draw and save results
    stag::drawDetectedMarkers(image, corners, ids);
    cv::imwrite("example_result.jpg", image);

    return 0;
}