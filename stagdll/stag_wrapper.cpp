// stag_wrapper.cpp

#include "pch.h"
#include "src/Stag.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <array>
using cv::Mat;
using namespace stag;

#ifdef _WIN32
#define STAG_API __declspec(dllexport)
#else
#define STAG_API __attribute__((visibility("default")))
#endif

extern "C" STAG_API void FindStagCorners(
    unsigned char* image,
    int width, int height,
    float* result,
    int resultLen)
{
    cv::Mat img(height, width, CV_8UC1, (void*)image);
    if (img.channels() > 1)
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    auto rejectedImgPoints = std::vector<std::vector<cv::Point2f>>(); // optional, helpful for debugging
    stag::detectMarkers(img, 21, corners, ids, -1, rejectedImgPoints);

    std::fill(result, result + resultLen, 0.0f);      // safety

    int numMarkers = std::min<int>(corners.size(), 2);
    result[0] = static_cast<float>(numMarkers);

    auto putCorner = [&](int markerIdx, int cornerIdx, int dst) {
        result[dst] = corners[markerIdx][cornerIdx].x;
        result[dst + 1] = corners[markerIdx][cornerIdx].y;
        };

    if (numMarkers >= 1)
        for (int k = 0; k < 4; ++k) putCorner(0, k, 1 + k * 2);
    if (numMarkers >= 2)
        for (int k = 0; k < 4; ++k) putCorner(1, k, 9 + k * 2);
}
