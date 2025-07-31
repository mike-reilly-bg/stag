// stag_wrapper.cpp

#include "pch.h"
#include "src/Stag.h"
#include <opencv2/opencv.hpp>
#include <vector>
using cv::Mat;
using namespace stag;

extern "C" __declspec(dllexport)
int __cdecl FindStagCorners(const unsigned char* image, int width, int height, int maxMarkers)
{
    try {
        cv::Mat img(height, width, CV_8UC1, (void*)image);
        if (img.channels() > 1) cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> rejectedImgPoints;
        stag::detectMarkers(img, 21, corners, ids, rejectedImgPoints);

        int numMarkers = std::min((int)corners.size(), maxMarkers);
        /*for (int i = 0; i < numMarkers; ++i) {
            for (int k = 0; k < 4; ++k) {
                outCorners[i * 8 + k * 2 + 0] = corners[i][k].x;
                outCorners[i * 8 + k * 2 + 1] = corners[i][k].y;
            }
        }*/
        //return numMarkers;
        return 42;
    }
    catch (...) {
        // Could also log the error
        return 0;
    }
}


//extern "C" __declspec(dllexport)
//int FindStagCorners(const unsigned char* image, int width, int height, double* outCorners, int maxMarkers)
//{
//    cv::Mat img(height, width, CV_8UC1, (void*)image);
//    int libraryHD = 21; // Use appropriate dictionary as needed
//
//    std::vector<std::vector<cv::Point2f>> corners;
//    std::vector<int> ids;
//    std::vector<std::vector<cv::Point2f>> rejectedImgPoints;
//
//    stag::detectMarkers(img, libraryHD, corners, ids, rejectedImgPoints);
//
//    int count = 0;
//    for (const auto& c : corners) {
//        if (count >= maxMarkers) break;
//        for (int k = 0; k < 4; ++k) {
//            outCorners[count * 8 + k * 2 + 0] = c[k].x;
//            outCorners[count * 8 + k * 2 + 1] = c[k].y;
//        }
//        ++count;
//    }
//    return count;
//}