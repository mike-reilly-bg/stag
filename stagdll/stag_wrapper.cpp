// stag_wrapper.cpp

// #include "pch.h"
// #include "src/Stag.h"
// #include <opencv2/opencv.hpp>
// #include <vector>
// #include <array>
// using cv::Mat;
// using namespace stag;

// #ifdef _WIN32
// #define STAG_API __declspec(dllexport)
// #else
// #define STAG_API __attribute__((visibility("default")))
// #endif

// #define DEBUG 0

// extern "C" STAG_API void FindStagCorners(
//     unsigned char* image,
//     int width, int height,
//     double* result,
//     int resultLen,
//     int error_correction)
// {
//     cv::Mat img(height, width, CV_8UC1, (void*)image);
//     if (img.channels() > 1)
//         cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);

//     std::vector<std::vector<cv::Point2f>> corners;
//     std::vector<int> ids;
//     auto rejectedImgPoints = std::vector<std::vector<cv::Point2f>>(); // optional, helpful for debugging
//     stag::detectMarkers(img, 23, corners, ids, error_correction, rejectedImgPoints);

//     std::fill(result, result + resultLen, 0.0f);      // safety

//     int numMarkers = std::min<int>(corners.size(), 2);
//     result[0] = static_cast<double>(numMarkers);

//     auto putCorner = [&](int markerIdx, int cornerIdx, int dst) {
//         result[dst] = corners[markerIdx][cornerIdx].x;
//         result[dst + 1] = corners[markerIdx][cornerIdx].y;
//         };

//     if (numMarkers >= 1)
//         for (int k = 0; k < 4; ++k) putCorner(0, k, 1 + k * 2);
//     if (numMarkers >= 2)
//         for (int k = 0; k < 4; ++k) putCorner(1, k, 9 + k * 2);
// }






// // // stag_wrapper.cpp

// #include "pch.h"
// #include "src/Stag.h"
// #include <opencv2/opencv.hpp>
// #include <opencv2/core.hpp>
// #include <opencv2/calib3d.hpp>
// #include <vector>
// #include <algorithm>
// #include <iostream>

// using namespace cv;
// using namespace stag;

// #ifdef _WIN32
// #define STAG_API __declspec(dllexport)
// #else
// #define STAG_API __attribute__((visibility("default")))
// #endif

// // —————————————————————————————————————————
// // 1) Camera intrinsics & fisheye distortion (double precision)
// //    NOTE: DIST_COEFFS are fisheye (k1,k2,k3,k4).
// // —————————————————————————————————————————
// static const Mat CAMERA_MATRIX = (Mat_<double>(3, 3) <<
//     5.53994720e+02, 4.07947431e-01, 1298.0,
//     0.00000000e+00, 5.54078139e+02, 1024.0,
//     0.00000000e+00, 0.00000000e+00, 1.00000000e+00
//     );

// static const Mat DIST_COEFFS = (Mat_<double>(1, 4) <<
//     0.02025687,   // k1 (fisheye)
//     0.00113570,   // k2 (fisheye)
//     0.00332130,   // k3 (fisheye)
//     -0.00216723    // k4 (fisheye)
//     );

// // —————————————————————————————————————————
// // 2) Tag edge length (mm)
// // —————————————————————————————————————————
// static const float FIDUCIAL_EDGE_LENGTH_MM = 89.0f;

// // —————————————————————————————————————————
// // 3) Undistort-map buffers (fisheye) + Knew
// // —————————————————————————————————————————
// static Mat map1, map2;
// static Mat Knew;              // intrinsics for the UNDISTORTED image
// static Matx33d KnewInv;       // inverse of Knew
// static Size mapSize;
// static bool mapReady = false;

// // Balance: 0 keeps a tighter FOV (less blank), 1 preserves full FOV
// static double kBalance = 1.0;

// static void ensureUndistortMap(const Size& sz) {
//     if (mapReady && mapSize == sz) return;

//     Mat R = Mat::eye(3, 3, CV_64F);

//     // Compute Knew for the undistorted (pinhole) view
//     fisheye::estimateNewCameraMatrixForUndistortRectify(
//         CAMERA_MATRIX, DIST_COEFFS, sz, R, Knew, kBalance
//     );

//     // Build undistort/rectify maps using Knew
//     fisheye::initUndistortRectifyMap(
//         CAMERA_MATRIX, DIST_COEFFS, R, Knew, sz, CV_32FC1, map1, map2
//     );

//     // Precompute Knew inverse
//     const double fx = Knew.at<double>(0, 0);
//     const double fy = Knew.at<double>(1, 1);
//     const double cx = Knew.at<double>(0, 2);
//     const double cy = Knew.at<double>(1, 2);

//     KnewInv = Matx33d(
//         1.0 / fx, 0.0, -cx / fx,
//         0.0, 1.0 / fy, -cy / fy,
//         0.0, 0.0, 1.0
//     );

//     mapSize = sz;
//     mapReady = true;
// }

// // —————————————————————————————————————————
// // 4) Main wrapper
// //    Contract: 'image' is an 8-bit single-channel (grayscale) buffer.
// //    result layout:
// //      result[0]           = num detected (0–2)
// //      result[1..8]        = RAW/DISTORTED XY corners of tag0
// //      result[9..16]       = RAW/DISTORTED XY corners of tag1
// //      result[17..19]      = X,Y,Z (mm) of tag0 (solvePnP on undistorted image)
// //      result[20..22]      = X,Y,Z (mm) of tag1
// // —————————————————————————————————————————
// extern "C" STAG_API void FindStagCorners(
//     unsigned char* image,   // grayscale (8UC1)
//     int width, int height,
//     double* result,
//     int resultLen,
//     int error_correction
// ) {
//     // We will write up to index 22.
//     const int kMinLen = 23;
//     if (!result || resultLen < kMinLen || !image || width <= 0 || height <= 0) {
//         return;
//     }

//     // Clear output buffer
//     std::fill(result, result + resultLen, 0.0);

//     // Wrap input (caller must supply grayscale 8UC1)
//     Mat imgDistorted(height, width, CV_8UC1, image);

//     // 1) Undistort to pinhole image using fisheye model, with Knew
//     ensureUndistortMap(imgDistorted.size());
//     Mat imgU; // undistorted image
//     remap(imgDistorted, imgU, map1, map2, INTER_LINEAR);

//     // 2) Detect up to 2 STAG markers on the undistorted image
//     std::vector<std::vector<Point2f>> corners_u; // undistorted pixel coords
//     std::vector<int> ids;
//     std::vector<std::vector<Point2f>> rejected;
//     detectMarkers(imgU, 23, corners_u, ids, error_correction, rejected);

//     const int num = std::min<int>(corners_u.size(), 2);
//     result[0] = num;

//     // 3) Map undistorted-image corners back to ORIGINAL (distorted) pixels
//     //    via fisheye::distortPoints. (Normalize with Knew, then distort.)
//     for (int m = 0; m < num; ++m) {
//         // a) undistorted pixel -> normalized using KnewInv
//         std::vector<Point2d> undist_norm;
//         undist_norm.reserve(4);
//         for (int k = 0; k < 4; ++k) {
//             const Point2f& pu = corners_u[m][k];
//             Vec3d hn = KnewInv * Vec3d(pu.x, pu.y, 1.0);
//             undist_norm.emplace_back(hn[0] / hn[2], hn[1] / hn[2]);
//         }

//         // b) distort normalized points back to RAW pixels using original K,D
//         std::vector<Point2d> distorted_px;
//         fisheye::distortPoints(undist_norm, distorted_px, CAMERA_MATRIX, DIST_COEFFS);

//         // Write into result[1..16]
//         const int base = 1 + m * 8;
//         if (base + 7 < resultLen) {
//             for (int k = 0; k < 4; ++k) {
//                 result[base + 2 * k] = distorted_px[k].x;
//                 result[base + 2 * k + 1] = distorted_px[k].y;
//             }
//         }
//     }

//     //// 4) 3D object points for square tag centered at origin on z=0
//     const float s2 = FIDUCIAL_EDGE_LENGTH_MM * 0.5f;
//     std::vector<Point3f> objPts = {
//         {-s2, -s2, 0.0f},
//         { s2, -s2, 0.0f},
//         { s2,  s2, 0.0f},
//         {-s2,  s2, 0.0f}
//     };

//     // 5) Pose on the undistorted image with zero distortion (we already rectified)
//     Mat zeroDist; // empty is fine (equivalent to zeros)
//     for (int m = 0; m < num; ++m) {
//        Mat rvec, tvec;
//        bool ok = solvePnP(
//            objPts,
//            corners_u[m],   // undistorted pixel corners
//            Knew,
//            zeroDist,
//            rvec, tvec,
//            false,
//            SOLVEPNP_IPPE_SQUARE
//        );
//         if (ok) {
//            const int pbase = 17 + m * 3;
//            if (pbase + 2 < resultLen) {
//                result[pbase + 0] = tvec.at<double>(0);
//                result[pbase + 1] = tvec.at<double>(1);
//                result[pbase + 2] = tvec.at<double>(2);
//            }

//            // ——— Sanity print: Z ≈ fx_new * S_mm / p ———
//            // Estimate mean edge length in pixels on the undistorted image.
//            auto edgeLen = [&](const Point2f& a, const Point2f& b) {
//                const double dx = double(b.x) - double(a.x);
//                const double dy = double(b.y) - double(a.y);
//                return std::sqrt(dx * dx + dy * dy);
//                };
//            const std::vector<Point2f>& C = corners_u[m];
//            if (C.size() == 4) {
//                double p0 = edgeLen(C[0], C[1]);
//                double p1 = edgeLen(C[1], C[2]);
//                double p2 = edgeLen(C[2], C[3]);
//                double p3 = edgeLen(C[3], C[0]);
//                double p = 0.25 * (p0 + p1 + p2 + p3);

//                const double fx_new = Knew.at<double>(0, 0);
//                const double Z_est = (fx_new * double(FIDUCIAL_EDGE_LENGTH_MM)) / std::max(1e-6, p);

//                std::cerr << "[STAG] tag#" << m
//                    << " mean_px=" << p
//                    << " fx_new=" << fx_new
//                    << " Z_solvePnP=" << tvec.at<double>(2)
//                    << " Z_est≈" << Z_est
//                    << " (mm)\n";
//            }
//        }
//     }
// }









// stag_wrapper.cpp

#include "pch.h"
#include "src/Stag.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <algorithm>
#include <iostream>

using namespace cv;
using namespace stag;

#ifdef _WIN32
#define STAG_API __declspec(dllexport)
#else
#define STAG_API __attribute__((visibility("default")))
#endif

// —————————————————————————————————————————
// 1) Camera intrinsics & fisheye distortion (double precision)
//    NOTE: DIST_COEFFS are fisheye (k1,k2,k3,k4).
// —————————————————————————————————————————
static const Mat CAMERA_MATRIX = (Mat_<double>(3, 3) <<
    5.53994720e+02, 4.07947431e-01, 1352.35985,
    0.00000000e+00, 5.54078139e+02, 1031.45909,
    0.00000000e+00, 0.00000000e+00, 1.00000000e+00
);
// static const Mat CAMERA_MATRIX = (Mat_<double>(3, 3) <<
//     5.53994720e+02, 4.07947431e-01, 1292.0,
//     0.00000000e+00, 5.54078139e+02, 1024.0,
//     0.00000000e+00, 0.00000000e+00, 1.00000000e+00
// );

static const Mat DIST_COEFFS = (Mat_<double>(1, 4) <<
    0.02025687,   // k1 (fisheye)
    0.00113570,   // k2 (fisheye)
    0.00332130,   // k3 (fisheye)
   -0.00216723    // k4 (fisheye)
);

// —————————————————————————————————————————
// 2) Tag edge length (mm)
// —————————————————————————————————————————
static const float FIDUCIAL_EDGE_LENGTH_MM = 69.0f;

// —————————————————————————————————————————
// 3) Knew (undistorted intrinsics), cached per image size
// —————————————————————————————————————————
static Mat Knew;              // intrinsics for the UNDISTORTED image
static Size cachedSize;
static bool knewReady = false;
static double kBalance = 0.0; // 0 = tighter FOV, 1 = full FOV

static void ensureKnew(const Size& sz)
{
    if (knewReady && cachedSize == sz)
        return;

    Mat R = Mat::eye(3, 3, CV_64F);
    fisheye::estimateNewCameraMatrixForUndistortRectify(
        CAMERA_MATRIX, DIST_COEFFS, sz, R, Knew, kBalance
    );

    cachedSize = sz;
    knewReady  = true;
}

// —————————————————————————————————————————
// 4) Main wrapper
//    Contract: 'image' is an 8-bit single-channel (grayscale) buffer.
//    result layout:
//      result[0]           = num detected (0–2)
//      result[1..8]        = DISTORTED XY corners of tag0 (as detected)
//      result[9..16]       = DISTORTED XY corners of tag1
//      result[17..19]      = X,Y,Z (mm) of tag0  (solvePnP P3P on UNDISTORTED points, Knew, zero distortion)
//      result[20..22]      = X,Y,Z (mm) of tag1
// —————————————————————————————————————————
extern "C" STAG_API void FindStagCorners(
    unsigned char* image,   // grayscale (8UC1)
    int width, int height,
    double* result,
    int resultLen,
    int error_correction
) {
    const int kMinLen = 23;
    if (!result || resultLen < kMinLen || !image || width <= 0 || height <= 0) {
        return;
    }

    // Clear output buffer
    std::fill(result, result + resultLen, 0.0);

    // Wrap input (caller must supply grayscale 8UC1)
    Mat imgDistorted(height, width, CV_8UC1, image);

    // Prepare Knew for current size
    ensureKnew(imgDistorted.size());

    // 1) Detect STAG markers *on the distorted image*
    std::vector<std::vector<Point2f>> corners_d; // distorted pixel coords
    std::vector<int> ids;
    std::vector<std::vector<Point2f>> rejected;
    detectMarkers(imgDistorted, 23, corners_d, ids, error_correction, rejected);

    const int num = std::min<int>(corners_d.size(), 2);
    result[0] = num;

    // 2) Write out distorted corners, and compute undistorted corners for PnP
    //    We'll undistort *only* the detected corners to the Knew pixel domain.
    std::vector<std::vector<Point2f>> corners_u; // undistorted pixel coords (Knew domain)
    corners_u.resize(num);

    for (int m = 0; m < num; ++m) {
        // 2a) Save distorted pixels to result buffer
        const int base = 1 + m * 8;
        for (int k = 0; k < 4; ++k) {
            if (base + 2 * k + 1 < resultLen) {
                result[base + 2 * k    ] = corners_d[m][k].x;
                result[base + 2 * k + 1] = corners_d[m][k].y;
            }
        }

        // 2b) Undistort these specific points to *undistorted pixel* coords
        //     using fisheye::undistortPoints with P = Knew.
        std::vector<Point2f> udst = corners_d[m];
        std::vector<Point2f> uout;
        Mat R = Mat::eye(3, 3, CV_64F);
        fisheye::undistortPoints(udst, uout, CAMERA_MATRIX, DIST_COEFFS, R, Knew);

        corners_u[m] = std::move(uout);
    }

    // 3) 3D object points for square tag centered at origin on z=0
    const float s2 = FIDUCIAL_EDGE_LENGTH_MM * 0.5f;
    std::vector<Point3f> objPts = {
        {-s2, -s2, 0.0f},
        { s2, -s2, 0.0f},
        { s2,  s2, 0.0f},
        {-s2,  s2, 0.0f}
    };

    // 4) Pose via P3P on the UNDISTORTED points (Knew, zero distortion)
    Mat zeroDist; // empty => zeros
    for (int m = 0; m < num; ++m) {
        // P3P expects at least 4 points in OpenCV's implementation
        Mat rvec, tvec;
        bool ok = solvePnP(
            objPts,
            corners_u[m],   // undistorted pixel corners (Knew domain)
            Knew,
            zeroDist,
            rvec, tvec,
            false,
            SOLVEPNP_P3P
        );

        if (ok) {
            const int pbase = 17 + m * 3;
            if (pbase + 2 < resultLen) {
                result[pbase + 0] = tvec.at<double>(0);
                result[pbase + 1] = tvec.at<double>(1);
                result[pbase + 2] = tvec.at<double>(2);
            }

            // ——— Quick sanity: Z ≈ fx_new * S_mm / mean_edge_px ———
            auto edgeLen = [](const Point2f& a, const Point2f& b) {
                const double dx = double(b.x) - double(a.x);
                const double dy = double(b.y) - double(a.y);
                return std::sqrt(dx * dx + dy * dy);
            };
            const std::vector<Point2f>& C = corners_u[m];
            if (C.size() == 4) {
                const double p0 = edgeLen(C[0], C[1]);
                const double p1 = edgeLen(C[1], C[2]);
                const double p2 = edgeLen(C[2], C[3]);
                const double p3 = edgeLen(C[3], C[0]);
                const double p  = 0.25 * (p0 + p1 + p2 + p3);

                const double fx_new = Knew.at<double>(0, 0);
                const double Z_est  = (fx_new * double(FIDUCIAL_EDGE_LENGTH_MM)) / std::max(1e-6, p);

                std::cerr << "[STAG] tag#" << m
                          << " mean_px=" << p
                          << " fx_new=" << fx_new
                          << " Z_solvePnP=" << tvec.at<double>(2)
                          << " Z_est≈" << Z_est
                          << " (mm)\n";
            }
        }
    }
}
