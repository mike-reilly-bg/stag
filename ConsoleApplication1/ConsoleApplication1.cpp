#include "Stag.h"
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <windows.h>
using cv::Mat;

typedef int(__cdecl* FindStagCorners_t)(const unsigned char*, int, int, double*, int);

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: StagTestHarness <imagefile>" << std::endl;
        return 1;
    }

    // Load image as grayscale
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    if (img.empty())
    {
        std::cerr << "Failed to load image: " << argv[1] << std::endl;
        return 1;
    }
    int width = img.cols;
    int height = img.rows;

    // Load the DLL
    HMODULE hDll = LoadLibraryA("stagdll.dll");
    if (!hDll)
    {
        std::cerr << "Could not load stagdll.dll" << std::endl;
        return 1;
    }

    // Get function pointer
    auto FindStagCorners = (FindStagCorners_t)GetProcAddress(hDll, "FindStagCorners");
    if (!FindStagCorners)
    {
        std::cerr << "Could not find FindStagCorners in stagdll.dll" << std::endl;
        return 1;
    }

    // Prepare buffers
    int maxMarkers = 10;
    std::vector<double> outCorners(maxMarkers * 8);

    // Call the function
    int numMarkers = FindStagCorners(img.data, width, height, outCorners.data(), maxMarkers);

    std::cout << "Number of markers detected: " << numMarkers << std::endl;
    for (int i = 0; i < numMarkers; ++i)
    {
        std::cout << "Marker " << i << " corners: ";
        for (int k = 0; k < 4; ++k)
        {
            double x = outCorners[i * 8 + k * 2 + 0];
            double y = outCorners[i * 8 + k * 2 + 1];
            std::cout << "(" << x << "," << y << ") ";
        }
        std::cout << std::endl;
    }

    FreeLibrary(hDll);
    return 0;
}
