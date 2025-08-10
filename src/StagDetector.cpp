#include "StagDetector.h"
#include "Ellipse.h"
#include "utility.h"
#include <iostream>

#define HALF_PI 1.570796326794897
#define DEBUG 0

using cv::Mat;
using cv::Point2d;

StagDetector::StagDetector(int libraryHD, int inErrorCorrection)
{
	errorCorrection = inErrorCorrection;

	quadDetector = QuadDetector();
	fillCodeLocations();
	decoder = Decoder(libraryHD);
}

const std::vector<Marker>& StagDetector::getMarkers() {
	return markers;
}

const std::vector<Quad>& StagDetector::getFalseCandidates() {
    return falseCandidates;
}

void StagDetector::detectMarkers(const Mat& inImage)
{
	image = inImage;
	quadDetector.detectQuads(image, &edInterface);

	vector<Quad> quads = quadDetector.getQuads();

	for (auto & quad : quads)
	{
		quad.estimateHomography();
#if DEBUG
		for (int i = 0; i < 4; ++i) {
			cv::circle(image, quad.corners[i], 5, cv::Scalar(255), -1);  // Draw corner
			cv::putText(image, std::to_string(i), quad.corners[i] + cv::Point2d(5, 5),
				cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
		}
		cv::imwrite("quad_corners.png", image);
#endif
		Codeword c = readCode(quad);
		int shift;
		int id;
		if (decoder.decode(c, errorCorrection, id, shift))
		{
			Marker marker(quad, id);
			marker.shiftCorners2(shift);

			// only add marker if similar not already found
			if (!marker.isSimilarIn(markers))
			{
				markers.push_back(marker);
			}
		}
		else
			falseCandidates.push_back(quad);
			//std::cout << c;
			//std::cout << "fin";
			//std::cout << "\n";
	}

	for (auto & marker : markers)
		poseRefiner.refineMarkerPose(&edInterface, marker);
}


void StagDetector::logResults(const string& path)
{
#if DEBUG
	drawer.drawEdgeMap(path + "1_edges.png", image, edInterface.getEdgeMap());
	drawer.drawLines(path + "2_lines.png", image, edInterface.getEDLines());
	drawer.drawCorners(path + "3_corners.png", image, quadDetector.getCornerGroups());
	drawer.drawQuads(path + "4_quads.png", image, quadDetector.getQuads());
    drawer.drawQuads(path + "5_distorted_quads.png", image, quadDetector.getDistortedQuads());
	drawer.drawMarkers(path + "6_markers.png", image, markers);
    drawer.drawQuads(path + "7_false_quads.png", image, falseCandidates);
	drawer.drawEllipses(path + "8_ellipses.png", image, markers);
#endif
}


Codeword StagDetector::readCode(const Quad &q)
{
	// take readings from 48 code locations, 12 black border locations, and 12 white border locations
	vector<unsigned char> samples(72);

	// a better idea may be creating a list of points to be sampled and let the OpenCV's interpolation function handle the sampling
	for (int i = 0; i < 48; i++)
	{
		Mat projectedPoint = q.H * codeLocs[i];
		//std::cout << projectedPoint << "\n";
		samples[i] = readPixelSafeBilinear(image, Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2)));
		//std::cout << static_cast<int>(samples[i]) << "\n";
		//printf("%d", (int)samples[i]);
		//std::cout << "\n";
	}
	for (int i = 0; i < 12; i++)
	{
		Mat projectedPoint = q.H * blackLocs[i];
		samples[i + 48] = readPixelSafeBilinear(image, Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2)));
	}
	for (int i = 0; i < 12; i++)
	{
		Mat projectedPoint = q.H * whiteLocs[i];
		samples[i + 60] = readPixelSafeBilinear(image, Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2)));
	}

#if DEBUG
	for (int i = 0; i < 48; i++) {
		Mat projectedPoint = q.H * codeLocs[i];
		double x = projectedPoint.at<double>(0) / projectedPoint.at<double>(2);
		double y = projectedPoint.at<double>(1) / projectedPoint.at<double>(2);
		cv::circle(image, cv::Point2d(x, y), 2, cv::Scalar(255), -1);  // White dots
	}cv::imwrite("code_sample_points.png", image);


	for (double u = 0.1; u < 1.0; u += 0.2) {
		for (double v = 0.1; v < 1.0; v += 0.2) {
			cv::Mat pt = (cv::Mat_<double>(3, 1) << u, v, 1);
			Mat proj = q.H * pt;
			double x = proj.at<double>(0) / proj.at<double>(2);
			double y = proj.at<double>(1) / proj.at<double>(2);
			cv::circle(image, cv::Point2d(x, y), 2, cv::Scalar(127), -1);  // Gray grid dots
		}
	}
	cv::imwrite("homography_grid_overlay.png", image);
#endif



	//std::cout << "samples\n";
	//for (int i = 0; i < 48; i++)
	//	std::cout << samples[i] / 255;
	//	std::cout << "\n";
	//std::cout << "endsamples\n";

	// threshold the readings using Otsu's method
	cv::threshold(samples, samples, 0, 255, cv::THRESH_OTSU + cv::THRESH_BINARY_INV);

	// create a codeword using the thresholded readings
	Codeword c;
	for (int i = 0; i < 48; i++)
		c[i] = samples[i] / 255;

	return c;
}

void StagDetector::fillCodeLocations()
{
	// fill coordinates to be sampled
	codeLocs = vector<Mat>(48);

	// code circles are located in a circle with radius outerCircleRadius
	double outerCircleRadius = 0.4;
	double innerCircleRadius = outerCircleRadius * 0.9;

	// each quadrant is rotated by HALF_PI
	// these part is left as is for self-documenting purposes
	for (int i = 0; i < 4; i++)
	{
		codeLocs[0 + i * 12] = createMatFromPolarCoords(0.088363142525988, 0.785398163397448 + i * HALF_PI, innerCircleRadius);

		codeLocs[1 + i * 12] = createMatFromPolarCoords(0.206935928182607, 0.459275804122858 + i * HALF_PI, innerCircleRadius);
		codeLocs[2 + i * 12] = createMatFromPolarCoords(0.206935928182607, HALF_PI - 0.459275804122858 + i * HALF_PI, innerCircleRadius);

		codeLocs[3 + i * 12] = createMatFromPolarCoords(0.313672146827381, 0.200579720495241 + i * HALF_PI, innerCircleRadius);
		codeLocs[4 + i * 12] = createMatFromPolarCoords(0.327493143484516, 0.591687617505840 + i * HALF_PI, innerCircleRadius);
		codeLocs[5 + i * 12] = createMatFromPolarCoords(0.327493143484516, HALF_PI - 0.591687617505840 + i * HALF_PI, innerCircleRadius);
		codeLocs[6 + i * 12] = createMatFromPolarCoords(0.313672146827381, HALF_PI - 0.200579720495241 + i * HALF_PI, innerCircleRadius);

		codeLocs[7 + i * 12] = createMatFromPolarCoords(0.437421957035861, 0.145724938287167 + i * HALF_PI, innerCircleRadius);
		codeLocs[8 + i * 12] = createMatFromPolarCoords(0.437226762361658, 0.433363129825345 + i * HALF_PI, innerCircleRadius);
		codeLocs[9 + i * 12] = createMatFromPolarCoords(0.430628029742607, 0.785398163397448 + i * HALF_PI, innerCircleRadius);
		codeLocs[10 + i * 12] = createMatFromPolarCoords(0.437226762361658, HALF_PI - 0.433363129825345 + i * HALF_PI, innerCircleRadius);
		codeLocs[11 + i * 12] = createMatFromPolarCoords(0.437421957035861, HALF_PI - 0.145724938287167 + i * HALF_PI, innerCircleRadius);
	}

	double borderDist = 0.045;

	blackLocs = vector<Mat>(12);
	whiteLocs = vector<Mat>(12);

	for (int i = 0; i < 12; i++)
		blackLocs[i] = Mat(3, 1, CV_64FC1);
	for (int i = 0; i < 12; i++)
		whiteLocs[i] = Mat(3, 1, CV_64FC1);

	blackLocs[0].at<double>(0) = borderDist;
	blackLocs[0].at<double>(1) = borderDist * 3;
	blackLocs[0].at<double>(2) = 1;

	blackLocs[1].at<double>(0) = borderDist * 2;
	blackLocs[1].at<double>(1) = borderDist * 2;
	blackLocs[1].at<double>(2) = 1;

	blackLocs[2].at<double>(0) = borderDist * 3;
	blackLocs[2].at<double>(1) = borderDist;
	blackLocs[2].at<double>(2) = 1;

	blackLocs[3].at<double>(0) = 1 - 3 * borderDist;
	blackLocs[3].at<double>(1) = borderDist;
	blackLocs[3].at<double>(2) = 1;

	blackLocs[4].at<double>(0) = 1 - 2 * borderDist;
	blackLocs[4].at<double>(1) = borderDist * 2;
	blackLocs[4].at<double>(2) = 1;

	blackLocs[5].at<double>(0) = 1 - borderDist;
	blackLocs[5].at<double>(1) = borderDist * 3;
	blackLocs[5].at<double>(2) = 1;

	blackLocs[6].at<double>(0) = 1 - borderDist;
	blackLocs[6].at<double>(1) = 1 - 3 * borderDist;
	blackLocs[6].at<double>(2) = 1;

	blackLocs[7].at<double>(0) = 1 - 2 * borderDist;
	blackLocs[7].at<double>(1) = 1 - 2 * borderDist;
	blackLocs[7].at<double>(2) = 1;

	blackLocs[8].at<double>(0) = 1 - 3 * borderDist;
	blackLocs[8].at<double>(1) = 1 - borderDist;
	blackLocs[8].at<double>(2) = 1;

	blackLocs[9].at<double>(0) = borderDist * 3;
	blackLocs[9].at<double>(1) = 1 - borderDist;
	blackLocs[9].at<double>(2) = 1;

	blackLocs[10].at<double>(0) = borderDist * 2;
	blackLocs[10].at<double>(1) = 1 - 2 * borderDist;
	blackLocs[10].at<double>(2) = 1;

	blackLocs[11].at<double>(0) = borderDist;
	blackLocs[11].at<double>(1) = 1 - 3 * borderDist;
	blackLocs[11].at<double>(2) = 1;


	whiteLocs[0].at<double>(0) = 0.25;
	whiteLocs[0].at<double>(1) = -borderDist;
	whiteLocs[0].at<double>(2) = 1;

	whiteLocs[1].at<double>(0) = 0.5;
	whiteLocs[1].at<double>(1) = -borderDist;
	whiteLocs[1].at<double>(2) = 1;

	whiteLocs[2].at<double>(0) = 0.75;
	whiteLocs[2].at<double>(1) = -borderDist;
	whiteLocs[2].at<double>(2) = 1;

	whiteLocs[3].at<double>(0) = 1 + borderDist;
	whiteLocs[3].at<double>(1) = 0.25;
	whiteLocs[3].at<double>(2) = 1;

	whiteLocs[4].at<double>(0) = 1 + borderDist;
	whiteLocs[4].at<double>(1) = 0.5;
	whiteLocs[4].at<double>(2) = 1;

	whiteLocs[5].at<double>(0) = 1 + borderDist;
	whiteLocs[5].at<double>(1) = 0.75;
	whiteLocs[5].at<double>(2) = 1;

	whiteLocs[6].at<double>(0) = 0.75;
	whiteLocs[6].at<double>(1) = 1 + borderDist;
	whiteLocs[6].at<double>(2) = 1;

	whiteLocs[7].at<double>(0) = 0.5;
	whiteLocs[7].at<double>(1) = 1 + borderDist;
	whiteLocs[7].at<double>(2) = 1;

	whiteLocs[8].at<double>(0) = 0.25;
	whiteLocs[8].at<double>(1) = 1 + borderDist;
	whiteLocs[8].at<double>(2) = 1;

	whiteLocs[9].at<double>(0) = -borderDist;
	whiteLocs[9].at<double>(1) = 0.75;
	whiteLocs[9].at<double>(2) = 1;

	whiteLocs[10].at<double>(0) = -borderDist;
	whiteLocs[10].at<double>(1) = 0.5;
	whiteLocs[10].at<double>(2) = 1;

	whiteLocs[11].at<double>(0) = -borderDist;
	whiteLocs[11].at<double>(1) = 0.25;
	whiteLocs[11].at<double>(2) = 1;
}


Mat StagDetector::createMatFromPolarCoords(double radius, double radians, double circleRadius)
{
	Mat point(3, 1, CV_64FC1);
	point.at<double>(0) = 0.5 + cos(radians) * radius * (circleRadius / 0.5);
	point.at<double>(1) = 0.5 - sin(radians) * radius * (circleRadius / 0.5);
	point.at<double>(2) = 1;
	return point;
}