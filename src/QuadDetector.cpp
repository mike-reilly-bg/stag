// QuadDetector.cpp

#include <vector>
#include <algorithm>
#include "opencv2/opencv.hpp"
#include <opencv2/flann.hpp>
#include "QuadDetector.h"
#include "utility.h"
#include <random>

using cv::Point2d;


struct MergedLine {
	cv::Point2d start, end;
};

void drawMergedLines(
	const std::vector<MergedLine>& merged,
	const std::vector<std::vector<int>>& lineGroups,
	const EDLines* edLines,
	cv::Mat& image)
{	

	if (image.channels() == 1)
		cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
	// First draw original segments in white
	for (const auto& group : lineGroups) {
		for (int idx : group) {
			const auto& l = edLines->lines[idx];
			cv::line(image, cv::Point2d(l.sx, l.sy), cv::Point2d(l.ex, l.ey), cv::Scalar(255, 255, 255), 1); // White
		}
	}

	// Then draw merged lines in red
	for (const auto& line : merged) {
		cv::line(image, line.start, line.end, cv::Scalar(0, 0, 255), 2); // Red
	}
}



void drawQuadsColored(const std::vector<Quad>& quads, cv::Mat& image)
{
	if (image.channels() == 1)
		cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
	std::mt19937 rng(54321);
	std::uniform_int_distribution<int> colorDist(80, 255);

	for (const auto& q : quads) {
		cv::Scalar color(colorDist(rng), colorDist(rng), colorDist(rng));
		for (int i = 0; i < 4; ++i) {
			const cv::Point2d& p1 = q.corners[i];
			const cv::Point2d& p2 = q.corners[(i + 1) % 4];
			cv::line(image, p1, p2, color, 2);
		}
	}
}




std::vector<MergedLine> computeMergedLines(const std::vector<std::vector<int>>& lineGroups, const EDLines* edLines, double angleThresholdDeg = 20.0) {
	std::vector<MergedLine> mergedLines;

	for (const auto& group : lineGroups) {
		if (group.size() < 2) continue;

		std::vector<cv::Point2d> mergedPoints;

		// Start with first line
		int prevIdx = group[0];
		const LineSegment* prev = &edLines->lines[prevIdx];
		cv::Point2d prevVec(prev->ex - prev->sx, prev->ey - prev->sy);
		mergedPoints.push_back(cv::Point2d(prev->sx, prev->sy));
		mergedPoints.push_back(cv::Point2d(prev->ex, prev->ey));

		for (size_t i = 1; i < group.size(); ++i) {
			int currIdx = group[i];
			const LineSegment* curr = &edLines->lines[currIdx];
			cv::Point2d currVec(curr->ex - curr->sx, curr->ey - curr->sy);

			double dot = prevVec.x * currVec.x + prevVec.y * currVec.y;
			double mag1 = cv::norm(prevVec);
			double mag2 = cv::norm(currVec);

			if (mag1 == 0 || mag2 == 0)
				continue;

			double angleDeg = std::acos(dot / (mag1 * mag2)) * 180.0 / CV_PI;

			if (std::abs(angleDeg) < angleThresholdDeg || std::abs(angleDeg - 180.0) < angleThresholdDeg) {
				// Merge: accumulate endpoints
				mergedPoints.push_back(cv::Point2d(curr->sx, curr->sy));
				mergedPoints.push_back(cv::Point2d(curr->ex, curr->ey));
			}
			else {
				// Finish current merged line
				cv::Point2d minPt = *std::min_element(mergedPoints.begin(), mergedPoints.end(), [](const auto& a, const auto& b) {
					return a.x + a.y < b.x + b.y;
					});
				cv::Point2d maxPt = *std::max_element(mergedPoints.begin(), mergedPoints.end(), [](const auto& a, const auto& b) {
					return a.x + a.y < b.x + b.y;
					});
				mergedLines.push_back({ minPt, maxPt });
				mergedPoints.clear();

				// Start new sequence
				mergedPoints.push_back(cv::Point2d(curr->sx, curr->sy));
				mergedPoints.push_back(cv::Point2d(curr->ex, curr->ey));
			}
			prevVec = currVec;
		}

		// Final flush
		if (mergedPoints.size() >= 2) {
			cv::Point2d minPt = *std::min_element(mergedPoints.begin(), mergedPoints.end(), [](const auto& a, const auto& b) {
				return a.x + a.y < b.x + b.y;
				});
			cv::Point2d maxPt = *std::max_element(mergedPoints.begin(), mergedPoints.end(), [](const auto& a, const auto& b) {
				return a.x + a.y < b.x + b.y;
				});
			mergedLines.push_back({ minPt, maxPt });
		}
	}

	return mergedLines;
}




void mergeNearbyCorners(std::vector<std::vector<Corner>>& cornerGroups, double mergeThreshold = 10.0) {
	std::vector<cv::Point2f> allPoints;
	std::vector<Corner> allCorners;

	// Flatten all corner groups
	for (const auto& group : cornerGroups) {
		for (const auto& c : group) {
			allPoints.emplace_back(static_cast<cv::Point2f>(c.loc));  // Ensure Point2f
			allCorners.push_back(c);
		}
	}

	if (allPoints.empty()) return;

	// Convert points to a 2D float matrix
	cv::Mat pointsMat(static_cast<int>(allPoints.size()), 2, CV_32F);
	for (size_t i = 0; i < allPoints.size(); ++i) {
		pointsMat.at<float>(static_cast<int>(i), 0) = allPoints[i].x;
		pointsMat.at<float>(static_cast<int>(i), 1) = allPoints[i].y;
	}

	// Create FLANN index
	cv::flann::KDTreeIndexParams indexParams(1);
	cv::flann::Index kdtree(pointsMat, indexParams);

	std::vector<bool> used(allCorners.size(), false);
	std::vector<std::vector<Corner>> mergedGroups;

	for (size_t i = 0; i < allCorners.size(); ++i) {
		if (used[i]) continue;

		std::vector<int> indices;
		std::vector<float> dists;

		cv::Mat query = (cv::Mat_<float>(1, 2) << allPoints[i].x, allPoints[i].y);

		// radiusSearch requires squared radius
		kdtree.radiusSearch(query, indices, dists, mergeThreshold * mergeThreshold, static_cast<int>(allCorners.size()));

		std::vector<Corner> group;
		for (int idx : indices) {
			if (!used[idx]) {
				group.push_back(allCorners[idx]);
				used[idx] = true;
			}
		}

		if (group.size() >= 3)  // Or 2 if needed
			mergedGroups.push_back(group);
	}

	cornerGroups = mergedGroups;
}


QuadDetector::QuadDetector() = default;


void QuadDetector::detectQuads(const cv::Mat &image, EDInterface* edInterface)
{
	cornerGroups.clear();
	distortedQuads.clear();
	quads.clear();

	edInterface->runEDPFandEDLines(image);

	EDLines* edLines = edInterface->getEDLines();

	vector<vector<int>> lineGroups = groupLines(image, edInterface);

	cv::Mat debugImage = image.clone();  // Make a modifiable copy

	std::vector<MergedLine> mergedLines = computeMergedLines(lineGroups, edLines, 20.0);
	drawMergedLines(mergedLines, lineGroups, edLines, debugImage);
	cv::imwrite("merged_lines.png", debugImage);
	
	detectCorners(edInterface, lineGroups);
	mergeNearbyCorners(cornerGroups, /* mergeThreshold= */ 10.0);


	cv::Mat quadCornersImage = image.clone();
	for (int i = 0; i < edLines->noLines; ++i) {
		auto& l = edLines->lines[i];
		cv::line(quadCornersImage, cv::Point2d(l.sx, l.sy), cv::Point2d(l.ex, l.ey), cv::Scalar(100), 1);
	}

	// After cornerGroups are filled:
	for (auto& group : cornerGroups)
		for (auto& corner : group)
			cv::circle(quadCornersImage, corner.loc, 3, cv::Scalar(255), -1);
	cv::imwrite("quad_corners2.png", quadCornersImage);

	// create quads using corner groups
	for (int indCornerGroup = 0; indCornerGroup < cornerGroups.size(); indCornerGroup++)
	{
		// assumed that at least 3 corners are needed. actually, we need at least two opposite corners.
		// however, it is assumed that there is an additional corner between opposite corners, hence the need for 3 corners
		if (cornerGroups[indCornerGroup].size() < 3)
			continue;

		vector<Corner> currCornerGroup = cornerGroups[indCornerGroup];

		for (unsigned int cornerInd = 0; cornerInd < currCornerGroup.size(); cornerInd++)
		{
			int i1 = cornerInd, i2 = (i1 + 1) % currCornerGroup.size(), i3 = (i1 + 2) % currCornerGroup.size(), i4 = (i1 + 3) % currCornerGroup.size();

			Corner c1 = currCornerGroup[i1], c2 = currCornerGroup[i2], c3 = currCornerGroup[i3], c4 = currCornerGroup[i4];

			// if there are only 3 corners, replace the 4th corner with an invalid one
			if (i1 == i4)
				c4 = Corner(Point2d(INFINITY, INFINITY), LineSegment(), LineSegment());

			vector<Corner> corners = { c1, c2, c3, c4 };

			if (!checkIfCornersFormQuad(corners, edInterface))
				continue;

			vector<Point2d> cornerLocs = { corners[0].loc, corners[1].loc, corners[2].loc, corners[3].loc };

			Quad quad(cornerLocs);

			// eliminate if projective distortion is larger than the threshold
			/*std::cout << "Quad distortion: " << quad.projectiveDistortion << std::endl;
			if (quad.projectiveDistortion > thresProjectiveDistortion)
				std::cout << "Quad rejected: distortion " << quad.projectiveDistortion << "\n";
			if (quad.projectiveDistortion > thresProjectiveDistortion)
			{
                distortedQuads.push_back(quad);
			}
			else {
                quads.push_back(quad);
            }*/
			quads.push_back(quad);
		}
	}

	cv::Mat quadImage = image.clone();
	drawQuadsColored(quads, quadImage);
	cv::imwrite("quads_colored.png", quadImage);
}


const vector<vector<Corner>>& QuadDetector::getCornerGroups()
{
	return cornerGroups;
}


const vector<Quad>& QuadDetector::getQuads() const
{
	return quads;
}


const vector<Quad>& QuadDetector::getDistortedQuads() const
const std::vector<Quad>& QuadDetector::getDistortedQuads() const
{
	return distortedQuads;
}


vector<vector<int>> QuadDetector::groupLines(const cv::Mat &image, EDInterface* edInterface)
{
	vector<vector<int>> lineGroups;
	EDLines* edLines = edInterface->getEDLines();

	// if there are more than 4 line segments in an edge segment, form a group
	int noOfLinesInCurrentSegment = 0;
	int currentSegment = edLines->lines[0].segmentNo;

	for (int i = 0; i < edLines->noLines; i++)
	{
		if (edLines->lines[i].segmentNo == currentSegment)
		{
			noOfLinesInCurrentSegment++;
			continue;
		}

		if (noOfLinesInCurrentSegment >= 4)
		{
			lineGroups.push_back(vector<int>());
			for (int j = 0; j < noOfLinesInCurrentSegment; j++)
				lineGroups.back().push_back(i - noOfLinesInCurrentSegment + j);
		}

		currentSegment = edLines->lines[i].segmentNo;
		noOfLinesInCurrentSegment = 1;
	}
	if (noOfLinesInCurrentSegment >= 4)
	{
		lineGroups.push_back(vector<int>());
		for (int j = 0; j < noOfLinesInCurrentSegment; j++)
			lineGroups.back().push_back(edLines->noLines - noOfLinesInCurrentSegment + j);
	}

	// correct line directions by sampling the image
	for (int i = 0; i < lineGroups.size(); i++)
	{
		for (int j = 0; j < lineGroups[i].size(); j++)
			edInterface->correctLineDirection(image, edLines->lines[lineGroups[i][j]]);

		// ensure this order: line1.start->line1.end->line2.start->line2.end->line3.start...
		LineSegment line1 = edLines->lines[lineGroups[i][0]];
		LineSegment line2 = edLines->lines[lineGroups[i][1]];

		// ############################################
		// Mod for ignoring fisheye distortion
		// Compute direction vectors for each line
		cv::Point2d v1(line1.ex - line1.sx, line1.ey - line1.sy);
		cv::Point2d v2(line2.ex - line2.sx, line2.ey - line2.sy);

		// Normalize vectors
		double mag1 = std::sqrt(v1.x * v1.x + v1.y * v1.y);
		double mag2 = std::sqrt(v2.x * v2.x + v2.y * v2.y);
		if (mag1 == 0 || mag2 == 0) continue; // skip degenerate lines

		double dot = v1.x * v2.x + v1.y * v2.y;
		double angle = std::acos(dot / (mag1 * mag2)) * 180.0 / CV_PI;

		if (std::abs(angle) < 20.0 || std::abs(angle - 180.0) < 20.0) {
			// Lines are nearly colinear → skip forming a corner here
			continue;
		}
		// ############################################

		Point2d inters = edInterface->intersectionOfLineSegments(line1, line2);

		if (abs(line1.sx - inters.x) + abs(line1.sy - inters.y) < abs(line1.ex - inters.x) + abs(line1.ey - inters.y))
			std::reverse(lineGroups[i].begin(), lineGroups[i].end());
	}
	return lineGroups;
}


void QuadDetector::detectCorners(EDInterface* edInterface, const vector<vector<int>> &lineGroups)
{
	cornerGroups = vector<vector<Corner>>();
	EdgeMap* edgeMap = edInterface->getEdgeMap();
	EDLines* edLines = edInterface->getEDLines();

	// create corner groups using the line groups
	for (int lineGroupInd = 0; lineGroupInd < lineGroups.size(); lineGroupInd++)
	{
		bool createdNewCornerGroup = false;
		for (int lineInd = 0; lineInd < lineGroups[lineGroupInd].size(); lineInd++)
		{
			int lineIndNext = (lineInd + 1) % (lineGroups[lineGroupInd].size());

			LineSegment line1 = edLines->lines[lineGroups[lineGroupInd][lineInd]];
			LineSegment line2 = edLines->lines[lineGroups[lineGroupInd][lineIndNext]];

			Point2d vec1start1end(line1.ex - line1.sx, line1.ey - line1.sy);
			Point2d vec1start2end(line2.ex - line1.sx, line2.ey - line1.sy);

			// the below condition direction (<=) looks for quads that are darker inside
			// if you are looking for quads that are lighter inside, simply change the condition direction (>=)
			if (crossProduct(vec1start1end, vec1start2end) <= 0)
				continue;

			Point2d inters = edInterface->intersectionOfLineSegments(line1, line2);

			// check the corner distance to edge segment to make sure that corners are not coming from nonlinear features
			bool onTheSegment = false;
			double thresManhDist = thresDist * 1.41; // multiply by sqrt(2)
			for (int edgePixInd = 0; edgePixInd < edgeMap->segments[line1.segmentNo].noPixels; edgePixInd++)
			{
				if (abs(edgeMap->segments[line1.segmentNo].pixels[edgePixInd].c - inters.x) + abs(edgeMap->segments[line1.segmentNo].pixels[edgePixInd].r - inters.y) < thresManhDist)
				{
					onTheSegment = true;
					break;
				}
			}

			if (!onTheSegment)
				continue;

			if (!createdNewCornerGroup)
			{
				cornerGroups.push_back(vector<Corner>());
				createdNewCornerGroup = true;
			}
			cornerGroups.back().push_back(Corner(inters, line1, line2));
		}
	}
}


bool QuadDetector::checkIfCornersFormQuad(vector<Corner> &corners, EDInterface* edInterface)
{
	if (!checkIfTwoCornersFaceEachother(corners[0], corners[2]))
		return false;

	// estimate corners[1] and corners[3] using corners[0] and corners[2]
	Corner estC1, estC3;
	// there is two combinations when forming corners[1] and corners[3]
	// we try the first one, check if it works. if not, use the second one.
	estC1 = Corner(edInterface->intersectionOfLineSegments(corners[0].l1, corners[2].l1), corners[0].l1, corners[2].l1);
	estC3 = Corner(edInterface->intersectionOfLineSegments(corners[0].l2, corners[2].l2), corners[0].l2, corners[2].l2);
	vector<Corner> estCorners = { corners[0], estC1, corners[2], estC3 };
	/*if (!checkIfQuadIsSimple(estCorners)) {
		std::cout << "Quad rejected: Not simple\n";
		return false;
	}
	if (!checkIfQuadIsSimple(estCorners))
	{
		estC1 = Corner(edInterface->intersectionOfLineSegments(corners[0].l1, corners[2].l2), corners[0].l1, corners[2].l2);
		estC3 = Corner(edInterface->intersectionOfLineSegments(corners[0].l2, corners[2].l1), corners[0].l2, corners[2].l1);
		estCorners[1] = estC1;
		estCorners[3] = estC3;
	}
	if (!checkIfQuadIsSimple(estCorners))
		return false;*/

	// check the distances between detected corners and estimated corners
	// if they are close enough, detected corners are used
	double distC1estC1 = squaredDistance(corners[1].loc, estC1.loc);
	double distC1estC3 = squaredDistance(corners[1].loc, estC3.loc);
	double distC3estC1 = squaredDistance(corners[3].loc, estC1.loc);
	double distC3estC3 = squaredDistance(corners[3].loc, estC3.loc);
	double thresDistSquared = thresDist * thresDist;

	// corners[1] - estC1 is a good match
	if ((distC1estC1 < distC1estC3) && (distC1estC1 < distC3estC1) && (distC1estC1 < distC3estC3) && (distC1estC1 < thresDistSquared))
	{
		// corners[3] is similar enough to estC3, it doesn't need to replaced
		if (distC3estC3 < thresDistSquared)
			;
		else
			corners[3] = estC3;
	}
	// corners[1] - estC3 is a good match
	else if ((distC1estC3 < distC1estC1) && (distC1estC3 < distC3estC1) && (distC1estC3 < distC3estC3) && (distC1estC3 < thresDistSquared))
	{
		// corners[3] is similar enough to estC1, it doesn't need to replaced
		if (distC3estC1 < thresDistSquared)
			;
		else
			corners[3] = estC1;
	}
	// corners[3] - estC1 is a good match
	else if ((distC3estC1 < distC1estC1) && (distC3estC1 < distC1estC3) && (distC3estC1 < distC3estC3) && (distC3estC1 < thresDistSquared))
	{
		// corners[1] is similar enough to estC3, it doesn't need to replaced
		if (distC1estC3 < thresDistSquared)
			;
		else
			corners[1] = estC3;
	}
	// corners[3] - estC3 is a good match
	else if ((distC3estC3 < distC1estC1) && (distC3estC3 < distC1estC3) && (distC3estC3 < distC3estC1) && (distC3estC3 < thresDistSquared))
	{
		// corners[1] is similar enough to estC1, it doesn't need to replaced
		if (distC1estC1 < thresDistSquared)
			;
		else
			corners[1] = estC1;
	}
	// no good match
	else
		return false;

	// order corners in clockwise
	Point2d vec13(corners[2].loc.x - corners[0].loc.x, corners[2].loc.y - corners[0].loc.y);
	Point2d vec12(corners[1].loc.x - corners[0].loc.x, corners[1].loc.y - corners[0].loc.y);

	if (crossProduct(vec13, vec12) > 0)
	{
		Corner temp = corners[1];
		corners[1] = corners[3];
		corners[3] = temp;
	}

	return true;
}


bool QuadDetector::checkIfQuadIsSimple(const vector<Corner> &corners)
{
	Point2d vec13(corners[2].loc.x - corners[0].loc.x, corners[2].loc.y - corners[0].loc.y);
	Point2d vec12(corners[1].loc.x - corners[0].loc.x, corners[1].loc.y - corners[0].loc.y);
	Point2d vec14(corners[3].loc.x - corners[0].loc.x, corners[3].loc.y - corners[0].loc.y);

	double product = crossProduct(vec13, vec12) * crossProduct(vec13, vec14);
	if (isnan(product) || product >= 0)
		return false;

	Point2d vec24(corners[3].loc.x - corners[1].loc.x, corners[3].loc.y - corners[1].loc.y);
	Point2d vec21(corners[0].loc.x - corners[1].loc.x, corners[0].loc.y - corners[1].loc.y);
	Point2d vec23(corners[2].loc.x - corners[1].loc.x, corners[2].loc.y - corners[1].loc.y);

	product = crossProduct(vec24, vec21) * crossProduct(vec24, vec23);
	if (isnan(product) || product >= 0)
		return false;

	return true;
}


bool QuadDetector::checkIfTwoCornersFaceEachother(const Corner& c1, const Corner& c2)
{
	// for both corners, we need its location and a point from each of its line segments
	// rather than using any point on the line segment, we choose the furthermost point from Corner.loc
	// we wouldn't need to do this if line segments were guaranteed to not intersect
	// however, there are some edge cases where this happens

	Point2d c1p1, c1p2, c2p1, c2p2, linePoint1, linePoint2;

	// choose a point for c1 from its line segment #1
	linePoint1 = Point2d(c1.l1.sx, c1.l1.sy);
	linePoint2 = Point2d(c1.l1.ex, c1.l1.ey);
	if (squaredDistance(c1.loc, linePoint1) > squaredDistance(c1.loc, linePoint2))
		c1p1 = Point2d(c1.l1.sx - c1.loc.x, c1.l1.sy - c1.loc.y);
	else
		c1p1 = Point2d(c1.l1.ex - c1.loc.x, c1.l1.ey - c1.loc.y);

	// choose a point for c1 from its line segment #2
	linePoint1 = Point2d(c1.l2.sx, c1.l2.sy);
	linePoint2 = Point2d(c1.l2.ex, c1.l2.ey);
	if (squaredDistance(c1.loc, linePoint1) > squaredDistance(c1.loc, linePoint2))
		c1p2 = Point2d(c1.l2.sx - c1.loc.x, c1.l2.sy - c1.loc.y);
	else
		c1p2 = Point2d(c1.l2.ex - c1.loc.x, c1.l2.ey - c1.loc.y);

	// choose a point for c2 from its line segment #1
	linePoint1 = Point2d(c2.l1.sx, c2.l1.sy);
	linePoint2 = Point2d(c2.l1.ex, c2.l1.ey);
	if (squaredDistance(c2.loc, linePoint1) > squaredDistance(c2.loc, linePoint2))
		c2p1 = Point2d(c2.l1.sx - c2.loc.x, c2.l1.sy - c2.loc.y);
	else
		c2p1 = Point2d(c2.l1.ex - c2.loc.x, c2.l1.ey - c2.loc.y);

	// choose a point for c2 from its line segment #2
	linePoint1 = Point2d(c2.l2.sx, c2.l2.sy);
	linePoint2 = Point2d(c2.l2.ex, c2.l2.ey);
	if (squaredDistance(c2.loc, linePoint1) > squaredDistance(c2.loc, linePoint2))
		c2p2 = Point2d(c2.l2.sx - c2.loc.x, c2.l2.sy - c2.loc.y);
	else
		c2p2 = Point2d(c2.l2.ex - c2.loc.x, c2.l2.ey - c2.loc.y);

	// create vectors from corner to corner
	Point2d c1c2(c2.loc.x - c1.loc.x, c2.loc.y - c1.loc.y);
	Point2d c2c1(c1.loc.x - c2.loc.x, c1.loc.y - c2.loc.y);

	// check if these two corners mutually have each other in their fan
	// is c2 inside c1's fan?
	if (crossProduct(c1c2, c1p1) * crossProduct(c1c2, c1p2) >= 0)
		return false;
	// is it behind it or in front of it?
	if (crossProduct(c1p1, c1c2) * crossProduct(c1p1, c1p2) <= 0)
		return false;

	// is c1 inside c2's fan?
	if (crossProduct(c2c1, c2p1) * crossProduct(c2c1, c2p2) >= 0)
		return false;
	// is it behind it or in front of it?
	if (crossProduct(c2p1, c2c1) * crossProduct(c2p1, c2p2) <= 0)
		return false;

	return true;
}