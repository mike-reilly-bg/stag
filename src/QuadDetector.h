// QuadDetector.h
#ifndef QUADDETECTOR_H
#define QUADDETECTOR_H

#include <vector>
#include "EDInterface.h"
#include "Quad.h"

using std::vector;

class Corner {
public:
    cv::Point2d loc;
    LineSegment l1, l2;
    Corner(cv::Point2d inLoc, LineSegment inL1, LineSegment inL2)
        : loc(inLoc), l1(inL1), l2(inL2) {}
    Corner() {}
};

class QuadDetector {
    double thresDist = 7;
    double thresProjectiveDistortion = 1.5;

    vector<vector<Corner>> cornerGroups;
    vector<Quad> distortedQuads;
    vector<Quad> quads;

    vector<vector<int>> groupLines(const cv::Mat &image, EDInterface* edInterface, EDLines* edLines);
    void detectCorners(EDInterface* edInterface, EDLines* edLines, const std::vector<std::vector<int>>& lineGroups);
    bool checkIfCornersFormQuad(vector<Corner> &corners, EDInterface* edInterface);
    bool checkIfQuadIsSimple(const vector<Corner> &corners);
    bool checkIfTwoCornersFaceEachother(const Corner& c1, const Corner& c2);

public:
    QuadDetector();

    void detectQuads(const cv::Mat &image, EDInterface* edInterface);

    const vector<vector<Corner>>& getCornerGroups();
    const vector<Quad>& getQuads() const;
    const vector<Quad>& getDistortedQuads() const;
};

#endif
