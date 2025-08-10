// QuadDetector.cpp

#include <vector>
#include <algorithm>
#include "opencv2/opencv.hpp"
#include <opencv2/flann.hpp>
#include "QuadDetector.h"
#include "utility.h"
#include <random>
#include <iostream>
#include <algorithm>

# define DEBUG 0

using cv::Point2d;

struct MergedLine
{
    cv::Point2d start, end;
    int SegmentNo;
};

// Draw original line segments in white and merged lines in red
void drawMergedLines(
    const std::vector<MergedLine>& merged,
    const std::vector<std::vector<int>>& lineGroups,
    const EDLines* edLines,
    cv::Mat& image)
{
    if (image.channels() == 1)
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
    for (const auto& group : lineGroups)
    {
        for (int idx : group)
        {
            const auto& l = edLines->lines[idx];
            cv::line(image, cv::Point2d(l.sx, l.sy), cv::Point2d(l.ex, l.ey), cv::Scalar(255, 255, 255), 1);
        }
    }
    for (const auto& line : merged)
    {
        cv::line(image, line.start, line.end, cv::Scalar(0, 0, 255), 2);
    }
}

// Draw quads with a different color for each
void drawQuadsColored(const std::vector<Quad>& quads, cv::Mat& image)
{
    if (image.channels() == 1)
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
    std::mt19937 rng(54321);
    std::uniform_int_distribution<int> colorDist(80, 255);

    for (const auto& q : quads)
    {
        cv::Scalar color(colorDist(rng), colorDist(rng), colorDist(rng));
        for (int i = 0; i < 4; ++i)
        {
            const cv::Point2d& p1 = q.corners[i];
            const cv::Point2d& p2 = q.corners[(i + 1) % 4];
            cv::line(image, p1, p2, color, 2);
        }
    }
}

// Draw all EDLines, color-coded by segment number
void drawLinesBySegmentNo(const EDLines* edLines, cv::Mat& image)
{
    if (image.channels() == 1)
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);

    // Make a color table for segment numbers
    std::unordered_map<int, cv::Scalar> segColor;
    std::mt19937 rng(1234);
    std::uniform_int_distribution<int> colDist(60, 255);

    for (int i = 0; i < edLines->noLines; ++i)
    {
        int seg = edLines->lines[i].segmentNo;
        if (segColor.find(seg) == segColor.end())
            segColor[seg] = cv::Scalar(colDist(rng), colDist(rng), colDist(rng));
        const auto& l = edLines->lines[i];
        cv::line(image,
            cv::Point2d(l.sx, l.sy),
            cv::Point2d(l.ex, l.ey),
            segColor[seg], 2);
    }
}

// Draw all EDLines, color-coded by segment number
void drawLinesBySegmentNo_Vector(const std::vector<MergedLine> Lines, cv::Mat& image)
{
    if (image.channels() == 1)
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);

    // Make a color table for segment numbers
    std::unordered_map<int, cv::Scalar> segColor;
    std::mt19937 rng(1235);
    std::uniform_int_distribution<int> colDist(60, 255);

    for (MergedLine line : Lines)
    {
        int seg = line.SegmentNo;
        if (segColor.find(seg) == segColor.end())
            segColor[seg] = cv::Scalar(colDist(rng), colDist(rng), colDist(rng));
        const auto& l = line;
        cv::line(image,
            cv::Point2d(l.start.x, l.start.y),
            cv::Point2d(l.end.x, l.end.y),
            segColor[seg], 2);
    }
}


    namespace util {
        template<class T, class Compare = std::less<>>
        const T& clamp(const T& v, const T& lo, const T& hi, Compare comp = Compare{})
        {
            return comp(v, lo) ? lo : comp(hi, v) ? hi : v;
        }
    }
    // use: auto clamped = util::clamp(x, 0.0, 1.0);



    // Merge nearly-colinear line segments in each group
    std::vector<MergedLine> computeMergedLines(const std::vector<std::vector<int>>& lineGroups, const EDLines* edLines, double angleThresholdDeg = 20.0)
    {
        std::vector<MergedLine> mergedLines;
        std::vector<cv::Point2d> mergedPoints;
        double dot;
        double mag1;
        double mag2;
        cv::Point2d dir;
        double cosVal;
        double angleDeg;
        cv::Point2d minPt, maxPt;

        // Iterate over each line group, which represent physical objects / edgeloops 
        for (const auto& group : lineGroups)
        {
            // Get starting size of mergedLines for later reference
            size_t groupStart = mergedLines.size();

            // Flag
            bool zero_length_line_exception = false;

            // Ignore groups with size less than 2
            if (group.size() < 2)
                continue;
        
            // Clear mergedPoints
            mergedPoints.clear();
            mergedPoints.reserve(group.size() * 2);

            // Initialize the "previous line" index as the first line in the group.
            int prevIdx = group[0];
            const LineSegment* prev = &edLines->lines[prevIdx];
            cv::Point2d prevVec(prev->ex - prev->sx, prev->ey - prev->sy);
            mergedPoints.push_back(cv::Point2d(prev->sx, prev->sy));
            mergedPoints.push_back(cv::Point2d(prev->ex, prev->ey));
        
           // Get segment number for this line group
            int runSegmentNo = prev->segmentNo;

            // Iterate over all lines in the line group and check
            for (size_t i = 1; i < group.size(); ++i)
            {
                // Get current line segment.
                // Starts at 0, ends at last line in group.
                int currIdx = group[i];
                const LineSegment* curr = &edLines->lines[currIdx];
                cv::Point2d currVec(curr->ex - curr->sx, curr->ey - curr->sy);
            
                // Do math on current and previous line segments
                dot = prevVec.x * currVec.x + prevVec.y * currVec.y;
                mag1 = cv::norm(prevVec);
                mag2 = cv::norm(currVec);
                if (mag1 == 0 || mag2 == 0)
                    continue;
                cosVal = util::clamp(dot / (mag1 * mag2), -1.0, 1.0);
                angleDeg = std::acos(cosVal) * 180.0 / CV_PI;

                // debug
#if DEBUG
                std::cout << "\n\nsegment number:  " << runSegmentNo;
                std::cout << "\ncurrent line index in group: " << i;
                std::cout << "\ncurrent line index in edLines" << currIdx;
                std::cout << "\ncurrent line starting point:  (" << curr->sx << ", " << curr->sy << ")";
                std::cout << "\ncurrent line ending point:    (" << curr->ex << ", " << curr->ey << ")";
                std::cout << "\nPrevious line starting point:  (" << prev->sx << ", " << prev->sy << ")";
                std::cout << "\nPrevious line ending point:    (" << prev->ex << ", " << prev->ey << ")";
#endif

                // Check colinearity
                if ((std::abs(angleDeg) < angleThresholdDeg || std::abs(angleDeg - 180.0) < angleThresholdDeg))
                    // Close to colinear, merge current and previous lines
                {
                    mergedPoints.push_back(cv::Point2d(curr->sx, curr->sy));
                    mergedPoints.push_back(cv::Point2d(curr->ex, curr->ey));
#if DEBUG
                    std::cout << "\ncolinear:  YES";
#endif
                }
                else
                // Not close to colinear, no merging needed
                {
                    // Dump buffered colinear line points if they exist
                    // MergedPoints buffer looks like [line1_start, line1_end, line2_start, ...]
#if DEBUG
                    std::cout << "\ncolinear:  NO";
#endif
                    if (mergedPoints.size() > 0)
                    {
#if DEBUG
                        std::cout << "\n** Existing colinear line point buffer dumped. Point buffer size:  " << mergedPoints.size();
#endif
                        // project onto the unit direction of the run and pick min/max projection
                        dir = mergedPoints.back() - mergedPoints.front();
                        double n = cv::norm(dir);
                        if (n != 0) dir /= n;
                        auto projCmp = [&dir](const cv::Point2d& p, const cv::Point2d& q)
                            {
                                return p.dot(dir) < q.dot(dir);
                            };
                        if (n == 0) {          // degenerate run: use first/last
                            minPt = mergedPoints.front();
                            maxPt = mergedPoints.back();
                        }
                        else {
                            minPt = *std::min_element(mergedPoints.begin(), mergedPoints.end(), projCmp);
                            maxPt = *std::max_element(mergedPoints.begin(), mergedPoints.end(), projCmp);

                        }
#if DEBUG
                        std::cout << "\n** Colinear line point buffer starting point:  (" << minPt.x << ", " << minPt.y << ")";
                        std::cout << "\n** Colinear line point buffer ending point:    (" << maxPt.x << ", " << maxPt.y << ")";
#endif
                        mergedLines.push_back({ minPt, maxPt, runSegmentNo });
                    }
                    // Start new buffer for colinear lines
                    mergedPoints.clear();
                    mergedPoints.push_back(cv::Point2d(curr->sx, curr->sy));
                    mergedPoints.push_back(cv::Point2d(curr->ex, curr->ey));
#if DEBUG
                    std::cout << "\nCurrent line starting and ending points added to point buffer.";
#endif
                }
                prevVec = currVec;
            }

            // #######################################################################
            // #######################################################################
            // vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
            // Pac-Man: Check for colinearity with the current mergedPoints buffer and the first merged line in the group
#if DEBUG
            std::cout << "\n\nDone with line group (segment " << runSegmentNo << "). Checking colinearity with the first and last elements...";
#endif

            // Special case:  whole group was colinear
            if (mergedLines.size() == groupStart) {   // nothing flushed yet
                // entire group is colinear: treat prev ↔ first check as “YES”
                // we can just append the buffered run and skip Pac-Man logic
                if (mergedPoints.size() >= 2) {
                    // project onto the unit direction of the run and pick min/max projection
                    dir = mergedPoints.back() - mergedPoints.front();
                    double n = cv::norm(dir);
                    if (n != 0) dir /= n;
                    auto projCmp = [&dir](const cv::Point2d& p, const cv::Point2d& q)
                        {
                            return p.dot(dir) < q.dot(dir);
                        };
                    if (n == 0) {          // degenerate run: use first/last
                         minPt = mergedPoints.front();
                         maxPt = mergedPoints.back();
                    }
                    else {
                         minPt = *std::min_element(mergedPoints.begin(), mergedPoints.end(), projCmp);
                         maxPt = *std::max_element(mergedPoints.begin(), mergedPoints.end(), projCmp);
                         
                    }
                    mergedLines.push_back({ minPt, maxPt, runSegmentNo });
                }
                continue;            // go to next group
            }

            // Get the first line in the merged line group (the first line that was already processed).
            // This becomes the "current line"
            MergedLine firstLineInGroup = mergedLines[groupStart];
            cv::Point2d currVec(firstLineInGroup.end.x - firstLineInGroup.start.x, firstLineInGroup.end.y - firstLineInGroup.start.y);
        
            // Do math.  "Current line" is the first merged line, "previous line" is the last line in the overall line group that still needs processing.
            dot = prevVec.x * currVec.x + prevVec.y * currVec.y;
            mag1 = cv::norm(prevVec);
            mag2 = cv::norm(currVec);
            if (mag1 == 0 || mag2 == 0)
                zero_length_line_exception = true;
            if (!zero_length_line_exception)
            {
                cosVal = util::clamp(dot / (mag1 * mag2), -1.0, 1.0);
                angleDeg = std::acos(cosVal) * 180.0 / CV_PI;
                if ((std::abs(angleDeg) < angleThresholdDeg || std::abs(angleDeg - 180.0) < angleThresholdDeg))
                    // Close to colinear, merge current and previous lines
                {
#if DEBUG
                    std::cout << "\nFirst and last lines colinear:  YES";
#endif
                    // Add the first line to the point buffer and remove it from the merged line list.
                    mergedPoints.push_back(Point2d(firstLineInGroup.start.x, firstLineInGroup.start.y));
                    mergedPoints.push_back(Point2d(firstLineInGroup.end.x, firstLineInGroup.end.y));
#if DEBUG
                    std::cout << "\n** First merged line starting point:  (" << firstLineInGroup.start.x << ", " << firstLineInGroup.start.y << ")";
                    std::cout << "\n** First merged line ending point:    (" << firstLineInGroup.end.x << ", " << firstLineInGroup.end.y << ")";
#endif
                    mergedLines.erase(mergedLines.begin() + groupStart);
                }
                else
                    // Not close to colinear, no merging needed
                {
#if DEBUG
                    std::cout << "\nFirst and last lines colinear:  NO";
#endif
                }
                // Add the last line to the merged line list.
                // If the previous colinearity check was true, it will also contain the first merged line.
                if (mergedPoints.size() >= 2) // Should always be true
                {
#if DEBUG
                    std::cout << "\n** Adding the last line to the merged lines list. Point buffer size:  " << mergedPoints.size();
#endif
                    // project onto the unit direction of the run and pick min/max projection
                    dir = mergedPoints.back() - mergedPoints.front();
                    double n = cv::norm(dir);
                    if (n != 0) dir /= n;
                    auto projCmp = [&dir](const cv::Point2d& p, const cv::Point2d& q)
                        {
                            return p.dot(dir) < q.dot(dir);
                        };
                    if (n == 0) {          // degenerate run: use first/last
                        minPt = mergedPoints.front();
                        maxPt = mergedPoints.back();
                    }
                    else {
                        minPt = *std::min_element(mergedPoints.begin(), mergedPoints.end(), projCmp);
                        maxPt = *std::max_element(mergedPoints.begin(), mergedPoints.end(), projCmp);

                    }
#if DEBUG
                    std::cout << "\n** Last merged line starting point:  (" << minPt.x << ", " << minPt.y << ")";
                    std::cout << "\n** Last merged line ending point:    (" << maxPt.x << ", " << maxPt.y << ")";
#endif
                    mergedLines.push_back({ minPt, maxPt, runSegmentNo });
                }
            }
            if (zero_length_line_exception && mergedPoints.size() >= 2) {
                dir = mergedPoints.back() - mergedPoints.front();
                double n = cv::norm(dir);
                if (n != 0) dir /= n;
                auto projCmp = [&dir](const cv::Point2d& p, const cv::Point2d& q) {
                    return p.dot(dir) < q.dot(dir);
                    };
                if (n == 0) {
                    minPt = mergedPoints.front();
                    maxPt = mergedPoints.back();
                }
                else {
                    minPt = *std::min_element(mergedPoints.begin(), mergedPoints.end(), projCmp);
                    maxPt = *std::max_element(mergedPoints.begin(), mergedPoints.end(), projCmp);
                }
                mergedLines.push_back({ minPt, maxPt, runSegmentNo });
            }
            // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            // #######################################################################
            // #######################################################################
#if DEBUG
            std::cout << "\n\nDone with line group (segment " << runSegmentNo << ").";
#endif
        }
        return mergedLines;
    }

QuadDetector::QuadDetector() = default;

auto sortCornersClockwise =
[](std::vector<Corner>& v)
    {
        // centre of gravity
        Point2d centre(0, 0);
        for (auto& c : v) centre += c.loc;
        centre *= 1.0 / v.size();

        std::sort(v.begin(), v.end(),
            [&centre](const Corner& a, const Corner& b)
            {
                double angA = std::atan2(a.loc.y - centre.y,
                    a.loc.x - centre.x);
                double angB = std::atan2(b.loc.y - centre.y,
                    b.loc.x - centre.x);
                return angA < angB;          // counter-clockwise
            });
    };

// === The MAIN detection function with fallback logic ===
void QuadDetector::detectQuads(const cv::Mat& image, EDInterface* edInterface)
{
    cornerGroups.clear();
    distortedQuads.clear();
    quads.clear();

    edInterface->runEDPFandEDLines(image);

    EDLines* edLines = edInterface->getEDLines();

    // Debug
    cv::Mat segmentColorImg = image.clone();
    drawLinesBySegmentNo(edLines, segmentColorImg);
    cv::imwrite("edLines_by_segment.png", segmentColorImg);

    std::vector<std::vector<int>> lineGroups = groupLines(image, edInterface, edLines);
    // std::cout << "lineGroups size: " << static_cast<int>(lineGroups.size());

    // Debug
    cv::Mat debugImage = image.clone(); // Make a modifiable copy
    std::vector<MergedLine> mergedLines = computeMergedLines(lineGroups, edLines, 20.0);
    drawMergedLines(mergedLines, lineGroups, edLines, debugImage);
    cv::imwrite("merged_lines.png", debugImage);

    // Create a synthetic EDLines object
    EDLines* mergedEDLines = new EDLines(1, 1);
    mergedEDLines->clear();
    for (const auto& m : mergedLines)
    {
        double dx = m.end.x - m.start.x;
        double dy = m.end.y - m.start.y;
        double len = std::sqrt(dx * dx + dy * dy);
        if (len < 10)
            continue;
        double a, b;
        int invert;
        if (std::abs(dx) > std::abs(dy))
        {
            invert = 0;
            b = dy / dx;
            a = m.start.y - b * m.start.x;
        }
        else
        {
            invert = 1;
            b = dx / dy;
            a = m.start.x - b * m.start.y;
        }
        mergedEDLines->add(a, b, invert, m.start.x, m.start.y, m.end.x, m.end.y, m.SegmentNo, 0, static_cast<int>(len));
    }

    // Debug
    cv::Mat mergedsegmentColorImg = image.clone();
    drawLinesBySegmentNo_Vector(mergedLines, mergedsegmentColorImg);
    cv::imwrite("mergedLines_by_segment.png", mergedsegmentColorImg);

    // Debug
    cv::Mat mergedsegmentColorImg2 = image.clone();
    drawLinesBySegmentNo(mergedEDLines, mergedsegmentColorImg2);
    cv::imwrite("mergedEDLines_by_segment.png", mergedsegmentColorImg2);

    // Group merged lines
    std::vector<std::vector<int>> mergedLineGroups = groupLines(image, edInterface, mergedEDLines);
    // std::cout << "mergedLineGroups size: " << static_cast<int>(mergedLineGroups.size());

    // Run detection on merged lines
    detectCorners(edInterface, mergedEDLines, mergedLineGroups); // skip segment check for merged

    std::mt19937 rng(54321);
    std::uniform_int_distribution<int> colorDist(80, 255);

    // Debug
    // std::cout << "merged line corner group count: " << static_cast<int>(cornerGroups.size()) << "\n";
    cv::Mat merged_line_corners_before_corner_merge_image = image.clone();
    if (merged_line_corners_before_corner_merge_image.channels() == 1)
        cv::cvtColor(merged_line_corners_before_corner_merge_image, merged_line_corners_before_corner_merge_image, cv::COLOR_GRAY2BGR);
    for (size_t groupIdx = 0; groupIdx < cornerGroups.size(); ++groupIdx)
    {
        const auto& group = cornerGroups[groupIdx];
        for (const auto& corner : group)
        {
            cv::circle(merged_line_corners_before_corner_merge_image, corner.loc, 3, cv::Scalar(0, 255, 0), -1);
            // Draw group number next to the corner
            cv::putText(merged_line_corners_before_corner_merge_image,
                std::to_string(groupIdx),
                corner.loc + cv::Point2d(5, -5), // offset so text doesn't cover the circle
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        }
    }
    cv::imwrite("merged_line_corners.png", merged_line_corners_before_corner_merge_image);

    // Debug
    // std::cout << "merged line merged corner group count: " << static_cast<int>(cornerGroups.size()) << "\n";
    cv::Mat merged_line_corners_after_corner_merge_image = image.clone();
    if (merged_line_corners_after_corner_merge_image.channels() == 1)
        cv::cvtColor(merged_line_corners_after_corner_merge_image, merged_line_corners_after_corner_merge_image, cv::COLOR_GRAY2BGR);
    for (int i = 0; i < mergedEDLines->noLines; ++i)
    {
        auto& l = mergedEDLines->lines[i];
        cv::Scalar color(colorDist(rng), colorDist(rng), colorDist(rng));
        cv::line(merged_line_corners_after_corner_merge_image, cv::Point2d(l.sx, l.sy), cv::Point2d(l.ex, l.ey), color, 1);
    }
    for (size_t groupIdx = 0; groupIdx < cornerGroups.size(); ++groupIdx)
    {
        const auto& group = cornerGroups[groupIdx];
        for (const auto& corner : group)
        {
            cv::circle(merged_line_corners_after_corner_merge_image, corner.loc, 3, cv::Scalar(0, 255, 0), -1);
            // Draw group number next to the corner
            cv::putText(merged_line_corners_after_corner_merge_image,
                std::to_string(groupIdx),
                corner.loc + cv::Point2d(5, -5), // offset so text doesn't cover the circle
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        }
    }
    cv::imwrite("merged_line_corners_and_unique_lines.png", merged_line_corners_after_corner_merge_image);

    int i_debug = -1;
    // Find quads from merged line groups
    for (const auto& group : cornerGroups)
    {
        std::cout << "\n";
        i_debug++;
#if DEBUG
        std::cout << "\nExamining group  " << i_debug << ".";
        std::cout << "\n--> Group size is " << group.size() << ".";
#endif
        if (group.size() < 4) {
#if DEBUG
            std::cout << "\n--> Group " << i_debug << " only has " << group.size() << " corners (<4). Skipping.";
#endif
            continue;
        }
        else {
#if DEBUG
            std::cout << "\n--> Group " << i_debug << " has enough corners.";
#endif
        }
        for (size_t start = 0; start < group.size(); ++start)
        {
            const Corner& c1 = group[start];
            const Corner& c2 = group[(start + 1) % group.size()];
            const Corner& c3 = group[(start + 2) % group.size()];
            const Corner& c4 = group[(start + 3) % group.size()];
#if false
            std::cout << "\n --> corner 1, line 1: (" << c1.l1.sx << ", " << c1.l1.sy << ")";
            std::cout << "\n --> corner 1, line 2: (" << c1.l2.sx << ", " << c1.l2.sy << ")";
            std::cout << "\n --> corner 2, line 1: (" << c2.l1.sx << ", " << c2.l1.sy << ")";
            std::cout << "\n --> corner 2, line 2: (" << c2.l2.sx << ", " << c2.l2.sy << ")";
            std::cout << "\n --> corner 3, line 1: (" << c3.l1.sx << ", " << c3.l1.sy << ")";
            std::cout << "\n --> corner 3, line 2: (" << c3.l2.sx << ", " << c3.l2.sy << ")";
            std::cout << "\n --> corner 4, line 1: (" << c4.l1.sx << ", " << c4.l1.sy << ")";
            std::cout << "\n --> corner 4, line 2: (" << c4.l2.sx << ", " << c4.l2.sy << ")";
#endif
            std::vector<Corner> corners = { c1, c2, c3, c4 };
            if (!checkIfCornersFormQuad(corners, edInterface)) {
#if DEBUG
                std::cout << "\n--> The corners from group " << i_debug << " fail the checkIfCornersFormQuad check. Skipping.";
#endif
                continue;
            }
#if DEBUG
            std::cout << "\n--> The corners from group " << i_debug << " pass checkIfCornersFormQuad check.";
#endif
            std::vector<Point2d> cornerLocs = {
                corners[0].loc, corners[1].loc, corners[2].loc, corners[3].loc };
            Quad quad(cornerLocs);
            quads.push_back(quad);
        }
    }

    // Fallback to raw lines if needed
    if (quads.empty())
    {
        detectCorners(edInterface, edLines, lineGroups);

        // Debug
        // std::cout << "nonmerged line corner group count: " << static_cast<int>(cornerGroups.size()) << "\n";
        cv::Mat nonmerged_line_corners_before_corner_merge_image = image.clone();
        if (nonmerged_line_corners_before_corner_merge_image.channels() == 1)
            cv::cvtColor(nonmerged_line_corners_before_corner_merge_image, nonmerged_line_corners_before_corner_merge_image, cv::COLOR_GRAY2BGR);
        for (auto& group : cornerGroups)
            for (auto& corner : group)
                cv::circle(nonmerged_line_corners_before_corner_merge_image, corner.loc, 3, cv::Scalar(0, 255, 0), -1);
        cv::imwrite("nonmerged_line_corners.png", nonmerged_line_corners_before_corner_merge_image);

        // Debug
        //std::cout << "nonmerged line merged corner group count: " << static_cast<int>(cornerGroups.size()) << "\n";
        cv::Mat nonmerged_line_corners_after_corner_merge_image = image.clone();
        if (nonmerged_line_corners_after_corner_merge_image.channels() == 1)
            cv::cvtColor(nonmerged_line_corners_after_corner_merge_image, nonmerged_line_corners_after_corner_merge_image, cv::COLOR_GRAY2BGR);
        for (int i = 0; i < edLines->noLines; ++i)
        {
            auto& l = edLines->lines[i];
            cv::Scalar color(colorDist(rng), colorDist(rng), colorDist(rng));
            cv::line(nonmerged_line_corners_after_corner_merge_image, cv::Point2d(l.sx, l.sy), cv::Point2d(l.ex, l.ey), color, 1);
        }
        for (auto& group : cornerGroups)
            for (auto& corner : group)
                cv::circle(nonmerged_line_corners_after_corner_merge_image, corner.loc, 3, cv::Scalar(0, 255, 0), -1);
        cv::imwrite("nonmerged_line_corners_with_unique_lines.png", nonmerged_line_corners_after_corner_merge_image);

        // === PASTE THE SAME LOOP HERE for fallback ===
        for (const auto& group : cornerGroups)
        {
            if (group.size() < 4)
                continue;
            for (size_t start = 0; start < group.size(); ++start)
            {
                const Corner& c1 = group[start];
                const Corner& c2 = group[(start + 1) % group.size()];
                const Corner& c3 = group[(start + 2) % group.size()];
                const Corner& c4 = group[(start + 3) % group.size()];
                std::vector<Corner> corners = { c1, c2, c3, c4 };
                if (!checkIfCornersFormQuad(corners, edInterface))
                    continue;
                std::vector<Point2d> cornerLocs = {
                    corners[0].loc, corners[1].loc, corners[2].loc, corners[3].loc };
                Quad quad(cornerLocs);
                quads.push_back(quad);
            }
        }
    }
    cv::Mat quadImage = image.clone();
    if (quadImage.channels() == 1)
        cv::cvtColor(quadImage, quadImage, cv::COLOR_GRAY2BGR);
    drawQuadsColored(quads, quadImage);
    cv::imwrite("quads_colored.png", quadImage);
    delete mergedEDLines;
}

const std::vector<std::vector<Corner>>& QuadDetector::getCornerGroups() { return cornerGroups; }
const std::vector<Quad>& QuadDetector::getQuads() const { return quads; }
const std::vector<Quad>& QuadDetector::getDistortedQuads() const
{
    return distortedQuads;
}

vector<vector<int>> QuadDetector::groupLines(const cv::Mat& image, EDInterface* edInterface, EDLines* edLines)
{
    vector<vector<int>> lineGroups;

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

        LineSegment line1;
        LineSegment line2;
        // ensure this order: line1.start->line1.end->line2.start->line2.end->line3.start...
        line1 = edLines->lines[lineGroups[i][0]];
        line2 = edLines->lines[lineGroups[i][1]];

        //// ############################################
        //// Mod for ignoring fisheye distortion
        //// Compute direction vectors for each line
        // cv::Point2d v1(line1.ex - line1.sx, line1.ey - line1.sy);
        // cv::Point2d v2(line2.ex - line2.sx, line2.ey - line2.sy);

        //// Normalize vectors
        // double mag1 = std::sqrt(v1.x * v1.x + v1.y * v1.y);
        // double mag2 = std::sqrt(v2.x * v2.x + v2.y * v2.y);
        // if (mag1 == 0 || mag2 == 0) continue; // skip degenerate lines

        // double dot = v1.x * v2.x + v1.y * v2.y;
        // double angle = std::acos(dot / (mag1 * mag2)) * 180.0 / CV_PI;

        // if (std::abs(angle) < 20.0 || std::abs(angle - 180.0) < 20.0) {
        //  // Lines are nearly colinear → skip forming a corner here
        //  continue;
        // }
        //// ############################################

        Point2d inters = edInterface->intersectionOfLineSegments(line1, line2);

        if (abs(line1.sx - inters.x) + abs(line1.sy - inters.y) < abs(line1.ex - inters.x) + abs(line1.ey - inters.y))
            std::reverse(lineGroups[i].begin(), lineGroups[i].end());
    }
    return lineGroups;
}

void QuadDetector::detectCorners(EDInterface* edInterface, EDLines* edLines, const std::vector<std::vector<int>>& lineGroups)
{
    cornerGroups.clear();
    EdgeMap* edgeMap = edInterface->getEdgeMap();

    // create corner groups using the line groups
    for (int lineGroupInd = 0; lineGroupInd < lineGroups.size(); lineGroupInd++)
    {
        bool createdNewCornerGroup = false;
#if false
        std::cout << "\ncorner group: " << lineGroupInd;
#endif
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
#if false
            std::cout << "\nthe intersection of (" << line1.sx << ", " << line1.sy << "), (" << line1.ex << ", " << line1.ey << ") and (" << line2.sx << ", " << line2.sy << "), (" << line2.ex << ", " << line2.ey << ") is (" << inters.x << ", " << inters.y << ").";
#endif

            // check the corner distance to edge segment to make sure that corners are not coming from nonlinear features
            bool onTheSegment = true; // Default to allow if skipping check

            onTheSegment = false;
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
#if false
            std::cout << "\npushed back corner (" << inters.x << ", " << inters.y << ").";
#endif
            cornerGroups.back().push_back(Corner(inters, line1, line2));
        }
    }
}

bool QuadDetector::checkIfCornersFormQuad(vector<Corner>& corners, EDInterface* edInterface)
{
#if false
    std::cout << "\n ----> corner 1, line 1: (" << corners[0].l1.sx << ", " << corners[0].l1.sy << ")";
    std::cout << "\n ----> corner 1, line 2: (" << corners[0].l2.sx << ", " << corners[0].l2.sy << ")";
    std::cout << "\n ----> corner 2, line 1: (" << corners[1].l1.sx << ", " << corners[1].l1.sy << ")";
    std::cout << "\n ----> corner 2, line 2: (" << corners[1].l2.sx << ", " << corners[1].l2.sy << ")";
    std::cout << "\n ----> corner 3, line 1: (" << corners[2].l1.sx << ", " << corners[2].l1.sy << ")";
    std::cout << "\n ----> corner 3, line 2: (" << corners[2].l2.sx << ", " << corners[2].l2.sy << ")";
    std::cout << "\n ----> corner 4, line 1: (" << corners[3].l1.sx << ", " << corners[3].l1.sy << ")";
    std::cout << "\n ----> corner 4, line 2: (" << corners[3].l2.sx << ", " << corners[3].l2.sy << ")";
#endif
    if (!checkIfTwoCornersFaceEachother(corners[0], corners[2]))
    {
#if false
        std::cout << "\n----> Quad fail: c1/c3 not facing each other. Corners: ";
        for (int i = 0; i < 4; ++i)
            std::cout << "(" << corners[i].loc.x << "," << corners[i].loc.y << ") ";
#endif
        return false;
    }
    
    // estimate corners[1] and corners[3] using corners[0] and corners[2]
    Corner estC1, estC3;
    // there is two combinations when forming corners[1] and corners[3]
    // we try the first one, check if it works. if not, use the second one.
    estC1 = Corner(edInterface->intersectionOfLineSegments(corners[0].l1, corners[2].l1), corners[0].l1, corners[2].l1);
    estC3 = Corner(edInterface->intersectionOfLineSegments(corners[0].l2, corners[2].l2), corners[0].l2, corners[2].l2);
#if DEBUG
    std::cout << "\nthe intersection of (" << corners[0].l1.sx << ", " << corners[0].l1.sy << "), (" << corners[0].l1.ex << ", " << corners[0].l1.ey << ") and (" << corners[2].l1.sx << ", " << corners[2].l1.sy << "), (" << corners[2].l1.ex << ", " << corners[2].l1.ey << ") is (" << estC1.loc.x << ", " << estC1.loc.y << ").";
    std::cout << "\nthe intersection of (" << corners[0].l2.sx << ", " << corners[0].l2.sy << "), (" << corners[0].l2.ex << ", " << corners[0].l2.ey << ") and (" << corners[2].l2.sx << ", " << corners[2].l2.sy << "), (" << corners[2].l2.ex << ", " << corners[2].l2.ey << ") is (" << estC3.loc.x << ", " << estC3.loc.y << ").";
#endif
    vector<Corner> estCorners = { corners[0], estC1, corners[2], estC3 };
    if (!checkIfQuadIsSimple(estCorners))
    {
#if DEBUG
        std::cout << "\n----> Quad rejected: Not simple. Trying second configuration.";
#endif
        estC1 = Corner(edInterface->intersectionOfLineSegments(corners[0].l1, corners[2].l2), corners[0].l1, corners[2].l2);
        estC3 = Corner(edInterface->intersectionOfLineSegments(corners[0].l2, corners[2].l1), corners[0].l2, corners[2].l1);
        estCorners[1] = estC1;
        estCorners[3] = estC3;
#if DEBUG
        std::cout << "\nthe intersection of (" << corners[0].l1.sx << ", " << corners[0].l1.sy << "), (" << corners[0].l1.ex << ", " << corners[0].l1.ey << ") and (" << corners[2].l2.sx << ", " << corners[2].l2.sy << "), (" << corners[2].l2.ex << ", " << corners[2].l2.ey << ") is (" << estC1.loc.x << ", " << estC1.loc.y << ").";
        std::cout << "\nthe intersection of (" << corners[0].l2.sx << ", " << corners[0].l2.sy << "), (" << corners[0].l2.ex << ", " << corners[0].l2.ey << ") and (" << corners[2].l1.sx << ", " << corners[2].l1.sy << "), (" << corners[2].l1.ex << ", " << corners[2].l1.ey << ") is (" << estC3.loc.x << ", " << estC3.loc.y << ").";
#endif
    }
    if (!checkIfQuadIsSimple(estCorners)) {
#if DEBUG
        std::cout << "\n----> Quad rejected: Not simple";
#endif
        return false;
    }
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
    else {
#if DEBUG
        std::cout << "\n----> Quad fail: no good match.";
        for (int i = 0; i < 4; ++i)
            std::cout << " (" << corners[i].loc.x << "," << corners[i].loc.y << ") ";
#endif
        return false;
    }
    // order corners in clockwise
    Point2d vec13(corners[2].loc.x - corners[0].loc.x, corners[2].loc.y - corners[0].loc.y);
    Point2d vec12(corners[1].loc.x - corners[0].loc.x, corners[1].loc.y - corners[0].loc.y);

    if (crossProduct(vec13, vec12) > 0)
    {
        Corner temp = corners[1];
        corners[1] = corners[3];
        corners[3] = temp;
    }
#if DEBUG
    std::cout << "\nPassed corner quad check!";
#endif
    return true;
}

bool QuadDetector::checkIfQuadIsSimple(const vector<Corner>& corners)
{
    Point2d vec13(corners[2].loc.x - corners[0].loc.x, corners[2].loc.y - corners[0].loc.y);
    Point2d vec12(corners[1].loc.x - corners[0].loc.x, corners[1].loc.y - corners[0].loc.y);
    Point2d vec14(corners[3].loc.x - corners[0].loc.x, corners[3].loc.y - corners[0].loc.y);

    double product = crossProduct(vec13, vec12) * crossProduct(vec13, vec14);
    if (isnan(product) || product >= 0) {
#if false
        std::cout << "\n ---->2 corner 1, line 1: (" << corners[0].l1.sx << ", " << corners[0].l1.sy << ")";
        std::cout << "\n ---->2 corner 1, line 2: (" << corners[0].l2.sx << ", " << corners[0].l2.sy << ")";
        std::cout << "\n ---->2 corner 2, line 1: (" << corners[1].l1.sx << ", " << corners[1].l1.sy << ")";
        std::cout << "\n ---->2 corner 2, line 2: (" << corners[1].l2.sx << ", " << corners[1].l2.sy << ")";
        std::cout << "\n ---->2 corner 3, line 1: (" << corners[2].l1.sx << ", " << corners[2].l1.sy << ")";
        std::cout << "\n ---->2 corner 3, line 2: (" << corners[2].l2.sx << ", " << corners[2].l2.sy << ")";
        std::cout << "\n ---->2 corner 4, line 1: (" << corners[3].l1.sx << ", " << corners[3].l1.sy << ")";
        std::cout << "\n ---->2 corner 4, line 2: (" << corners[3].l2.sx << ", " << corners[3].l2.sy << ")";
#endif
#if DEBUG
        std::cout << "\n ---->3 corner 1 location: (" << corners[0].loc.x << ", " << corners[0].loc.y << ")";
        std::cout << "\n ---->3 corner 2 location: (" << corners[1].loc.x << ", " << corners[1].loc.y << ")";
        std::cout << "\n ---->3 corner 3 location: (" << corners[2].loc.x << ", " << corners[2].loc.y << ")";
        std::cout << "\n ---->3 corner 4 location: (" << corners[3].loc.x << ", " << corners[3].loc.y << ")";
        std::cout << "\n------> Quad fail: cross.";
        for (int i = 0; i < 4; ++i)
            std::cout << " (" << corners[i].loc.x << "," << corners[i].loc.y << ") ";
#endif
        return false;
    }

    Point2d vec24(corners[3].loc.x - corners[1].loc.x, corners[3].loc.y - corners[1].loc.y);
    Point2d vec21(corners[0].loc.x - corners[1].loc.x, corners[0].loc.y - corners[1].loc.y);
    Point2d vec23(corners[2].loc.x - corners[1].loc.x, corners[2].loc.y - corners[1].loc.y);

    product = crossProduct(vec24, vec21) * crossProduct(vec24, vec23);
    if (isnan(product) || product >= 0) {
#if false
        std::cout << "\n ---->2 corner 1, line 1: (" << corners[0].l1.sx << ", " << corners[0].l1.sy << ")";
        std::cout << "\n ---->2 corner 1, line 2: (" << corners[0].l2.sx << ", " << corners[0].l2.sy << ")";
        std::cout << "\n ---->2 corner 2, line 1: (" << corners[1].l1.sx << ", " << corners[1].l1.sy << ")";
        std::cout << "\n ---->2 corner 2, line 2: (" << corners[1].l2.sx << ", " << corners[1].l2.sy << ")";
        std::cout << "\n ---->2 corner 3, line 1: (" << corners[2].l1.sx << ", " << corners[2].l1.sy << ")";
        std::cout << "\n ---->2 corner 3, line 2: (" << corners[2].l2.sx << ", " << corners[2].l2.sy << ")";
        std::cout << "\n ---->2 corner 4, line 1: (" << corners[3].l1.sx << ", " << corners[3].l1.sy << ")";
        std::cout << "\n ---->2 corner 4, line 2: (" << corners[3].l2.sx << ", " << corners[3].l2.sy << ")";
        std::cout << "\n ---->3 corner 1 location: (" << corners[0].loc.x << ", " << corners[0].loc.y << ")";
        std::cout << "\n ---->3 corner 2 location: (" << corners[1].loc.x << ", " << corners[1].loc.y << ")";
        std::cout << "\n ---->3 corner 3 location: (" << corners[2].loc.x << ", " << corners[2].loc.y << ")";
        std::cout << "\n ---->3 corner 4 location: (" << corners[3].loc.x << ", " << corners[3].loc.y << ")";
#endif
#if DEBUG
        std::cout << "\n------> Quad fail: cross.";
#endif
#if false
        for (int i = 0; i < 4; ++i)
            std::cout << "\n (" << corners[i].loc.x << "," << corners[i].loc.y << ") ";
#endif
        return false;
    }
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