/****************************************************************************
 *
 *   Copyright (c) 2016 AIT, ETH Zurich. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name AIT nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/*
 * trackFeatures.cpp
 *
 *  Created on: Mar 9, 2016
 *      Author: nicolas
 */

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/operations.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <trackFeatures.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// "persistent" local variables
static cv::Mat prev_img;
static std::vector<cv::Point2f> prev_corners;
static std::vector<cv::Point2f> prev_corners_right;
static std::vector<unsigned char> prev_status(100, 0);

// for throttling debug messages
static int debug_msg_count = 0;

// local functions
static void initMorePoints(const cv::Mat &img_l, const cv::Mat &img_r, std::vector<int> &updateVect, std::vector<cv::Point2f> &z_all_l,
        std::vector<cv::Point2f> &z_all_r, int stereo);
bool stereoMatchOpticalFlow(const cv::Mat &img_l, const cv::Mat &img_r, std::vector<cv::KeyPoint> &keypointsL, std::vector<cv::Point2f> &leftPoints,
        std::vector<cv::Point2f> &rightPoints);
bool compareMatch(const cv::DMatch &first, const cv::DMatch &second);
bool compareKeypoints(const cv::KeyPoint &first, const cv::KeyPoint &second);

// corners (z_all_l, z_all_r) and status are output variables
void trackFeatures(const cv::Mat &img_l, const cv::Mat &img_r, std::vector<cv::Point2f> &features_l, std::vector<cv::Point2f> &features_r, std::vector<int> &status,
        int stereo) {
    if (!img_l.data)
        throw "Left image is invalid";
    if (stereo && !img_r.data)
        throw "Right image is invalid";

    unsigned int numPoints = status.size();
    features_l.resize(numPoints);
    std::fill(features_l.begin(), features_l.end(), cv::Point2f(-100, -100));

    features_r.resize(numPoints);
    std::fill(features_r.begin(), features_r.end(), cv::Point2f(-100, -100));

    for (size_t i = 0; i < status.size() && i < numPoints; ++i) {
        if (status[i] == 1) {
            prev_status[i] = 1;
        } else {
            prev_status[i] = 0;  // if updateVect[i] == 0 feature is inactive, == 2 request new feature
        }
    }

    std::vector<unsigned char> status_left, status_right;
    std::vector<cv::Point2f> cur_corners, right_corners;
    std::vector<float> error;

    if (!prev_img.empty()) {
        if (!prev_corners.empty()) {
            cv::calcOpticalFlowPyrLK(prev_img, img_l, prev_corners, cur_corners, status_left, error, cv::Size(9, 9), 3);
            prev_corners = cur_corners;
            if (stereo == 2)
                cv::calcOpticalFlowPyrLK(img_l, img_r, prev_corners, right_corners, status_right, error, cv::Size(9, 9), 3);

            for (size_t i = 0; i < prev_corners.size() && i < numPoints; ++i) {
                if (!(prev_status[i] && status_left[i] && (stereo != 2 || status_right[i])))
                    prev_status[i] = 0;

                if (prev_status[i] == 1) {
                    if (prev_corners[i].x < 0 || prev_corners[i].x > img_l.cols || prev_corners[i].y < 0 || prev_corners[i].y > img_l.rows
                            || ((stereo == 2)
                                    && (right_corners[i].x < 0 || right_corners[i].x > img_l.cols || right_corners[i].y < 0 || right_corners[i].y > img_l.rows))) {
                        status[i] = 0;
                    } else {
                        features_l[i] = prev_corners[i];

                        if (stereo == 2) {
                            features_r[i] = right_corners[i];
                        }
                        status[i] = 1;
                    }
                } else {
                    if (status[i] == 1)  // be careful not to overwrite 2s in updateVect
                        status[i] = 0;
                }
            }
        }
    }

    img_l.copyTo(prev_img);

    // initialize new points if needed
    initMorePoints(img_l, img_r, status, features_l, features_r, stereo);
}

// ==== local functions, hidden from outside this file ====

static void initMorePoints(const cv::Mat &img_l, const cv::Mat &img_r, std::vector<int> &updateVect, std::vector<cv::Point2f> &z_all_l,
        std::vector<cv::Point2f> &z_all_r, int stereo) {
    if (!img_l.data)
        throw "Left image is invalid";
    if (stereo && !img_r.data)
        throw "Right image is invalid";

    unsigned int targetNumPoints = 0;
    // count the features that need to be initialized
    for (int i = 0; i < updateVect.size(); i++) {
        if (updateVect[i] == 2)  // 2 means new feature requested
            targetNumPoints++;
    }

    if (!targetNumPoints)
        return;

    std::vector<cv::KeyPoint> keypointsL, keypointsR, goodKeypointsL, unusedKeypoints;
    cv::Mat descriptorsL, descriptorsR;

    int numBinsX = 4;
    int numBinsY = 4;
    int binWidth = img_l.cols / numBinsX;
    int binHeight = img_l.rows / numBinsY;
    int targetFeaturesPerBin = (updateVect.size() - 1) / (numBinsX * numBinsY) + 1;  // total number of features that should be in each bin

    std::vector<std::vector<int> > featuresPerBin(numBinsX, std::vector<int>(numBinsY, 0));

    // count the number of active features in each bin
    for (int i = 0; i < prev_corners.size(); i++) {
        if (updateVect[i] == 1) {
            int binX = prev_corners[i].x / binWidth;
            int binY = prev_corners[i].y / binHeight;

            if (binX >= numBinsX) {
                printf("Warning: writing to binX out of bounds: %d >= %d\n", binX, numBinsX);
                continue;
            }
            if (binY >= numBinsY) {
                printf("Warning: writing to binY out of bounds: %d >= %d\n", binY, numBinsY);
                continue;
            }

            featuresPerBin[binX][binY]++;
        }
    }

    unsigned int dist = binWidth / targetFeaturesPerBin;
    // go through each cell and detect features
    for (int x = 0; x < numBinsX; x++) {
        for (int y = 0; y < numBinsY; y++) {
            int neededFeatures = std::max(0, targetFeaturesPerBin - featuresPerBin[x][y]);

            if (neededFeatures) {
                int col_from = x * binWidth;
                int col_to = std::min((x + 1) * binWidth, img_l.cols);
                int row_from = y * binHeight;
                int row_to = std::min((y + 1) * binHeight, img_l.rows);

                std::vector<cv::KeyPoint> keypoints, goodKeypointsBin;
                FAST(img_l.rowRange(row_from, row_to).colRange(col_from, col_to), keypoints, 10);

                sort(keypoints.begin(), keypoints.end(), compareKeypoints);

                // add bin offsets to the points
                for (int i = 0; i < keypoints.size(); i++) {
                    keypoints[i].pt.x += col_from;
                    keypoints[i].pt.y += row_from;
                }

                // check if the new features are far enough from existing points
                int newPtIdx = 0;
                for (; newPtIdx < keypoints.size(); newPtIdx++) {
                    int new_pt_x = keypoints[newPtIdx].pt.x;
                    int new_pt_y = keypoints[newPtIdx].pt.y;

                    bool far_enough = true;
                    for (int j = 0; j < prev_corners.size(); j++) {
                        if (prev_status[j] == 0)
                            continue;
                        int existing_pt_x = prev_corners[j].x;
                        int existing_pt_y = prev_corners[j].y;
                        if (abs(existing_pt_x - new_pt_x) < dist && abs(existing_pt_y - new_pt_y) < dist) {
                            far_enough = false;
                            unusedKeypoints.push_back(keypoints[newPtIdx]);
                            break;
                        }
                    }
                    if (far_enough) {
                        // check if the new feature is too close to a new one
                        for (int j = 0; j < goodKeypointsBin.size(); j++) {
                            int existing_pt_x = goodKeypointsBin[j].pt.x;
                            int existing_pt_y = goodKeypointsBin[j].pt.y;
                            if (abs(existing_pt_x - new_pt_x) < dist && abs(existing_pt_y - new_pt_y) < dist) {
                                far_enough = false;
                                unusedKeypoints.push_back(keypoints[newPtIdx]);
                                break;
                            }
                        }
                        if (far_enough) {
                            goodKeypointsBin.push_back(keypoints[newPtIdx]);
                            if (goodKeypointsBin.size() == neededFeatures)
                                break;
                        }
                    }
                }
                // insert the good points into the vector containing the new points of the whole image
                goodKeypointsL.insert(goodKeypointsL.end(), goodKeypointsBin.begin(), goodKeypointsBin.end());
                // save the unused keypoints for later
                if (newPtIdx < keypoints.size() - 1) {
                    unusedKeypoints.insert(unusedKeypoints.end(), keypoints.begin() + newPtIdx, keypoints.end());
                }
            }
        }
    }

    // if not many features were requested, we may have found too many features. delete from all bins for equal distancing
    if (goodKeypointsL.size() > targetNumPoints) {
        int numFeaturesToRemove = goodKeypointsL.size() - targetNumPoints;
        int stepSize = targetNumPoints / numFeaturesToRemove + 2;  // make sure the step size is big enough so we dont remove too many features

        std::vector<cv::KeyPoint> goodKeypointsL_shortened;
        for (int i = 0; i < goodKeypointsL.size(); i++) {
            if (i % stepSize) {
                goodKeypointsL_shortened.push_back(goodKeypointsL[i]);
            }
        }
        goodKeypointsL = goodKeypointsL_shortened;
    }

    if (goodKeypointsL.size() < targetNumPoints) {
        // try to insert new points that were not used in the bins
        sort(unusedKeypoints.begin(), unusedKeypoints.end(), compareKeypoints);

        dist /= 2;  // reduce the distance criterion

        for (int newPtIdx = 0; newPtIdx < unusedKeypoints.size(); newPtIdx++) {
            int new_pt_x = unusedKeypoints[newPtIdx].pt.x;
            int new_pt_y = unusedKeypoints[newPtIdx].pt.y;

            bool far_enough = true;
            for (int j = 0; j < prev_corners.size(); j++) {
                if (prev_status[j] == 0)
                    continue;
                int existing_pt_x = prev_corners[j].x;
                int existing_pt_y = prev_corners[j].y;
                if (abs(existing_pt_x - new_pt_x) < dist && abs(existing_pt_y - new_pt_y) < dist) {
                    far_enough = false;
                    break;
                }
            }
            if (far_enough) {
                // check if the new feature is too close to a new one
                for (int j = 0; j < goodKeypointsL.size(); j++) {
                    int existing_pt_x = goodKeypointsL[j].pt.x;
                    int existing_pt_y = goodKeypointsL[j].pt.y;
                    if (abs(existing_pt_x - new_pt_x) < dist && abs(existing_pt_y - new_pt_y) < dist) {
                        far_enough = false;
                        break;
                    }
                }
                if (far_enough) {
                    goodKeypointsL.push_back(unusedKeypoints[newPtIdx]);
                    if (goodKeypointsL.size() == targetNumPoints)
                        break;
                }
            }
        }
    }

    if (goodKeypointsL.empty()) {
        for (int i = 0; i < updateVect.size(); i++) {
            if (updateVect[i] == 2)
                updateVect[i] = 0;
        }
        return;
    }

    std::vector<cv::Point2f> leftPoints, rightPoints;

    if (stereo) {
        if (!stereoMatchOpticalFlow(img_l, img_r, goodKeypointsL, leftPoints, rightPoints)) {
            for (int i = 0; i < updateVect.size(); i++) {
                if (updateVect[i] == 2)
                    updateVect[i] = 0;
            }
            return;
        }
        if (leftPoints.size() != rightPoints.size()) { // debug
            debug_msg_count ++;
            if (debug_msg_count % 50 == 0) {
                printf("Left and right points have different sizes: left %d, right %d\n", (int) leftPoints.size(), (int) rightPoints.size());
            }
        }
    } else {
        leftPoints.resize(goodKeypointsL.size());
        for (int i = 0; i < goodKeypointsL.size(); i++)
        {
            leftPoints[i] = goodKeypointsL[i].pt;
        }
    }
    if (leftPoints.size() < targetNumPoints) {
        debug_msg_count ++;
        if (debug_msg_count % 50 == 0) {
            printf("Number of good matches: %d, desired: %d\n", (int) leftPoints.size(), targetNumPoints);
        }
    }

    if (prev_corners.size() < updateVect.size())
        prev_corners.resize(updateVect.size());
    int matches_idx = 0;
    for (int i = 0; i < updateVect.size(); i++) {
        if (updateVect[i] == 2) {
            if (matches_idx < leftPoints.size()) {
                prev_corners[i] = leftPoints[matches_idx];
                prev_status[i] = 1;

                z_all_l[i] = leftPoints[matches_idx];

                if (stereo) {
                    z_all_r[i] = rightPoints[matches_idx];
                }

                matches_idx++;
            } else {
                updateVect[i] = 0;
            }
        }
    }
}

bool stereoMatchOpticalFlow(const cv::Mat &img_l, const cv::Mat &img_r, std::vector<cv::KeyPoint> &keypointsL, std::vector<cv::Point2f> &leftPoints,
        std::vector<cv::Point2f> &rightPoints) {
    if (!img_l.data)
        throw "Left image is invalid";
    if (!img_r.data)
        throw "Right image is invalid";

    if (keypointsL.empty())
        return false;

    std::vector<cv::Point2f> leftPoints_flow, rightPoints_flow;
    for (int i = 0; i < keypointsL.size(); i++) {
        leftPoints_flow.push_back(keypointsL[i].pt);
    }
    // get sub pixel accurate points
    cv::Size winSize = cv::Size(5, 5);
    cv::Size zeroZone = cv::Size(-1, -1);
    cv::TermCriteria criteria = cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
    cv::cornerSubPix(img_l, leftPoints_flow, winSize, zeroZone, criteria);

    std::vector<unsigned char> statusRight;
    std::vector<float> error;

    cv::calcOpticalFlowPyrLK(img_l, img_r, leftPoints_flow, rightPoints_flow, statusRight, error, cv::Size(13, 13), 4);

    for (int i = 0; i < leftPoints_flow.size(); i++) {
        if (statusRight[i]) {
            leftPoints.push_back(leftPoints_flow[i]);
            rightPoints.push_back(rightPoints_flow[i]);
        }
    }

    return true;
}

bool compareMatch(const cv::DMatch &first, const cv::DMatch &second) {
    return first.distance < second.distance;
}

bool compareKeypoints(const cv::KeyPoint &first, const cv::KeyPoint &second) {
    return first.response > second.response;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
