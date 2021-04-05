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
 * trackFeatures.h
 *
 *  Created on: Mar 9, 2016
 *      Author: nicolas
 */

#ifndef KLT_POINT_HANDLING
#define KLT_POINT_HANDLING

#include <opencv2/core/core.hpp>
#include <cstdio>
#include <vector>

// extracts points on a grid and tracks them over time. img_r can be a empty cv::Mat, in that case no disparity computation is done
// output variables are z_all and updateVect that need to point to existing arrays of the correct size (3*sizeof(double)*numPoints for z_all and sizeof(unsigned char)*numPoints for updateVect)
// stereo: 0 = never do stereo, 1 = do stereo with new features, 2 = always do stereo
void trackFeatures(const cv::Mat &img_l, const cv::Mat &img_r, std::vector<cv::Point2f> &z_all_l, std::vector<cv::Point2f> &z_all_r,
        std::vector<int> &updateVect, int stereo = 0);

#endif
