/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MAV_PLANNING_MSGS_CONVERSIONS_H
#define MAV_PLANNING_MSGS_CONVERSIONS_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include "mav_planning_msgs/PolynomialSegment.h"
#include "mav_planning_msgs/PolynomialTrajectory.h"
#include "mav_planning_msgs/eigen_planning_msgs.h"

// deprecated
#include "mav_planning_msgs/conversions_deprecated.h"

namespace mav_planning_msgs {

/// Converts a PolynomialSegment double array to an Eigen::VectorXd.
inline void vectorFromMsgArray(const PolynomialSegment::_x_type& array,
                               Eigen::VectorXd* x) {
  *x = Eigen::Map<const Eigen::VectorXd>(&(array[0]), array.size());
}

/// Converts an Eigen::VectorXd to a PolynomialSegment double array.
inline void msgArrayFromVector(const Eigen::VectorXd& x,
                               PolynomialSegment::_x_type* array) {
  array->resize(x.size());
  Eigen::Map<Eigen::VectorXd> map =
      Eigen::Map<Eigen::VectorXd>(&((*array)[0]), array->size());
  map = x;
}

/// Converts a PolynomialSegment message to an EigenPolynomialSegment structure.
inline void eigenPolynomialSegmentFromMsg(const PolynomialSegment& msg,
                                          EigenPolynomialSegment* segment) {
  assert(segment != NULL);

  vectorFromMsgArray(msg.x, &(segment->x));
  vectorFromMsgArray(msg.y, &(segment->y));
  vectorFromMsgArray(msg.z, &(segment->z));
  vectorFromMsgArray(msg.yaw, &(segment->yaw));
  vectorFromMsgArray(msg.rx, &(segment->rx));
  vectorFromMsgArray(msg.ry, &(segment->ry));
  vectorFromMsgArray(msg.rz, &(segment->rz));

  segment->segment_time_ns = msg.segment_time.toNSec();
  segment->num_coeffs = msg.num_coeffs;
}

/// Converts a PolynomialTrajectory message to a EigenPolynomialTrajectory
inline void eigenPolynomialTrajectoryFromMsg(
    const PolynomialTrajectory& msg,
    EigenPolynomialTrajectory* eigen_trajectory) {
  assert(eigen_trajectory != NULL);
  eigen_trajectory->clear();
  eigen_trajectory->reserve(msg.segments.size());
  for (PolynomialTrajectory::_segments_type::const_iterator it =
           msg.segments.begin();
       it != msg.segments.end(); ++it) {
    EigenPolynomialSegment segment;
    eigenPolynomialSegmentFromMsg(*it, &segment);
    eigen_trajectory->push_back(segment);
  }
}

/// Converts an EigenPolynomialSegment to a PolynomialSegment message. Does NOT
/// set the header!
inline void polynomialSegmentMsgFromEigen(const EigenPolynomialSegment& segment,
                                          PolynomialSegment* msg) {
  assert(msg != NULL);
  msgArrayFromVector(segment.x, &(msg->x));
  msgArrayFromVector(segment.y, &(msg->y));
  msgArrayFromVector(segment.z, &(msg->z));
  msgArrayFromVector(segment.yaw, &(msg->yaw));
  msgArrayFromVector(segment.rx, &(msg->rx));
  msgArrayFromVector(segment.ry, &(msg->ry));
  msgArrayFromVector(segment.rz, &(msg->rz));

  msg->segment_time.fromNSec(segment.segment_time_ns);
  msg->num_coeffs = segment.num_coeffs;
}

/// Converts an EigenPolynomialTrajectory to a PolynomialTrajectory message.
/// Does NOT set the header!
inline void polynomialTrajectoryMsgFromEigen(
    const EigenPolynomialTrajectory& eigen_trajectory,
    PolynomialTrajectory* msg) {
  assert(msg != NULL);
  msg->segments.reserve(eigen_trajectory.size());
  for (EigenPolynomialTrajectory::const_iterator it = eigen_trajectory.begin();
       it != eigen_trajectory.end(); ++it) {
    PolynomialSegment segment;
    polynomialSegmentMsgFromEigen(*it, &segment);
    msg->segments.push_back(segment);
  }
}

}  // namespace mav_planning_msgs

#endif // MAV_PLANNING_MSGS_CONVERSIONS_H
