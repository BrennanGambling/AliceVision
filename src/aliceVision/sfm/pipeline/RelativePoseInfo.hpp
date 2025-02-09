// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/geometry/Pose3.hpp>

#include <vector>
#include <random>

namespace aliceVision {
namespace sfm {

/**
* @brief Estimate the best possible Rotation/Translation from E.
*  Four are possible, keep the one with most of the point in front.
*
* @param[in] K1 camera 1 intrinsics
* @param[in] K2 camera 2 intrinsics
* @param[in] x1 camera 1 image points
* @param[in] x2 camera 2 image points
* @param[in] E essential matrix
* @param[in] vec_inliers inliers indices
* @param[out] R estimated rotation
* @param[out] t estimated translation
*/
bool estimate_Rt_fromE
(
  const Mat3 & K1, const Mat3 & K2,
  const Mat & x1, const Mat & x2,
  const Mat3 & E, const std::vector<size_t> & vec_inliers,
  Mat3 & R, Vec3 & t
);

struct RelativePoseInfo
{
  Mat3 essential_matrix;
  geometry::Pose3 relativePose;
  std::vector<size_t> vec_inliers;
  double initial_residual_tolerance;
  double found_residual_precision;

  RelativePoseInfo()
    :initial_residual_tolerance(std::numeric_limits<double>::infinity()),
    found_residual_precision(std::numeric_limits<double>::infinity())
  {}
};

/**
 * @brief Estimate the Relative pose between two views from point matches and K matrices
 *  by using a robust essential matrix estimation.
 *
 * @param[in] K1 camera 1 intrinsics
 * @param[in] K2 camera 2 intrinsics
 * @param[in] x1 camera 1 image points
 * @param[in] x2 camera 2 image points
 * @param[in] randomNumberGenerator random number generator
 * @param[out] relativePose_info relative pose information
 * @param[in] size_ima1 width, height of image 1
 * @param[in] size_ima2 width, height of image 2
 * @param[in] max iteration count
 */
bool robustRelativePose
(
  const Mat3 & K1, const Mat3 & K2,
  const Mat & x1, const Mat & x2,
  std::mt19937 &randomNumberGenerator,
  RelativePoseInfo & relativePose_info,
  const std::pair<size_t, size_t> & size_ima1,
  const std::pair<size_t, size_t> & size_ima2,
  const size_t max_iteration_count = 4096
);

} // namespace sfm
} // namespace aliceVision
