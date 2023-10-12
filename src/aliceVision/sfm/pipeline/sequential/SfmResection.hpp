// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/track/Track.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/feature/FeaturesPerView.hpp>

namespace aliceVision {
namespace sfm {

class SfmResection
{
public:
    bool processView(
                const sfmData::SfMData & sfmData,
                const track::TracksMap & tracks,
                const track::TracksPerView & map_tracksPerView, 
                const feature::FeaturesPerView & featuresPerView,
                const IndexT viewId
            );
private:
    bool internalResection(
            const std::shared_ptr<camera::IntrinsicBase> & intrinsic,
            const std::vector<Eigen::Vector3d> & structure,
            const std::vector<Eigen::Vector2d> & observations
        );
private:
};

} // namespace sfm
} // namespace aliceVision

