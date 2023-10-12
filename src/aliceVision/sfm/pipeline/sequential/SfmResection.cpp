// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SfmResection.hpp"

#include <aliceVision/track/tracksUtils.hpp>
#include <aliceVision/stl/mapUtils.hpp>
#include <aliceVision/robustEstimation/ACRansac.hpp>

namespace aliceVision {
namespace sfm {

bool SfmResection::processView(
                        const sfmData::SfMData & sfmData,
                        const track::TracksMap & tracks,
                        const track::TracksPerView & tracksPerView, 
                        const feature::FeaturesPerView & featuresPerView,
                        const IndexT viewId
                        )
{
    // A. Compute 2D/3D matches
    // A1. list tracks ids used by the view
    const aliceVision::track::TrackIdSet& set_tracksIds = tracksPerView.at(viewId);

    // A2. Each landmark's id is equal to the associated track id
    // Get list of landmarks = get list of reconstructed tracks
    std::set<std::size_t> reconstructedTrackId;
    std::transform(sfmData.getLandmarks().begin(), sfmData.getLandmarks().end(),
                 std::inserter(reconstructedTrackId, reconstructedTrackId.begin()),
                 stl::RetrieveKey());

    // Remove all reconstructed tracks which were not observed in the view to resect.
    // --> Intersection of tracks observed in this view and tracks reconstructed.
    std::set<std::size_t> tracksId;
    std::set_intersection(tracksId.begin(), tracksId.end(),
                        reconstructedTrackId.begin(),
                        reconstructedTrackId.end(),
                        std::inserter(tracksId, tracksId.begin()));


    if (tracksId.size() < 3)
    {
        // If less than 3 points, the resection is theorically impossible.
        // Let ignore this view.
        return false;
    }

    // Associate feature id to each track
    // Feature id is the id of the observation of this track in the current view
    std::vector<track::FeatureId> featuresId;
    track::getFeatureIdInViewPerTrack(tracks, tracksId, viewId, featuresId);


    //Get information about this view
    const std::shared_ptr<sfmData::View> view = sfmData.getViews().at(viewId);
    const std::shared_ptr<camera::IntrinsicBase> intrinsic = sfmData.getIntrinsicsharedPtr(view->getIntrinsicId());

    //Loop over features and tracks to build data needed by resection process
    std::vector<Eigen::Vector3d> structure;
    std::vector<Eigen::Vector2d> observations;
    auto itFeatures = featuresId.begin();
    auto itTracks = tracksId.begin();
    for (; itFeatures != featuresId.end() && itTracks != tracksId.end(); itFeatures++, itTracks++)
    {
        const feature::EImageDescriberType descType = itFeatures->first;
        const std::size_t trackId = *itTracks;
        const IndexT featureId = itFeatures->second;
        
        const Eigen::Vector3d X = sfmData.getLandmarks().at(trackId).X;
        const Eigen::Vector2d x = featuresPerView.getFeatures(viewId, descType)[featureId].coords().cast<double>();

        structure.push_back(X);
        observations.push_back(x);
    }

    if (!internalResection(intrinsic, structure, observations))
    {
        return false;
    }

    return true;
}

bool SfmResection::internalResection(
            const std::shared_ptr<camera::IntrinsicBase> & intrinsic,
            const std::vector<Eigen::Vector3d> & structure,
            const std::vector<Eigen::Vector2d> & observations
        )
{
    
}

} // namespace sfm
} // namespace aliceVision

