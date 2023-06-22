/*
 * File: OSMapAlias.h
 * Project: MapSelect
 * Author: Christiaan Johann Müller 
 * -----
 * This file is part of MapSelect
 * 
 * Copyright (C) 2022 - 2023  Christiaan Johann Müller
 * 
 * MapSelect is free software: you can redistribute it and/or modify
 * 
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * MapSelect is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef MAPSELECT_OSMAP_ALIAS_H
#define MAPSELECT_OSMAP_ALIAS_H

#include "mapselect/maps/MapDataAlias.h"
#include "mapselect/maps/OSMapData.h"
#include "mapselect/utils/Timing.h"

namespace mselect
{
    /*! @brief This Class is used create a MapDataAlias when given a OSMAP file
     */
    class OSMapDataAlias : public MapDataAlias
    {
    private:
        /*
        static std::vector<size_t> extractFrameIds(const OSMapData &data)
        {
            std::vector<size_t> frame_ids;
            frame_ids.reserve(data.frames.size());
            for (auto frame : data.frames)
            {
                frame_ids.push_back(frame.frame_id);
            }
            return frame_ids;
        }

        static std::vector<size_t> extractMapIds(const OSMapData &data)
        {
            std::vector<size_t> map_ids;
            map_ids.reserve(map_ids.size());
            for (auto mappoint : data.mappoints)
            {
                map_ids.push_back(mappoint.map_id);
            }
            return map_ids;
        }
        */

    public:
        OSMapDataAlias(const OSMapData &data, bool allow_duplicates = true) : MapDataAlias(data.keyframe_ids, data.mappoint_ids)
        {

            // reserve & zero initialise vectors
            // size_t N_frames = data.nKeyframes; //frames.size();
            // size_t N_map = data.nMappoints; //mappoints.size();

            frame_obs.resize(n_frames);
            // frame_keypoints.resize(N_frames);
            map_obs.resize(n_points);
            map_keypoints.resize(n_points);
            size_t duplicates = 0;
            size_t feature_frames = data.protoc_featuresArray.feature_size();

            // Read and build KF obs
            for (size_t frame_i = 0; frame_i < feature_frames; frame_i++)
            {
                const SerializedKeyframeFeatures &keyframe = data.protoc_featuresArray.feature(frame_i);
                size_t frame_id = keyframe.keyframe_id();
                size_t frame_key = frame_alias->id2key(frame_id);

                typedef std::pair<size_t,AliasKeypoint> Key_KeyPointPair;

                std::vector<Key_KeyPointPair> tmp_frame_data;

                for (size_t feature_j = 0; feature_j < keyframe.feature_size(); feature_j++)
                {
                    const SerializedFeature &feature = keyframe.feature(feature_j);

                    // only use data from map point observations
                    size_t map_id = feature.mappoint_id();
                    if (!map_id)
                        continue;

                    // key must exist
                    size_t map_key = map_alias->id2key(map_id);
                    if (map_key == ID_ERROR)
                        continue;

                    // copy relevant information to Alias Datastructure
                    const SerializedKeypoint &keypoint = feature.keypoint();
                    double px = keypoint.ptx();
                    double py = keypoint.pty();
                    size_t oct = (size_t)floor(keypoint.octave());
                    double disp = -1.0;
                    if (feature.uright())
                        disp = keypoint.ptx() - feature.uright();

                    AliasKeypoint alias_keypoint(px, py, disp, oct);
                    
                    tmp_frame_data.emplace_back(map_key,alias_keypoint);
                }
                //! @todo this sort is needed since some algorithms are ordering dependent 
                //! @todo investigate this ^
                std::sort(tmp_frame_data.begin(),tmp_frame_data.end(),[](const Key_KeyPointPair& a, const Key_KeyPointPair& b) {return a.first < b.first;} );
                
                for(auto pair : tmp_frame_data)
                {
                    size_t map_key = pair.first;
                    AliasKeypoint keypoint = pair.second;

                    map_obs[map_key].push_back(frame_key);
                    map_keypoints[map_key].push_back(pair.second);
                    frame_obs[frame_key].push_back(map_key);
                }

                //map_obs[map_key].push_back(frame_key);
                //    map_keypoints[map_key].push_back(std::move(alias_keypoint));
                //   frame_obs[frame_key].push_back(map_key);
            }

            // sort frame obs
            // maps obs should already be in sorted order
            //for (size_t i = 0; i < n_frames; i++)
            //{
            //! @todo is it still needed to sort observations?
            //    std::sort(frame_obs[i].begin(), frame_obs[i].end());
            //}

            /*
            std::vector<std::vector<size_t>> alt_frame_obs;
            std::vector<AliasKeypointVector> alt_frame_keypoints;
            std::vector<std::vector<size_t>> alt_map_obs;
            std::vector<AliasKeypointVector> alt_map_keypoints;

            alt_frame_obs.resize(n_frames);
            alt_frame_keypoints.resize(n_frames);
            alt_map_obs.resize(n_points);
            alt_map_keypoints.resize(n_points);

            for (auto frame_data : data.frames)
            {
                size_t frame_key = frame_alias->id2key(frame_data.frame_id);
                std::vector<size_t> obs_keys = map_alias->ids2keys(frame_data.obs);

                std::vector<size_t> idx;
                idx.reserve(frame_data.obs.size());
                for (size_t i = 0U; i < obs_keys.size(); i++)
                {
                    idx.push_back(i);
                }

                std::sort(idx.begin(), idx.end(), [obs_keys](const auto &lhs, const auto &rhs)
                          { return obs_keys[lhs] < obs_keys[rhs]; });

                if (!allow_duplicates)
                {
                    auto iter = std::unique(idx.begin(), idx.end(), [obs_keys](const auto &lhs, const auto &rhs)
                                            { return obs_keys[lhs] == obs_keys[rhs]; });
                    duplicates = idx.end() - iter;
                    idx.resize(iter - idx.begin());
                }

                alt_frame_obs[frame_key].reserve(obs_keys.size());
                // frame_keypoints[frame_key].reserve(obs_keys.size());

                // now use it to sort obs_keys & corresponding keypoints - as well as the map point observations
                for (size_t i : idx)
                {
                    alt_frame_obs[frame_key].push_back(obs_keys[i]);
                    auto &keypoint = frame_data.keypoints[i];

                    alt_frame_keypoints[frame_key].emplace_back(keypoint.px, keypoint.py, keypoint.disp, keypoint.oct);
                    // map_obs_cnt[obs_keys[i]]++;

                    // map_obs[obs_keys[i]].push_back(frame_key);
                    // map_keypoints[obs_keys[i]].push_back(frame_keypoints[frame_key].back());
                }
            }

            for (size_t i = 0; i < n_points; i++)
            {
                // alt_map_obs[i].reserve(map_obs_cnt[i]);
                // alt_map_keypoints[i].reserve(map_obs_cnt[i]);
            }

            for (size_t i = 0; i < n_frames; i++)
            {
                for (size_t j = 0; j < alt_frame_obs[i].size(); j++)
                {
                    alt_map_obs[alt_frame_obs[i][j]].push_back(i);
                    alt_map_keypoints[alt_frame_obs[i][j]].push_back(alt_frame_keypoints[i][j]);
                }
            }

            for (size_t i = 0; i < n_points; i++)
            {
                assert(map_obs[i].size() == alt_map_obs[i].size());
                for (size_t j = 0; j < map_obs[i].size(); j++)
                {
                    if (alt_map_obs[i][j] != map_obs[i][j])
                    {
                        std::cout << alt_map_obs[i][j] << "!=" << map_obs[i][j] << std::endl;
                    }


                    if(!(alt_map_keypoints[i][j] == map_keypoints[i][j]))
                    {
                        alt_map_keypoints[i][j].print();
                        map_keypoints[i][j].print();
                        assert(false);
                        continue;

                    }
                }
            } */

            frames.reserve(n_frames);
            for (size_t i = 0, N = data.protoc_keyframesArray.keyframe_size(); i < N; i++)
            {
                const auto &protoc_frame = data.protoc_keyframesArray.keyframe(i);
                const auto &protoc_pose = protoc_frame.pose();
                double fx, fy, cx, cy, time_stamp, bf = std::numeric_limits<double>::quiet_NaN();
                if (protoc_frame.has_kmatrix())
                {
                    const auto &protoc_K = protoc_frame.kmatrix();
                    fx = protoc_K.fx();
                    fy = protoc_K.fy();
                    cx = protoc_K.cx();
                    cy = protoc_K.cy();
                    assert(data.baseline.size() >= 1);
                    bf = data.baseline[0] * fx;
                }
                else
                {
                    size_t j = protoc_frame.kindex();
                    fx = data.fx[j];
                    fy = data.fy[j];
                    cx = data.cx[j];
                    cy = data.cy[j];
                    bf = data.baseline[j] * fx;
                }
                time_stamp = protoc_frame.timestamp();

                Mat34 pose;
                pose(0, 0) = protoc_pose.element(0);
                pose(0, 1) = protoc_pose.element(1);
                pose(0, 2) = protoc_pose.element(2);
                pose(0, 3) = protoc_pose.element(3);
                pose(1, 0) = protoc_pose.element(4);
                pose(1, 1) = protoc_pose.element(5);
                pose(1, 2) = protoc_pose.element(6);
                pose(1, 3) = protoc_pose.element(7);
                pose(2, 0) = protoc_pose.element(8);
                pose(2, 1) = protoc_pose.element(9);
                pose(2, 2) = protoc_pose.element(10);
                pose(2, 3) = protoc_pose.element(11);
                /*
                pose <<
                    protoc_pose.element(0), protoc_pose.element(1), protoc_pose.element(2), protoc_pose.element(3),
                    protoc_pose.element(4), protoc_pose.element(5), protoc_pose.element(6), protoc_pose.element(7),
                    protoc_pose.element(8), protoc_pose.element(9), protoc_pose.element(10), protoc_pose.element(11);
                */
                frames.emplace_back(pose, fx, fy, cx, cy, bf, time_stamp);
            }

            mappoints.reserve(n_points);
            for (size_t i = 0, N = data.protoc_mappointsArray.mappoint_size(); i < N; i++)
            {
                const auto &protoc_mappoint = data.protoc_mappointsArray.mappoint(i);
                const auto &protoc_pos = protoc_mappoint.position();
                // double x,double y,double z, double vis
                // double pvis = (protoc_mappoint.found()+1.0) / (+2.0);

                size_t found_unit = (size_t)protoc_mappoint.found();
                size_t vis_unit = (size_t)protoc_mappoint.visible();
                // std::cout << "Found ("<< i<< ")" << protoc_mappoint.found() << ":" << found_unit << std::endl;

                mappoints.emplace_back(protoc_pos.x(), protoc_pos.y(), protoc_pos.z(), found_unit, vis_unit);
            }

            if (duplicates)
            {
                std::cerr << "Warning : " << duplicates << " observations was ignored due to the same point begin observed more than once per frame!" << std::endl;
            }

            std::vector<size_t> tracking_ids;

            for (size_t i = 0, N = data.protoc_trackingArray.track_feature_size(); i < N; i++)
            {
                const auto &feature = data.protoc_trackingArray.track_feature(i);
                if (feature.mappoint_id())
                    tracking_ids.push_back(feature.mappoint_id());
            }

            tracking_points = map_alias->ids2keys(tracking_ids);

            mappoint_base_cost = 67;
            mappoint_obs_cost = 62;
        }
    };
}

#endif