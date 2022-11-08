
/*
This file is part of MapSelect

Copyright (C) 2022  Christiaan Johann Müller

MapSelect is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version."

MapSelect is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details."

"You should have received a copy of the GNU General Public License"
"along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OSMAP_ALIAS_H
#define OSMAP_ALIAS_H

#include "mapselect/maps/MapDataAlias.h"
#include "mapselect/maps/OSMapData.h"


class OSMapDataAlias : public MapDataAlias
{
private:

    
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

public:

    OSMapDataAlias(const OSMapData &data, bool allow_duplicates = true) : MapDataAlias(extractFrameIds(data), extractMapIds(data))
    {

        // reserve & zero initialise vectors
        size_t N_frames = data.frames.size();
        size_t N_map = data.mappoints.size();

        frame_obs.resize(N_frames);
        frame_keypoints.resize(N_frames);
        map_obs.resize(N_map);
        map_keypoints.resize(N_map);


        std::vector<size_t> map_obs_cnt;
        map_obs_cnt.resize(N_map);
        

        size_t duplicates = 0;

        for (auto frame : data.frames)
        {
            size_t frame_key = frame_alias->id2key(frame.frame_id);
            std::vector<size_t> obs_keys = map_alias->ids2keys(frame.obs);

            std::vector<size_t> idx;
            idx.reserve(frame.obs.size());
            for (size_t i = 0U; i < obs_keys.size(); i++)
            {
                idx.push_back(i);
            }

            std::sort(idx.begin(), idx.end(), [obs_keys](const auto &lhs, const auto &rhs) {
                return obs_keys[lhs] < obs_keys[rhs];
            });

            if (!allow_duplicates)
            {
                auto iter = std::unique(idx.begin(), idx.end(), [obs_keys](const auto &lhs, const auto &rhs) {
                    return obs_keys[lhs] == obs_keys[rhs];
                });
                duplicates = idx.end() - iter;
                idx.resize(iter - idx.begin());
            }

            frame_obs[frame_key].reserve(obs_keys.size());
            frame_keypoints[frame_key].reserve(obs_keys.size());

            // now use it to sort obs_keys & corresponding keypoints - as well as the map point observations
            for (size_t i : idx)
            {
                frame_obs[frame_key].push_back(obs_keys[i]);
                auto &keypoint = frame.keypoints[i];

                frame_keypoints[frame_key].emplace_back(keypoint.px, keypoint.py, keypoint.disp);
                map_obs_cnt[obs_keys[i]]++;

                //map_obs[obs_keys[i]].push_back(frame_key);
                //map_keypoints[obs_keys[i]].push_back(frame_keypoints[frame_key].back());
            }
        }


        for(size_t i=0; i < N_map;i++ )
        {
            map_obs[i].reserve(map_obs_cnt[i]);
            map_keypoints[i].reserve(map_obs_cnt[i]);
        }

        for(size_t i=0; i < N_frames;i++)
        {
            for(size_t j=0; j < frame_obs[i].size();j++)
            {
                map_obs[frame_obs[i][j]].push_back(i);
                map_keypoints[frame_obs[i][j]].push_back(frame_keypoints[i][j]);
            }
            
        }

        


        for(size_t i=0, N = data.protoc_keyframesArray.keyframe_size(); i < N; i++)
        {   
            const auto& protoc_frame = data.protoc_keyframesArray.keyframe(i);
            const auto& protoc_pose = protoc_frame.pose();
            double fx,fy,cx,cy,time_stamp,bf = std::numeric_limits<double>::quiet_NaN();
            if (protoc_frame.has_kmatrix())
            {
                const auto& protoc_K = protoc_frame.kmatrix();
                fx = protoc_K.fx();
                fy = protoc_K.fy();
                cx = protoc_K.cx();
                cy = protoc_K.cy();
            }
            else
            {
                size_t j = protoc_frame.kindex();
                fx = data.fx[j];
                fy = data.fy[j];
                cx = data.cx[j];
                cy = data.cy[j];
                bf = data.baseline[j]*fx;
            }
            time_stamp = protoc_frame.timestamp();

            Mat34 pose;
            pose << 
            protoc_pose.element(0),protoc_pose.element(1),protoc_pose.element(2),protoc_pose.element(3),
            protoc_pose.element(4),protoc_pose.element(5),protoc_pose.element(6),protoc_pose.element(7),
            protoc_pose.element(8),protoc_pose.element(9),protoc_pose.element(10),protoc_pose.element(11);
            frames.emplace_back(std::move(pose),fx,fy,cx,cy,bf,time_stamp);

            //frames.back().print();
        }

        for(size_t i=0, N = data.protoc_mappointsArray.mappoint_size(); i < N;i++)
        {
            const auto& protoc_mappoint = data.protoc_mappointsArray.mappoint(i);
            const auto& protoc_pos = protoc_mappoint.position();
            // double x,double y,double z, double vis
            //double pvis = (protoc_mappoint.found()+1.0) / (+2.0);

            size_t found_unit =  (size_t) protoc_mappoint.found();
            size_t vis_unit = (size_t) protoc_mappoint.visible();
            //std::cout << "Found ("<< i<< ")" << protoc_mappoint.found() << ":" << found_unit << std::endl;


            mappoints.emplace_back(protoc_pos.x(),protoc_pos.y(),protoc_pos.z(),found_unit,vis_unit );
        }


        if (duplicates)
        {
            std::cerr << "Warning : " << duplicates << " observations was ignored due to the same point begin observed more than once per frame!" << std::endl;
        }


        std::vector<size_t> tracking_ids; 

        for(size_t i=0, N = data.protoc_trackingArray.track_feature_size();i<N;i++)
        {
            const auto& feature = data.protoc_trackingArray.track_feature(i);
            if (feature.mappoint_id())
                tracking_ids.push_back(feature.mappoint_id());
        }
        
        
        tracking_points = map_alias->ids2keys(tracking_ids);
    }


};


#endif