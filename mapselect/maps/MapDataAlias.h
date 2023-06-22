/*
 * File: MapDataAlias.h
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



#ifndef MAP_DATA_ALIAS_H
#define MAP_DATA_ALIAS_H

#include <mapselect/maps/Alias.h>
#include <mapselect/utils/KeyVectorUtils.h>

#include <assert.h>
#include <algorithm>
#include <set>
#include <map>
#include <memory>
#include <deque>
#include <iostream>
#include <bitset>
#include <numeric> // iota

#include <Eigen/Core>

namespace mselect
{

    // col major is default for Eigen & GTSAM by extention, but is included here explicitly for clarity
    typedef Eigen::Matrix<double, -1, -1, Eigen::ColMajor> MatXX;

    typedef Eigen::Matrix<double, 3, 1, Eigen::ColMajor> Mat21;
    typedef Eigen::Matrix<double, 3, 1, Eigen::ColMajor> Mat31;
    typedef Eigen::Matrix<double, 3, 4, Eigen::ColMajor> Mat34;

    typedef Eigen::Matrix<double, 4, 1, Eigen::ColMajor> Mat41;
    typedef Eigen::Matrix<double, 6, 1, Eigen::ColMajor> Mat61;
    typedef Eigen::Matrix<double, 4, 1, Eigen::ColMajor> Mat44;
    typedef Eigen::Matrix<double, 2, 2, Eigen::ColMajor> Mat22;
    typedef Eigen::Matrix<double, 2, 3, Eigen::ColMajor> Mat23;
    typedef Eigen::Matrix<double, 2, 6, Eigen::ColMajor> Mat26;
    typedef Eigen::Matrix<double, 3, 3, Eigen::ColMajor> Mat33;

    typedef Eigen::Matrix<double, 3, 6, Eigen::ColMajor> Mat36;
    typedef Eigen::Matrix<double, 6, 3, Eigen::ColMajor> Mat63;
    typedef Eigen::Matrix<double, 6, 6, Eigen::ColMajor> Mat66;

    // todo:: Investigate if this leads to meaningful speedups for pose * point multiplication
    typedef Eigen::Matrix<double, 3, 4, Eigen::RowMajor> Mat34r;

    typedef Mat66 PoseInfoMat;
    typedef Eigen::Matrix<double,6,6,Eigen::DontAlign> Mat66Unaligned; 
    typedef std::vector<PoseInfoMat, Eigen::aligned_allocator<PoseInfoMat>> PoseMatVector;


    class AliasKeypoint
    {
    public:
        AliasKeypoint(float px, float py, float disp = -1.0, unsigned short oct = 0) : px(px), py(py), disp(disp), oct(oct)
        {
        }

        bool operator==(const AliasKeypoint& rhs)
        {
            return (px == rhs.px) && (py == rhs.py) &&  (disp == rhs.disp) && (oct == rhs.oct);
        }

        void print()
        {
            std::cout << "Keypoint" << std::endl 
            << "px   "<< px << std::endl
            << "py   "<< py << std::endl
            << "disp "<< disp << std::endl 
            << "oct  "<< oct << std::endl; 
        }

        float px, py, disp;
        unsigned short oct;
    };

    class AliasMappoint
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        AliasMappoint(double x, double y, double z, unsigned short found = 0U, unsigned short vis = 0U) 
        {
            pos << x, y, z;
            track_found = found;
            track_vis = vis;
        }

        double pvis(unsigned short alpha = 0U, unsigned short beta = 0U) const
        {
            return (alpha + track_found) / ((double)(beta + track_vis));
        }

        double x() { return pos(0); }
        double y() { return pos(1); }
        double z() { return pos(2); }

        void print()
        {
                    std::cout << "Mappoint Alias" << std::endl
                      << "found" << track_found << std::endl
                      << "vis" << track_vis << std::endl
                      << "pos:" << std::endl
                      << pos<< std::endl;
        }

        
        Mat31 pos;
        unsigned short track_found;
        unsigned short track_vis;
    };

    class AliasFrame
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        AliasFrame(const Mat34 &Tcw_, double fx_, double fy_, double cx_ = 0.0, double cy_ = 0.0, double bf_ = -1.0, double time_stamp_ = 0.0)                                                                                                                                          
        {
            Tcw = Tcw_;
            fx = fx_;
            fy = fy_;
            cx = cx_; 
            cy = cy_; 
            bf = bf_; 
            time_stamp = time_stamp_;
        }

        void print()
        {
            std::cout << "Frame Alias" << std::endl
                      << "fx: " << fx << std::endl
                      << "fy: " << fy << std::endl
                      << "cx: " << cx << std::endl
                      << "cy: " << cy << std::endl
                      << "bf :" << bf << std::endl
                      << "timestamp" << time_stamp << std::endl
                      << "Tcw:" << std::endl
                      << Tcw << std::endl;
        }

        
        Mat34 Tcw;
        double fx, fy, cx, cy, bf;
        double time_stamp;
    };

    /*! @brief Data structure used to store a tempory copy of relevant SLAM
    information for map point selection.

    This data structure stores a copy of the relevant data for map point point
    selection (that is map point observations) and poses in the trajectory. By
    design, mapselect assumes all map points and poses have unique numerical
    indices called (keys), assigned to each keyframe and map point from 0 to
    n-1. This is opposed to IDs the ids assigned during creation. To apply any
    of the map point selection algorithms, you must first repack all the
    relevant data into a MapDataAlias. 

    See OSmapData.h as an example.
    */
    typedef std::vector<AliasKeypoint> AliasKeypointVector;
    typedef std::vector<AliasFrame,Eigen::aligned_allocator<AliasFrame>> AliasFrameVector;
    typedef std::vector<AliasMappoint,Eigen::aligned_allocator<AliasMappoint>> AliasMappointVector;

    class MapDataAlias
    {
    public:
    
        MapDataAlias(std::vector<size_t> &&frame_ids, std::vector<size_t> &&map_ids) : frame_alias(new AliasMap(frame_ids)),
                                                                                       map_alias(new AliasMap(map_ids))
        {
            n_frames = frame_ids.size();
            n_points = map_ids.size();
        }

        MapDataAlias(const std::vector<size_t> &frame_ids, const std::vector<size_t> &map_ids) : frame_alias(new AliasMap(frame_ids)),
                                                                                                 map_alias(new AliasMap(map_ids))
        {
            n_frames = frame_ids.size();
            n_points = map_ids.size();
        }

        MapDataAlias(size_t frames, size_t points) : frame_alias(new AliasIdentity(frames)),
                                                     map_alias(new AliasIdentity(points))
        {
            n_frames = frames;
            n_points = points;
        }

        inline size_t getLastFrameId() const { return frame_alias->getLastId(); }

        inline const std::vector<size_t> &getMappointObs(size_t key) const
        {
            assert(key < map_obs.size());
            return map_obs[key];
        }
        inline const std::vector<size_t> &getFrameObs(size_t key) const
        {
            assert(key < frame_obs.size());
            return frame_obs[key];
        }
        /*inline const std::vector<AliasKeypoint> &getFrameObsKeypoints(size_t key) const
        {
            assert(key < frame_keypoints.size());
            return frame_keypoints[key];
        }*/
        inline const std::vector<AliasKeypoint> &getMappointObsKeypoints(size_t key) const
        {
            assert(key < map_keypoints.size());
            return map_keypoints[key];
        }
        inline size_t getMapSize() const { return map_alias->size(); } 
        inline size_t getFrameSize() const { return frame_alias->size(); }
        inline double getMappointPvis(size_t key, size_t alpha = 0U, size_t beta = 0U) const
        {
            return mappoints[key].pvis(alpha, beta);
        }
        inline double getMappointCost(size_t key) const
        {
            return mappoint_base_cost + mappoint_obs_cost* map_obs[key].size(); 
        }


        /*
            Seperate mappoints keys into two sets: split_keys contains all the points seen in the last frames
            while remain keys contains the rest.

        */

        void lastFramesHeuristic(size_t frames, std::vector<size_t> &eval_keys, std::vector<size_t> &heuristic_keys)
        {
            if (tracking_points.size() == 0 && frames == 0)
            {
                eval_keys.resize(n_points);
                std::iota(eval_keys.begin(), eval_keys.end(), 0);
                heuristic_keys = std::vector<size_t>();
                return;
            }

            std::vector<size_t> all_keys;
            size_t size = tracking_points.size();
            for (size_t i = n_frames - frames; i < n_frames; i++)
                size += frame_obs[i].size();
            all_keys.reserve(size);
            std::copy(tracking_points.begin(), tracking_points.end(), std::back_inserter(all_keys));

            for (size_t i = n_frames - frames; i < n_frames; i++)
            {
                std::copy(frame_obs[i].begin(), frame_obs[i].end(), std::back_inserter(all_keys));
            }
            inverse_keyvector(all_keys, eval_keys, heuristic_keys, n_points);
        }

        std::vector<size_t> convertMappointKeysToIDs(const std::vector<size_t> &keys)
        {
            return map_alias->keys2ids(keys);
        }

        // covisibility counts, including total observations
        std::vector<std::map<size_t, size_t>> computeCovisCounts()
        {
            std::vector<std::map<size_t, size_t>> covis;
            covis.resize(getFrameSize());
            for (size_t i = 0, N = getMapSize(); i < N; i++)
            {
                for (size_t j = 0; j < map_obs[i].size(); j++)
                {
                    size_t ob0 = map_obs[i][j];
                    covis[ob0][ob0]++;

                    for (size_t k = j + 1; k < map_obs[i].size(); k++)
                    {

                        size_t ob1 = map_obs[i][k];

                        covis[ob0][ob1]++;
                        covis[ob1][ob0]++;
                    }
                }
            }
            return covis;
        }

        std::vector<std::vector<size_t>> computeThresholdCovis(double thresh)
        {
            std::vector<std::map<size_t, size_t>> covis = computeCovisCounts();
            std::vector<std::vector<size_t>> covis_vect;
            covis_vect.resize(getFrameSize());
            for (size_t i = 0, N = getFrameSize(); i < N; i++)
            {

                double frame_obs_threshold = frame_obs[i].size() * thresh;

                for (auto pair : covis[i])
                {
                    if (pair.second >= frame_obs_threshold)
                    {
                        covis_vect[i].push_back(pair.first);
                    }
                }
            }
            return covis_vect;
        }

        void pvis_covsibility(double threshold = 0.8)
        {
            std::vector<std::vector<size_t>> covis = computeThresholdCovis(threshold);
            // For every mappoint
            for (size_t i = 0, N = getMapSize(); i < N; i++)
            {
                // Get all frames from which that point is visible
                // And all covisible frames - with enough shared observations
                std::set<size_t> covis_frames;
                for (size_t ob0 : map_obs[i])
                {
                    covis_frames.insert(ob0);
                    covis_frames.insert(covis[ob0].begin(), covis[ob0].end());
                }

                // Assumption - Assume point should be visible from all those covisible frames
                mappoints[i].track_vis = covis_frames.size();
                mappoints[i].track_found = map_obs[i].size();
            }
        }



        /*! @todo: Make this protected again, this requires rewriting all the code that uses these members
        */
        // protected:
        std::unique_ptr<Alias> frame_alias, map_alias;
        std::vector<std::vector<size_t>> map_obs, frame_obs;
        //std::vector<AliasKeypointVector> frame_keypoints;
        std::vector<AliasKeypointVector> map_keypoints;
        
        std::vector<size_t> tracking_points;

        AliasFrameVector frames;
        AliasMappointVector mappoints;

        size_t n_frames;
        size_t n_points;
        double mappoint_base_cost;
        double mappoint_obs_cost;


        // std::vector<double> map_pvis;
    };

    
}

#endif