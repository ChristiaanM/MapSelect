
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

typedef Eigen::Matrix<double, 3, 1> Mat21;
typedef Eigen::Matrix<double, 3, 1> Mat31;
typedef Eigen::Matrix<double, 4, 1> Mat41;
typedef Eigen::Matrix<double, 6, 1> Mat61;

typedef Mat21 Vect2;
typedef Mat31 Vect3;
typedef Mat41 Vect4;
typedef Mat61 Vect6;

typedef Eigen::Matrix<double, 2, 2, Eigen::RowMajor> Mat22;
typedef Eigen::Matrix<double, 2, 2, Eigen::RowMajor> Mat23;
typedef Eigen::Matrix<double, 2, 6, Eigen::RowMajor> Mat26;

typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Mat33;
typedef Eigen::Matrix<double, 3, 4, Eigen::RowMajor> Mat34;
typedef Eigen::Matrix<double, 3, 6, Eigen::RowMajor> Mat36;

typedef Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Mat66;


struct AliasKeypoint
{
    AliasKeypoint(double px, double py, double disp = -1.0, size_t oct = 0) : px(px), py(py), disp(disp), oct(oct)
    {
    }

    double px, py, disp, oct;
};

struct AliasMappoint
{
    double x, y, z;
    size_t track_found;
    size_t track_vis;

    AliasMappoint(double x, double y, double z, size_t track_found = 0U, size_t track_vis = 0U) : //
     x(x), y(y), z(z), track_found(track_found), track_vis(track_vis)
    {
    }

    Mat41 hcords() const
    {
        return Mat41(x, y, z, 1.0);
    }

    double pvis(size_t alpha = 0U, size_t beta = 0U) const
    {
        return (alpha + track_found) / ((double)(beta + track_vis));
    }
};

struct AliasFrame
{

    double fx, fy, cx, cy, bf;
    double time_stamp;
    Mat34 Tcw;
    Mat34 P;

    AliasFrame(const Mat34 &pose, double fx, double fy, double cx = 0.0, double cy = 0.0, double bf = -1.0, double time_stamp = 0.0) : //
    Tcw(pose), fx(fx), fy(fy), cx(cx), cy(cy), bf(bf), time_stamp(time_stamp)
    {
        Mat33 C;
        C << fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0;
        P = C * Tcw;
    }

    Vect2 proj_mono(const AliasMappoint &p)
    {
        Mat31 pcam = P * p.hcords();
        Vect2 out;
        out << pcam(0, 0) / pcam(2, 0), pcam(0, 1) / pcam(2, 0);
        return out;
    }

    Vect3 local_cords(const AliasMappoint &p) const
    {
        Vect3 tmp = Tcw.block<3, 3>(0, 0) * Vect3(p.x, p.y, p.z);
        return tmp + Tcw.block<3, 1>(0, 3);
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
};

class MapDataAlias
{
public:
    MapDataAlias(std::vector<size_t> &&frame_ids, std::vector<size_t> &&map_ids)
    {
        frame_alias = std::make_unique<AliasMap>(frame_ids);
        map_alias = std::make_unique<AliasMap>(map_ids);
        n_frames = frame_ids.size();
        n_points = map_ids.size();
    }

    MapDataAlias(const std::vector<size_t> &frame_ids, const std::vector<size_t> &map_ids)
    {
        frame_alias = std::make_unique<AliasMap>(frame_ids);
        map_alias = std::make_unique<AliasMap>(map_ids);
        n_frames = frame_ids.size();
        n_points = map_ids.size();
    }

    MapDataAlias(size_t frames, size_t points)
    {
        frame_alias = std::make_unique<AliasIdentity>(frames);
        map_alias = std::make_unique<AliasIdentity>(points);
        n_frames = frames;
        n_points = points;
    }

    inline size_t getLastFrameId() const { return frame_alias->getLastId(); }

    inline const std::vector<size_t> &mapKeyToFrameObs(size_t key) const
    {
        assert(key < map_obs.size());
        return map_obs[key];
    }
    inline const std::vector<size_t> &frameKeyToMapObs(size_t key) const
    {
        assert(key < frame_obs.size());
        return frame_obs[key];
    }
    inline const std::vector<AliasKeypoint> &getKeypointsFromKey(size_t key) const { return frame_keypoints[key]; }
    inline size_t getMapSize() const { return map_alias->size(); }
    inline size_t getFrameSize() const { return frame_alias->size(); }

    inline double getMappointPvis(size_t key, size_t alpha = 0U, size_t beta = 0U) const
    {
        return mappoints[key].pvis(alpha, beta);
    }

    /*
        Seperate mappoints keys into two sets: split_keys contains all the points seen in the last frames
        while remain keys contains the rest. 

    */

    void lastFramesHeuristic(size_t frames, std::vector<size_t> &remain_keys, std::vector<size_t> &split_keys)
    {
        if (tracking_points.size() == 0 && frames == 0)
        {
            remain_keys.resize(n_points);
            std::iota(remain_keys.begin(), remain_keys.end(), 0);
            split_keys = std::vector<size_t>();
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
        inverse_keyvector(all_keys, remain_keys, split_keys, n_points);
   
    }

    std::vector<size_t> dealias_mappoints(const std::vector<size_t> &keys)
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

    // protected:
    std::unique_ptr<Alias> map_alias, frame_alias;
    std::vector<std::vector<size_t>> map_obs, frame_obs;
    std::vector<std::vector<AliasKeypoint>> frame_keypoints;
    std::vector<std::vector<AliasKeypoint>> map_keypoints;

    std::vector<size_t> tracking_points;

    std::vector<AliasFrame> frames;
    std::vector<AliasMappoint> mappoints;

    size_t n_frames;
    size_t n_points;

    // std::vector<double> map_pvis;
};

#endif