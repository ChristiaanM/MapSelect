/*
 * File: SLAMGraphGTSAM.h
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


#ifndef OPT_GTSAM_H
#define OPT_GTSAM_H

#include "mapselect/maps/MapDataAlias.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>

#include <string>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include "mapselect/slam/SLAMGraphGTSAMSettings.h"

namespace mselect
{

    /*! @brief Class for building map point selection approaches using GTSAM.

    This implemention offers an alternative to SLAMGraph, but includes additional overhead 
    due to internally using GTSAM and introduces an additional dependency. It is included 
    here for the purposes of replication of results generated for the SLAM approach.

    For the purposes of using these technique, it is advised to rather use SLAMGraph (without the GTSAM suffix)
     */
    class SLAMGraphGTSAM
    {
    public:
        SLAMGraphGTSAM();


        SLAMGraphGTSAM(std::shared_ptr<MapDataAlias> map_data_ptr, GTSAMGraphSettings gsettings = GTSAMGraphSettings());

        void buildSelectionLocalisation(PoseInfoMat &prior, std::vector<PoseMatVector> &landmark_factors);

        void buildSelectionOdometry(PoseInfoMat &prior, std::vector<PoseMatVector> &landmark_factors);

        void buildSelectionSLAMGraphFactors(gtsam::GaussianFactorGraph::shared_ptr base_graph, std::vector<gtsam::GaussianFactor::shared_ptr> &landmark_factors);

        // void buildSelectionSLAMGTGraph(std::vector<gtsam::GaussianFactorGraph::shared_ptr> &local_priors, std::vector<std::vector<gtsam::GaussianFactor::shared_ptr>> &local_factors);

        void buildSelectionSLAMMatrices(std::vector<MatXX> &landmark_A, std::vector<std::vector<size_t>> &landmark_keys);

        // void buildOffline(const std::vector<size_t> &eval_keys, size_t eval_frame, gtsam::NonlinearFactorGraph::shared_ptr out_graph, gtsam::Values::shared_ptr out_values, bool verbose = false, bool prior = false);

        gtsam::Values optimise(std::string out);

        // void offlineOptimise(const std::vector<size_t> &eval_keys, size_t eval_frame, std::string outpath);

        void writeTrajectory(gtsam::Values &out_values, const std::string out_path);

    private:
        gtsam::NonlinearFactor::shared_ptr obsFactor(size_t frame_key, size_t point_key, const AliasKeypoint &keypoint);
        gtsam::GaussianFactor::shared_ptr obsFactorLinear(size_t frame_key, size_t point_key, const AliasKeypoint &keypoint);
        bool isPointOk(size_t point_key);

    protected:
        void landmarkSelectionSLAMFactorsSmart(gtsam::GaussianFactorGraph::shared_ptr base_graph, std::vector<gtsam::GaussianFactor::shared_ptr> &landmark_factors);
        void landmarkSelectionSLAMFactorsEliminate(gtsam::GaussianFactorGraph::shared_ptr base_graph, std::vector<gtsam::GaussianFactor::shared_ptr> &landmark_factors);

        std::shared_ptr<MapDataAlias> map_data;
        size_t n_poses;
        size_t n_mappoints;

        gtsam::Ordering landmarks;
        gtsam::Ordering pose_order;

        gtsam::NonlinearFactorGraph graph;
        gtsam::Values values;

        gtsam::Cal3_S2::shared_ptr cal_mono;
        gtsam::Cal3_S2Stereo::shared_ptr cal_stereo;

        std::vector<gtsam::noiseModel::Base::shared_ptr> mono_noise;
        std::vector<gtsam::noiseModel::Base::shared_ptr> stereo_noise;
        std::vector<double> level_scale;
        std::vector<double> level_inlier_th;

        size_t outliers;
        GTSAMGraphSettings gsettings;
    };

}

#endif