
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

#ifndef OPT_GTSAM_H
#define OPT_GTSAM_H


#include "mapselect/maps/MapDataAlias.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>

#include <string>
#include <Eigen/Core>
#include <Eigen/StdVector>

typedef Eigen::Matrix<double,12,12> MatVO;
typedef gtsam::Matrix66 PoseMat;
typedef std::vector<PoseMat,Eigen::aligned_allocator<PoseMat>> PoseMatVector;

struct GTSAMGraphSettings
{
    bool robust = false;
    bool stereo_only = false;
    bool remove_outliers = false; 
    bool smart = false;
};


class SLAMGraphGTSAM
{
    public:      
        SLAMGraphGTSAM(); 
        SLAMGraphGTSAM(std::shared_ptr<MapDataAlias> map_data_ptr, GTSAMGraphSettings gsettings = GTSAMGraphSettings());
        gtsam::Values optimise(std::string out);
        void landmarkSelection(gtsam::GaussianFactorGraph::shared_ptr base_graph, std::vector<gtsam::GaussianFactor::shared_ptr>& landmark_factors);
        
        
        gtsam::NonlinearFactor::shared_ptr obsFactor(size_t frame_key , size_t point_key, const AliasKeypoint &keypoint );

        gtsam::GaussianFactor::shared_ptr obsFactorLinear(size_t frame_key , size_t point_key, const AliasKeypoint &keypoint );

        bool isPointOk(size_t point_key);

        void buildSelectionLocalisation(PoseMat &prior, std::vector<PoseMatVector>& landmark_factors);
        void buildSelectionOdometry(PoseMat &prior, std::vector<PoseMatVector>& landmark_factors);
        void buildSelectionSLAM( std::vector<gtsam::GaussianFactorGraph::shared_ptr>& local_priors,std::vector<std::vector<gtsam::GaussianFactor::shared_ptr>>& local_factors);

        void buildOffline(const std::vector<size_t>& eval_keys, size_t eval_frame, gtsam::NonlinearFactorGraph::shared_ptr out_graph, gtsam::Values::shared_ptr out_values, bool verbose = false, bool prior = false);
        void offlineOptimise(const std::vector<size_t>& eval_keys, size_t eval_frame, std::string outpath);
        void writeTrajectory(gtsam::Values &out_values, const std::string out_path);

        static gtsam::Symbol X(size_t frame_key);
        static gtsam::Symbol L(size_t point_key);
        static gtsam::Symbol N(size_t point_key);


    protected:
        void landmarkSelectionSmart(gtsam::GaussianFactorGraph::shared_ptr base_graph, std::vector<gtsam::GaussianFactor::shared_ptr>& landmark_factors);
        void landmarkSelectionFromEliminate(gtsam::GaussianFactorGraph::shared_ptr base_graph, std::vector<gtsam::GaussianFactor::shared_ptr>& landmark_factors);

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



#endif