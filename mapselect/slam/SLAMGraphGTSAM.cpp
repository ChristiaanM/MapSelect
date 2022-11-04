
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

#include "mapselect/slam/SLAMGraphGTSAM.h"

#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactor.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>

#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include <gtsam/geometry/StereoPoint2.h>

using namespace gtsam;
using namespace std;
#include <unordered_set>
#include <iostream>
#include <fstream>

const double POSE_PRIOR = 1e-4;
const double ORIGIN_PRIOR = 1e6;

gtsam::Symbol SLAMGraphGTSAM::X(size_t frame_key)
{
    return Symbol('x', frame_key);
}
gtsam::Symbol SLAMGraphGTSAM::L(size_t point_key)
{
    return Symbol('l', point_key);
}

gtsam::Symbol SLAMGraphGTSAM::N(size_t point_key)
{
    return Symbol('n', point_key);
}

bool SLAMGraphGTSAM::isPointOk(size_t point_key)
{
    return values.exists(L(point_key));
}

gtsam::NonlinearFactor::shared_ptr SLAMGraphGTSAM::obsFactor(size_t frame_key, size_t point_key, const AliasKeypoint &keypoint)
{
    double px = keypoint.px;
    double py = keypoint.py;
    double disp = keypoint.disp;
    size_t oct = keypoint.oct;
    double th = level_inlier_th[oct];

    NonlinearFactor::shared_ptr factor;
    gtsam::Pose3 pose = values.at<Pose3>(X(frame_key));
    gtsam::Point3 point(map_data->mappoints[point_key].x, map_data->mappoints[point_key].y, map_data->mappoints[point_key].z);

    if (disp > 0)
    {
        auto tmp = GenericStereoFactor<Pose3, Point3>::shared_ptr(new GenericStereoFactor<Pose3, Point3>(StereoPoint2(px, px - disp, py), stereo_noise[oct], X(frame_key), L(point_key), cal_stereo));
        if (gsettings.remove_outliers)
        {
            auto err_vect = tmp->evaluateError(pose, point);
            double e0 = err_vect(0);
            double e1 = err_vect(2);
            if (e0 * e0 + e1 * e1 > th)
            {
                outliers++;
                return nullptr;
            }
                
        }
        return tmp;
    }
    else if (!gsettings.stereo_only)
    {
        auto tmp = GenericProjectionFactor<Pose3, Point3, Cal3_S2>::shared_ptr(new GenericProjectionFactor<Pose3, Point3, Cal3_S2>(Point2(px, py), mono_noise[oct], X(frame_key), L(point_key), cal_mono));
        if (gsettings.remove_outliers)
        {
            auto err_vect = tmp->evaluateError(pose, point);
            double e0 = err_vect(0);
            double e1 = err_vect(1);
            if (e0 * e0 + e1 * e1 > th)
            {
                outliers++;
                return nullptr;

            }
        }
       return tmp;
    }
    return nullptr;
}

gtsam::GaussianFactor::shared_ptr SLAMGraphGTSAM::obsFactorLinear(size_t frame_key, size_t map_key, const AliasKeypoint &keypoint)
{
    gtsam::NonlinearFactor::shared_ptr nonlin = obsFactor(frame_key, map_key, keypoint);
    if (nonlin)
        return nonlin->linearize(values);
    return nullptr;
}

SLAMGraphGTSAM::SLAMGraphGTSAM(std::shared_ptr<MapDataAlias> map_data_ptr, GTSAMGraphSettings gsettings_) : map_data(map_data_ptr), gsettings(gsettings_)
{
    const AliasFrame &frame0 = map_data->frames[0];
    double fx = frame0.fx, fy = frame0.fy, cx = frame0.cx, cy = frame0.cy, b = frame0.bf / fx;

    n_poses = map_data->n_frames;
    n_mappoints = map_data->n_points;

    cal_mono = gtsam::Cal3_S2::shared_ptr(new Cal3_S2(fx, fy, 0, cx, cy));
    cal_stereo = gtsam::Cal3_S2Stereo::shared_ptr(new Cal3_S2Stereo(fx, fy, 0, cx, cy, b));

    {
        size_t scales = 8;
        double scale = 1.0;
        double scale_factor = 1.2;
        double sig = 1.0;
        double th = 5.991;

        for (size_t i = 0; i < scales; i++)
        {
            if (gsettings.robust)
            {

                auto mono_noise_tmp = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(sqrt(5.99)),
                                                                 noiseModel::Isotropic::Sigma(2, sig));
                auto stereo_noise_tmp = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(sqrt(7.815)),
                                                                   noiseModel::Isotropic::Sigma(3, sig));
                mono_noise.push_back(mono_noise_tmp);
                stereo_noise.push_back(stereo_noise_tmp);
            }
            else
            {
                mono_noise.push_back(noiseModel::Isotropic::Sigma(2, sig));
                stereo_noise.push_back(noiseModel::Isotropic::Sigma(3, sig));
            }
            level_scale.push_back(scale);
            level_inlier_th.push_back(th * th * sig * sig);
            sig *= scale_factor;
        }
    }

    outliers = 0;
    
    //std::vector<bool> point_ok;
    //point_ok.resize(map_data->n_points,false);

    for (size_t f = 0; f < map_data->n_frames; f++)
    {
        Symbol f_sym = X(f);
        const auto &frame = map_data->frames[f];
        const auto &Tcw = frame.Tcw;

        Rot3 R(
            Tcw(0, 0), Tcw(0, 1), Tcw(0, 2),
            Tcw(1, 0), Tcw(1, 1), Tcw(1, 2),
            Tcw(2, 0), Tcw(2, 1), Tcw(2, 2));

        Point3 t(frame.Tcw(0, 3), frame.Tcw(1, 3), frame.Tcw(2, 3));
        Pose3 pose = Pose3(R, t).inverse();
        values.insert(f_sym, pose);
    }

    for (size_t p = 0; p < map_data->n_points; p++)
    {

        const auto &mappoint_obs = map_data->map_obs[p];
        const auto &mappoint_keypoints = map_data->map_keypoints[p];

        // ignoring degenerate configurations, a landmark must be observed by observed at least twice
        // from stereo observations or 3 times from monocoluar observations
        size_t cnt = 0;
        for (auto keypoint : mappoint_keypoints)
        {
            if (keypoint.disp > 0.0)
                cnt+=2;
            else
                cnt++; 
        }
        if (cnt < 3)
            continue;

        const auto &mappoint = map_data->mappoints[p];

        Symbol p_sym = L(p);
        NonlinearFactorGraph::shared_ptr point_graph(new NonlinearFactorGraph);

        for (size_t i = 0; i < mappoint_obs.size(); i++)
        {
            size_t f = mappoint_obs[i];
            Symbol f_sym = X(f);
            const auto &keypoint = mappoint_keypoints[i];
            auto factor = obsFactor(f, p, keypoint);
            if (factor)
            {
                point_graph->push_back(factor);
            }
            else
            {
                // update counter if any factors were eliminated
                // due to settings such as stereo_only - or remove_outliers
                cnt--;
                if (keypoint.disp > 0.0)
                    cnt--;
            }
        }
        // validate that the point still has the required number of observations
        if (cnt > 2)
        {
            graph.push_back(point_graph->begin(), point_graph->end());
            Point3 point3(mappoint.x, mappoint.y, mappoint.z);
            values.insert(p_sym, point3);
            landmarks.push_back(p_sym);
        }
      
        //point_ok[p]=true;
    }
    if (outliers)
        std::cout << "Outliers Removed" << outliers << std::endl;
}

void SLAMGraphGTSAM::landmarkSelectionSmart(GaussianFactorGraph::shared_ptr base_graph, std::vector<gtsam::GaussianFactor::shared_ptr> &landmark_factors)
{
   gtsam::SmartStereoProjectionParams params(gtsam::LinearizationMode::JACOBIAN_SVD, gtsam::DegeneracyMode::ZERO_ON_DEGENERACY);

    for (size_t p = 0; p < map_data->n_points; p++)
    {

        const auto &mappoint_obs = map_data->map_obs[p];
        const auto &mappoint_keypoints = map_data->map_keypoints[p];

        size_t cnt = mappoint_obs.size();
        const auto &mappoint = map_data->mappoints[p];
        Symbol p_sym = L(p);

        auto factor = SmartStereoProjectionPoseFactor::shared_ptr(new SmartStereoProjectionPoseFactor(stereo_noise[0], params));
        for (size_t i = 0; i < mappoint_obs.size(); i++)
        {
            bool unique = true;
            for (size_t j = 0; j < i; j++)
                if (mappoint_obs[i] == mappoint_obs[j])
                {
                    unique = false;
                    break;
                }
            if (unique)
            {
                size_t f = mappoint_obs[i];
                Symbol f_sym = X(f);
                const auto &keypoint = mappoint_keypoints[i];
                double uR = (keypoint.disp > 0) ? keypoint.px - keypoint.disp : std::numeric_limits<double>::quiet_NaN();
                if ((!gsettings.stereo_only) || keypoint.disp > 0)
                {
                    factor->add(gtsam::StereoPoint2(keypoint.px, uR, keypoint.py), f_sym, cal_stereo);
                }
            }
        }

        if (factor->size())
        {
            auto tmp = factor->linearize(values);
            landmark_factors.push_back(tmp);
        }
        else
        {
            //auto tmp = gtsam::GaussianFactor::shared_ptr(new gtsam::JacobianFactor());
            //tmp->print();
            landmark_factors.push_back(nullptr);
        }

    }

    NonlinearFactorGraph::shared_ptr priors(new NonlinearFactorGraph());
    for (size_t i = 0; i < n_poses; i++)
    {
        Symbol f_sym = X(i);
        double info = POSE_PRIOR;
        if (i == 0)
            info = ORIGIN_PRIOR;
        priors->push_back(PriorFactor<Pose3>::shared_ptr(new PriorFactor<Pose3>(f_sym, values.at<Pose3>(f_sym), noiseModel::Isotropic::Precision(6, info))));
    }
    auto lin_priors = priors->linearize(values);
    base_graph->push_back(lin_priors->begin(), lin_priors->end());
}

void SLAMGraphGTSAM::landmarkSelectionFromEliminate(GaussianFactorGraph::shared_ptr base_graph, std::vector<gtsam::GaussianFactor::shared_ptr> &landmark_factors)
{
    auto elim = graph.linearize(values)->eliminatePartialSequential(landmarks, gtsam::EliminateQR);

    auto lin_graph = elim.second;
    landmark_factors.resize(n_mappoints);

    size_t cnt_null = 0;
    for (size_t i = 0; i < landmarks.size(); i++)
    {
        Symbol lsym(landmarks[i]);
        landmark_factors[lsym.index()] = lin_graph->at(i);
    }

    NonlinearFactorGraph::shared_ptr priors(new NonlinearFactorGraph());
    for (size_t i = 0; i < n_poses; i++)
    {
        Symbol f_sym = X(i);
        double info = POSE_PRIOR;
        if (i == 0)
            info = ORIGIN_PRIOR;
        priors->push_back(PriorFactor<Pose3>::shared_ptr(new PriorFactor<Pose3>(f_sym, values.at<Pose3>(f_sym), noiseModel::Isotropic::Precision(6, info))));
    }
    auto lin_priors = priors->linearize(values);
    base_graph->push_back(lin_priors->begin(), lin_priors->end());
}


void SLAMGraphGTSAM::landmarkSelection(GaussianFactorGraph::shared_ptr base_graph, std::vector<gtsam::GaussianFactor::shared_ptr> &landmark_factors)
{
    if (gsettings.smart) 
        return landmarkSelectionSmart(base_graph,landmark_factors);
    else
        return landmarkSelectionFromEliminate(base_graph,landmark_factors);
}

void SLAMGraphGTSAM::buildOffline(const std::vector<size_t> &eval_keys, size_t eval_frame, NonlinearFactorGraph::shared_ptr out_graph, Values::shared_ptr out_values, bool verbose, bool prior)
{
    std::vector<bool> ind_select(map_data->n_points, false);
    std::vector<bool> ind_graph(map_data->n_points, false);
    std::vector<bool> ind_duplicate(map_data->n_points, false);

    std::vector<bool> dup_ok(map_data->n_points, false);
    std::vector<bool> orig_ok(map_data->n_points, false);

    for (size_t p = 0; p < map_data->n_points; p++)
    {
        const auto &mappoint_obs = map_data->map_obs[p];
        const auto &mappoint_keypoints = map_data->map_keypoints[p];

        size_t cnt_orig = 0;
        size_t cnt_dup = 0;
        for (size_t i = 0; i < mappoint_obs.size(); i++)
        {
            auto keypoint = mappoint_keypoints[i];

            if (mappoint_obs[i] > eval_frame)
            {
                cnt_dup++;
                if (keypoint.disp > 0.0)
                    cnt_dup++;
            }
            else
            {
                cnt_orig++;
                if (keypoint.disp > 0.0)
                    cnt_orig;
            }
        }
        if (cnt_dup > 2)
            dup_ok[p] = true;
        if (cnt_orig > 2)
            orig_ok[p] = true;
    }

    for (size_t key : eval_keys)
    {
        ind_select[key] = true;
    }

    for (gtsam::NonlinearFactor::shared_ptr factor : graph)
    {
        if (factor->keys().size() == 2)
        {
            gtsam::Symbol sym_x(factor->keys()[0]);
            gtsam::Symbol sym_l(factor->keys()[1]);
            gtsam::Symbol sym_n = N(sym_l.index());

            size_t idx = sym_l.index();

            if (verbose)
                std::cout << sym_x << " " << sym_l << std::endl;

            if (ind_select[idx] && orig_ok[idx])
            {
                if (verbose)
                    std::cout << "select!" << std::endl;

                out_graph->push_back(factor);
                ind_graph[idx] = true;
            }
            else if (sym_x.index() > eval_frame && dup_ok[idx])
            {

                gtsam::KeyVector kv;
                kv.push_back(sym_x);
                kv.push_back(sym_n);

                if (verbose)
                    std::cout << "future! & not selected" << std::endl;

                size_t cnt = 0;

                ind_duplicate[idx] = true;
                out_graph->push_back(factor->rekey(kv));
            }
            else
            {
                if (verbose)
                    std::cout << "deleted" << std::endl;
                // "Delete"
            }
        }
        else
            out_graph->push_back(factor);
    }

    if (verbose)
        std::cout << "eval graph size" << out_graph->size() << std::endl;

    for (size_t i = 0; i < map_data->n_frames; i++)
        out_values->insert(X(i), values.at<gtsam::Pose3>(X(i)));

    for (size_t i = 0; i < map_data->n_points; i++)
    {

        if (ind_graph[i])
        {
            auto value = values.at<gtsam::Point3>(L(i));
            out_values->insert(L(i), value);
        }

        if (ind_duplicate[i])
        {
            auto value = values.at<gtsam::Point3>(L(i));
            out_values->insert(N(i), value);
            if (verbose)
                std::cout << N(i) << std::endl;
        }
    }

    if (prior)
    {
        for (size_t i = 0; i < n_poses; i++)
        {
            Symbol f_sym = X(i);
            double info = POSE_PRIOR;
            if (i == 0)
                info = ORIGIN_PRIOR;
            //std::cout << i << std::endl;
            out_graph->push_back(PriorFactor<Pose3>::shared_ptr(new PriorFactor<Pose3>(f_sym, values.at<Pose3>(f_sym), noiseModel::Isotropic::Precision(6, info))));
        }
    }
}

void SLAMGraphGTSAM::offlineOptimise(const std::vector<size_t> &eval_keys, size_t eval_frame, std::string outpath)
{
    NonlinearFactorGraph::shared_ptr eval_graph(new NonlinearFactorGraph());
    Values::shared_ptr eval_values(new Values);

    buildOffline(eval_keys, eval_frame, eval_graph, eval_values, false);

    LevenbergMarquardtParams params = LevenbergMarquardtParams::CeresDefaults();
    params.setVerbosity("ERROR");
    params.absoluteErrorTol = 1e-8;
    params.relativeErrorTol = 0;
    LevenbergMarquardtOptimizer lm(*eval_graph, *eval_values, params);

    gtsam::Values result = lm.optimize();

    writeTrajectory(result, outpath);
}

void SLAMGraphGTSAM::writeTrajectory(gtsam::Values &out_values, const std::string out_path)
{
    ofstream file;
    file.open(out_path + ".traject", ios::out);

    for (size_t i = 0; i < map_data->n_frames; i++)
    {
        gtsam::Pose3 pose = out_values.at<Pose3>(X(i));
        gtsam::Vector q = pose.rotation().quaternion();
        double qw = q(0), qx = q(1), qy = q(2), qz = q(3);
        file << map_data->frames[i].time_stamp << " " << pose.x() << " " << pose.y() << " " << pose.z() << " "
             << qx << " " << qy << " " << qz << " " << qw << endl;
    }
    file.close();
}

void SLAMGraphGTSAM::buildSelectionLocalisation(PoseMat &prior, std::vector<PoseMatVector> &landmark_factors)
{
    PoseMat zero = PoseMat::Zero();

    landmark_factors.resize(map_data->n_points);
    for (size_t point_key = 0; point_key < map_data->n_points; point_key++)
    {

        const auto &obs = map_data->map_obs[point_key];
        const auto &keypoints = map_data->map_keypoints[point_key];

        if (isPointOk(point_key))
        {
            landmark_factors[point_key].reserve(obs.size());
            for (size_t i = 0; i < obs.size(); i++)
            {
                size_t frame_key = obs[i];
                const AliasKeypoint &keypoint = keypoints[i];
                if (gsettings.stereo_only && keypoint.disp < 0)
                {
                    landmark_factors[point_key].push_back(zero);
                    continue;
                }
                GaussianFactor::shared_ptr obs_factor = obsFactorLinear(frame_key, point_key, keypoint);
                if (!obs_factor)
                {
                    landmark_factors[point_key].push_back(zero);
                    continue;
                }

                gtsam::Matrix jac = obs_factor->jacobian().first;
                size_t r = jac.rows();
                size_t c = jac.cols();

                PoseMat K = jac.leftCols<6>().transpose() * jac.leftCols<6>();

                landmark_factors[point_key].push_back(K);
            }
        }
    }
    prior = PoseMat::Identity() * 1e-6;
}



void SLAMGraphGTSAM::buildSelectionOdometry(PoseMat &prior, std::vector<PoseMatVector> &landmark_factors)
{
    PoseMat zero = PoseMat::Zero();

    std::vector<std::map<size_t, size_t>> covis = map_data->computeCovisCounts();

    std::vector<size_t> parents(map_data->n_frames, 0);

    for (size_t f = 0; f < map_data->n_frames; f++)
    {
        size_t parent = 0;
        size_t parent_weight = 0;

        std::map<size_t, size_t> &covis_map = covis[f];
        for (auto pair : covis_map)
        {
            if (pair.second > parent_weight && pair.first < f)
            {
                parent = pair.first;
                parent_weight = pair.second;
            }
        }
        parents[f] = parent;
    }

    landmark_factors.resize(map_data->n_points);
    for (size_t point_key = 0; point_key < map_data->n_points; point_key++)
    {
        const auto &obs = map_data->map_obs[point_key];
        const auto &keypoints = map_data->map_keypoints[point_key];

        if (isPointOk(point_key))
        {
            landmark_factors[point_key].reserve(obs.size());
            for (size_t i = 0; i < obs.size(); i++)
            {
                size_t parent = parents[obs[i]];
                size_t j = 0;
                for (; j < i; j++)
                {
                    if (obs[j] == parent)
                    {
                        break;
                    }
                }
                if (j != i && keypoints[i].disp > 0 && keypoints[j].disp > 0)
                {

                    GaussianFactor::shared_ptr factor_x0 = obsFactorLinear(obs[j], point_key, keypoints[j]);
                    GaussianFactor::shared_ptr factor_x1 = obsFactorLinear(obs[i], point_key, keypoints[i]);
                    if (!factor_x0 || !factor_x1)
                    {
                        landmark_factors[point_key].push_back(zero);
                        continue;
                    }

                    GaussianFactorGraph::shared_ptr lgraph(new GaussianFactorGraph);
                    lgraph->push_back(factor_x0);
                    lgraph->push_back(factor_x1);

                    gtsam::KeyVector landmark;
                    landmark.push_back(L(point_key));
                    gtsam::Ordering poses;
                    poses.push_back(X(obs[j]));
                    poses.push_back(X(obs[i]));

                    try
                    {
                        GaussianFactorGraph::shared_ptr elim = lgraph->eliminatePartialSequential(landmark).second;
                        gtsam::Matrix hess = elim->hessian(poses).first;
                        PoseMat K = hess.block<6, 6>(6, 6);
                        landmark_factors[point_key].push_back(K);
                    }
                    catch (gtsam::IndeterminantLinearSystemException e)
                    {
                        landmark_factors[point_key].push_back(zero);
                    }
                }
                else
                {
                    landmark_factors[point_key].push_back(zero);
                }
            }
        }
    }
    prior = PoseMat::Identity() * 1e-6;
}

Values SLAMGraphGTSAM::optimise(std::string out = "")
{
    LevenbergMarquardtParams params = LevenbergMarquardtParams::CeresDefaults();
    params.setVerbosity("ERROR");
    params.absoluteErrorTol = 1e-8;
    params.relativeErrorTol = 0;
    LevenbergMarquardtOptimizer lm(graph, values, params);
    Values result = lm.optimize();
    if (out.size())
    {
        writeTrajectory(result, out);
    }
    return result;
}

