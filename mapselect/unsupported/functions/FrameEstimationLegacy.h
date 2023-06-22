/*
 * File: FrameEstimationLegacy.h
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



#ifndef MAPSELECT_FRAME_ESTIMATION_H
#define MAPSELECT_FRAME_ESTIMATION_H

#include "mapselect/functions/SetFunction.h"
#include "mapselect/functions/SaturatedSetFunction.h"
#include "mapselect/maps/MapDataAlias.h"
#include "mapselect/slam/SLAMGraphGTSAM.h"

#include "mapselect/utils/Timing.h"
#include "mapselect/utils/MatFuncs.h"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <mapselect/slam/SLAMGraph.h>

namespace mselect
{

    enum FrameEstimationFunction
    {
        LEGACY_GTSAM_LOCAL = 0,
        LEGACY_GTSAM_ODOMETRY = 1,
        LEGACY_G2O_LOCAL,
        LEGACY_G2O_ODOM,
    };

    /*! @brief These Functions define the map point selection problem as the sum of information matrices
        This class implements both the localisation and odometry utility
        functions. Additionally, it supports calcluating the information
        matrices using either native implementations of the derivatives or using
        the information matrices from the gtsam toolbox.

    */
    class FrameEstimationLegacy : public SaturatedSetFunction
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /*!
        @param map_data_ptr map data used to construct the information matrices
        @param mode what to use to calcuate the information matrices. See FrameEstimationFunction for a more detailed description
        @param sat_agv_ if we set the paramater b should it limit the function value per frame,
            \f[
                \Sigma\min\left(f_j(S),b\right)
            \f]
        , or the average.
            \f[
                \min\left(\frac{1}{T}\Sigma(f_j(S)),b\right)
            \f]
        */
        FrameEstimationLegacy(std::shared_ptr<MapDataAlias> map_data_ptr, FrameEstimationFunction mode, bool sat_avg_ = false) : map_data(map_data_ptr), use_b_sat_avg(sat_avg_)
        {
            PoseInfoMat prior;
            //TIC(mat_calc);
            if (mode == FrameEstimationFunction::LEGACY_GTSAM_LOCAL)
            {
                GTSAMGraphSettings gsettings;
                auto gtsam_map = std::make_shared<SLAMGraphGTSAM>(map_data_ptr, gsettings);
                gtsam_map->buildSelectionLocalisation(prior, landmark_factors);
            }
            else if (mode == FrameEstimationFunction::LEGACY_GTSAM_ODOMETRY)
            {
                GTSAMGraphSettings gsettings;
                auto gtsam_map = std::make_shared<SLAMGraphGTSAM>(map_data_ptr, gsettings);
                gtsam_map->buildSelectionOdometry(prior, landmark_factors);
            }
            else if (mode == FrameEstimationFunction::LEGACY_G2O_LOCAL)
            {
                SLAMGraph().buildSelectionLocalisation(map_data_ptr, prior, landmark_factors);
            }
            else if (mode == FrameEstimationFunction::LEGACY_G2O_ODOM)
            {
                SLAMGraph().buildSelectionOdometry(map_data_ptr, prior, landmark_factors);
            }

            //TOC(mat_calc);
            //TIME_PRINT(mat_calc);

            prior_u = prior;
            prior_logdet = sym_log_det(prior);
            frame_K.resize(map_data->n_frames);
            frame_logdet.resize(map_data->n_frames);
            edge_evals = 0;
            reset();
        }

        double getBmax() override
        {
            clear();

            for (size_t i = 0; i < map_data->n_points; i++)
            {
                update(i);
            }

            double b_max = 0;

            for (size_t i = 0; i < map_data->n_frames; i++)
            {
                double val = sym_log_det(frame_K[i]);
                if (val > b_max)
                    b_max = val;
            }
            clear();
            return b_max;
        }

        double getBmin() override
        {
            return prior_logdet;
        }

        double gain(size_t key) override
        {

            const auto &map_obs = map_data->map_obs[key];
            double sum = 0;

            if (landmark_factors[key].size())
            {
                for (size_t i = 0; i < map_obs.size(); i++)
                {
                    edge_evals++;
                    // Hack - a quick way to identify empty factors
                    if (landmark_factors[key][i](0, 0) == 0.0)
                        continue;

                    size_t frame = map_obs[i];
                    PoseInfoMat tmp = frame_K[frame] + landmark_factors[key][i];

                    double logdet_before = frame_logdet[frame];
                    double logdet_after = sym_log_det(tmp);

                    if (use_b_sat_avg)
                        sum += logdet_after - logdet_before;
                    else
                        sum += std::min<double>(logdet_after, b_sat) - std::min<double>(logdet_before, b_sat);
                }
            }
            sum /= map_data->n_frames;
            if (use_b_sat_avg)
            {
                return std::max<double>(std::min<double>(sum, b_sat - b_sat_avg), 0);
            }
            return sum;
        }

        void update(size_t key) override
        {
            const auto &map_obs = map_data->map_obs[key];
            if (landmark_factors[key].size())
            {
                for (size_t i = 0; i < map_obs.size(); i++)
                {
                    if (landmark_factors[key][i](0, 0) == 0.0)
                        continue;

                    size_t frame = map_obs[i];
                    frame_K[frame] += landmark_factors[key][i];
                    frame_logdet[frame] = sym_log_det(frame_K[frame]);
                }
            }

            if (use_b_sat_avg)
            {
                b_sat_avg = 0;
                for (size_t i = 0; i < map_data->n_frames; i++)
                {
                    b_sat_avg += frame_logdet[i];
                }
                b_sat_avg /= map_data->n_frames;
            }
        }

        double eval() const
        {
            double sum = 0;
            if (use_b_sat_avg)
            {
                for (size_t i = 0; i < map_data->n_frames; i++)
                {
                    sum += sym_log_det(frame_K[i]);
                }
                sum /= map_data->n_frames;
                return std::min<double>(sum, b_sat);
            }
            else
            {

                for (size_t i = 0; i < map_data->n_frames; i++)
                {
                    sum += std::min<double>(sym_log_det(frame_K[i]), b_sat);
                }
                sum /= map_data->n_frames;
                return sum;
            }
        }

        void print() const override
        {
            std::cout << "Average Frame Utility\t" << eval() << std::endl;

            size_t mats = 0;
            for (auto vec : landmark_factors)
                mats += vec.size();

            std::cout << "Edge Evals\t" << edge_evals << std::endl;
            std::cout << "Allocated " << mats << "\tedge matrices" << std::endl;
            std::cout << "Allocated " << frame_K.size() << "\tframe matrices" << std::endl;
        }

    protected:
        void reset() override
        {
            PoseInfoMat prior = prior_u;
            for (size_t i = 0; i < map_data->n_frames; i++)
            {
                frame_K[i] = prior;
            }
            std::fill(frame_logdet.begin(), frame_logdet.end(), prior_logdet);
            b_sat_avg = prior_logdet;
        }

        // Workaround since old compilers refuse to do aligned memory allocation here.
       //! @todo Figure out why some old compilers give issues with aligned members, but are fine for other classes 
        Mat66Unaligned prior_u;
        double prior_logdet;

        PoseMatVector frame_K;
        std::vector<double> frame_logdet;

        std::vector<PoseMatVector> landmark_factors;
        std::shared_ptr<MapDataAlias> map_data;

        bool use_b_sat_avg;
        // used to help calculating the marginal gain when using sat_avg = true
        double b_sat_avg;

        size_t edge_evals;
    };

}
#endif
