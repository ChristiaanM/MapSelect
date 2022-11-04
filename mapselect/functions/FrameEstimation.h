
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

#ifndef FRAME_ESTIMATION_H
#define FRAME_ESTIMATION_H

#include "mapselect/functions/SetFunction.h"
#include "mapselect/maps/MapDataAlias.h"
#include "mapselect/slam/SLAMGraphGTSAM.h"

enum EntropyMode
{
    LOCAL = 0,
    ODOMETRY = 1,
};

class FrameEstimation : public SaturatedSetFunction, public EvaluableFunction<double>
{
public:
    FrameEstimation(std::shared_ptr<MapDataAlias> map_data_ptr, EntropyMode mode, bool sat_avg = false) : map_data(map_data_ptr), sat_avg(sat_avg)
    {
        GTSAMGraphSettings gsettings;
        auto gtsam_map = std::make_shared<SLAMGraphGTSAM>(map_data_ptr, gsettings);
        if (mode == EntropyMode::LOCAL)
            gtsam_map->buildSelectionLocalisation(prior, landmark_factors);
        else if (mode == EntropyMode::ODOMETRY)
            gtsam_map->buildSelectionOdometry(prior, landmark_factors);

        frame_K.resize(map_data->n_frames);
        frame_logdet.resize(map_data->n_frames);
        reset();

        for (size_t i = 0; i < map_data->n_points; i++)
        {
            update(i);
        }

        b_max = 0;

        for (size_t i = 0; i < map_data->n_frames; i++)
        {
            double val = log(frame_K[i].determinant());
            if (val > b_max)
                b_max = val;
        }
        // std::cout << "B_MAX" << b_max << std::endl;
        reset();
    }

    double getBmax() const override
    {
        return b_max;
    }

    double getBmin() const override
    {
        return log(prior.determinant());
    }

    double gain(size_t key) override
    {
        const auto &map_obs = map_data->map_obs[key];
        double sum = 0;

        if (landmark_factors[key].size())
        {
            for (size_t i = 0; i < map_obs.size(); i++)
            {
                size_t frame = map_obs[i];
                PoseMat tmp = frame_K[frame];

                double before = frame_logdet[frame];

                // Hack - a quick way to identify emtpy factors
                if (landmark_factors[key][i](0, 0) == 0.0)
                    continue;

                tmp += landmark_factors[key][i];
                double after = log(tmp.determinant());
                if (sat_avg)
                    sum += after - before;
                else
                    sum += std::min<double>(after, b_sat) - std::min<double>(before, b_sat);
            }
        }
        sum /= frame_K.size();
        if (sat_avg)
        {
            return std::max<double>(std::min<double>(sum, b_sat - average), 0);
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
                size_t frame = map_obs[i];
                frame_K[frame] += landmark_factors[key][i];
                frame_logdet[frame] = log(frame_K[frame].determinant());
            }
        }

        if (sat_avg)
        {
            average = 0;
            for (size_t i = 0; i < map_data->n_frames; i++)
            {
                average += frame_logdet[i];
            }
            average /= map_data->n_frames;
        }
    }

    double eval() const
    {
        double score = 0;

        for (size_t i = 0; i < map_data->n_frames; i++)
        {
            double logdet = log(frame_K[i].determinant());
            if (sat_avg)
                score += logdet;
            else
                score += std::min<double>(logdet, b_sat);
        }
        score /= map_data->n_frames;
        if (sat_avg && score > b_sat)
            score = b_sat;
        return score;
    }

    void print() const override
    {

        std::cout << "Avg Local Score " << eval() << std::endl;
    }

protected:
    void reset() override
    {
        for (size_t i = 0; i < map_data->n_frames; i++)
        {
            frame_K[i] = gtsam::Matrix66(prior);
        }
        double tmp = log(prior.determinant());
        std::fill(frame_logdet.begin(), frame_logdet.end(), tmp);

        average = eval();
    }

    double b_max = 0;
    bool sat_avg;
    double average;

    PoseMat prior;
    PoseMatVector frame_K;
    std::vector<double> frame_logdet;

    std::vector<PoseMatVector> landmark_factors;

    std::shared_ptr<MapDataAlias> map_data;
};

#endif
