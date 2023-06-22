/*
 * File: FrameFunction.h
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

#ifndef MAPSELECT_FRAME_FUNCTION_H
#define MAPSELECT_FRAME_FUNCTION_H

#include "mapselect/maps/MapDataAlias.h"
#include "mapselect/functions/SetFunction.h"
#include "mapselect/functions/SaturatedSetFunction.h"
#include "mapselect/slam/Derivatives.h"
#include "mapselect/slam/SLAMGraph.h"

#include "mapselect/utils/Timing.h"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
namespace mselect
{
    /*! Abstract class for implementing functions where the function value is the sum of functions
    defined for each frame.
    */
    class FrameFunction : public SaturatedSetFunction
    {

    public:
        virtual double evalFrame(size_t key) const = 0;
        /*! Update the frame value after selecting a map point
        @param map_key key of the selected map point
        @param frame_key key of the frame being upated
        @param index - keypoint index relevant to this observation
        */
        virtual void updateFrame(size_t map_key, size_t frame_key, size_t obs_index) = 0;
        /*! Calculate the summand of marginal gain resulting from a spesific observation
        @param map_key key of the selected map point
        @param frame_key key of the frame
        @param index - keypoint index relevant to this observation
        */
        virtual double gainFrame(size_t map_key, size_t frame_key, size_t obs_index) = 0;
        /*! @brief A virtual function that is called before calculating the marginal gain,
        or updating the function due to selecting a map point.*/
        virtual bool precompute(size_t map_key) = 0;

        FrameFunction(std::shared_ptr<MapDataAlias> map_data_ptr) : map_data(map_data_ptr)
        {
            b_computed = false;
        }

        double gain(size_t map_key) override
        {
            if (!precompute(map_key))
                return 0;

            const auto &map_obs = map_data->map_obs[map_key];
            double sum = 0;
            for (size_t i = 0; i < map_obs.size(); i++)
            {
                size_t frame_key = map_obs[i];
                double current_value = frame_value[frame_key];
                if (current_value < b_sat)
                {
                    double unsaturated_gain = gainFrame(map_key, map_obs[i], i);
                    sum += std::min<double>(unsaturated_gain, b_sat - frame_value[frame_key]);
                }
            }
            return sum / map_data->n_frames;
        }

        void update(size_t map_key) override
        {
            if (precompute(map_key))
            {
                const auto &map_obs = map_data->map_obs[map_key];
                for (size_t i = 0; i < map_obs.size(); i++)
                {
                    updateFrame(map_key, map_obs[i], i);
                    frame_value[map_obs[i]] = evalFrame(map_obs[i]);
                }
            }
        }

        double eval() const
        {
            double sum = 0;
            for (size_t i = 0; i < map_data->n_frames; i++)
                sum += std::min<double>(evalFrame(i), b_sat);
            return sum / map_data->n_frames;
        }

        void print() const override
        {
            std::cout << "Average Frame Utility " << eval() << std::endl;
        }

        void computeBBounds()
        {
            std::vector<size_t> copy = getSelected();
            clear();

            for (size_t i = 0; i < map_data->n_points; i++)
                update(i);

            for (size_t i = 0; i < map_data->n_frames; i++)
            {
                double val = evalFrame(i);
                if (val > b_max)
                    b_max = val;
            }
            b_computed = true;

            if (copy.size())
            {
                clear();
                for (size_t i = 0; i < copy.size(); i++)
                    update(copy[i]);
            }
        }

        double getBmax() override
        {
            if (!b_computed)
                computeBBounds();
            return b_max;
        }

        double getBmin() override
        {
            return 0;
        }

    protected:
        std::shared_ptr<MapDataAlias> map_data;
        std::vector<double> frame_value;
        double b_max;
        bool b_computed;
    };

}

#endif