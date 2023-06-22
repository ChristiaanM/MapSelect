/*
 * File: SaturatedFunctions.h
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



#ifndef MAPSELECT_SATURATED_FUNCTIONS_H
#define MAPSELECT_SATURATED_FUNCTIONS_H

#include "mapselect/functions/SaturatedSetFunction.h"

namespace mselect
{

    class CoverSat : public SaturatedSetFunction
    {
    public:
        CoverSat(std::shared_ptr<MapDataAlias> map_data_ptr) : map_data(map_data_ptr)
        {
            frame_obs_cnt.resize(map_data_ptr->n_frames, 0U);
        }

        double getBmax() override
        {
            double max_obs = 0;
            for (const auto &obs : map_data->frame_obs)
            {
                if (obs.size() > max_obs)
                    max_obs = obs.size();
            }
            return max_obs;
        }

        double getBmin() override
        {
            return 0;
        }

        double gain(size_t key) override
        {
            const auto &map_obs = map_data->getMappointObs(key);
            double sum = 0;
            for (size_t frame_key : map_obs)
                if (frame_obs_cnt[frame_key] < b_sat)
                {
                    sum += std::min<double>(1.0, b_sat - frame_obs_cnt[frame_key]);
                }
            // std::cout << "min" <<  sum << std::endl;
            // std::cout << b_sat << std::endl;
            return sum;
        }

        void update(size_t key) override
        {
            const auto &map_obs = map_data->getMappointObs(key);
            for (size_t frame_key : map_obs)
            {
                frame_obs_cnt[frame_key]++;
            }
        }

        void print() const override
        {
            size_t min_key = 0;
            size_t min_obs = frame_obs_cnt[0];
            for (size_t i = 1; i < frame_obs_cnt.size(); i++)
            {
                if (frame_obs_cnt[i] < min_obs)
                {
                    min_obs = frame_obs_cnt[i];
                    min_key = i;
                }
            }
            std::cout << "min obs " << min_obs << " @ " << min_key << std::endl;
            std::cout << "cover-sat" << b_sat << std::endl;
        }

    protected:
        void reset() override
        {
            std::fill(frame_obs_cnt.begin(), frame_obs_cnt.end(), 0);
        }

        std::shared_ptr<MapDataAlias> map_data;
        std::vector<size_t> frame_obs_cnt;
    };
    
}

#endif