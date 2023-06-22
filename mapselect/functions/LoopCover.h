/*
 * File: LoopCover.h
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



#ifndef MAPSELECT_LOOP_COVER_H
#define MAPSELECT_LOOP_COVER_H

#include "mapselect/maps/MapDataAlias.h"
#include "mapselect/functions/SaturatedSetFunction.h"
#include <map>

namespace mselect
{

    /*! @brief This class implements the set cover function for a given selection of loop frames
     */
    class LoopCover : public SaturatedSetFunction
    {
    public:
        LoopCover(std::shared_ptr<MapDataAlias> map_data_ptr, const std::vector<size_t> &loop_frames) : map_data(map_data_ptr)
        {
            for (size_t key : loop_frames)
            {
                oracle_cnts[key] = 0;
            }
        }

        double getBmax() override
        {
            double max_obs = 0;
            for (auto pair : oracle_cnts)
            {
                size_t obs = map_data->frame_obs[pair.first].size();
                if (obs > max_obs)
                    max_obs = obs;
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
            {
                auto iter = oracle_cnts.find(frame_key);
                if (iter != oracle_cnts.end())
                {
                    if (iter->second < b_sat)
                        sum += std::min<double>(1.0, b_sat - iter->second);
                }
            }
            return sum;
        }

        void update(size_t key) override
        {
            const auto &map_obs = map_data->getMappointObs(key);
            for (size_t frame_key : map_obs)
            {
                auto iter = oracle_cnts.find(frame_key);
                if (iter != oracle_cnts.end())
                {
                    iter->second++;
                }
            }
        }

        void print() const override
        {
            size_t min_key = std::numeric_limits<size_t>::max();
            size_t min_obs = std::numeric_limits<size_t>::max();
            for (auto pair : oracle_cnts)
            {
                if (pair.second < min_obs)
                {
                    min_obs = pair.second;
                    min_key = pair.first;
                }
            }

            std::cout << "min obs " << min_obs << " @ " << min_key << std::endl;
            std::cout << "bsat" << b_sat << std::endl;
        }

        double eval() const override
        {
            double sum = 0;
            for (auto pair : oracle_cnts)
            {
                sum += std::min<double>(b_sat, pair.second);
            }
            return sum;
        }

    protected:
        void reset() override
        {
            for (auto &pair : oracle_cnts)
            {
                pair.second = 0;
            }
        }
        // we use a map here instead of a unsorted map due to its improved performance on
        // a small number of frames - unsorted_map has beter asymptotic lookup complexity
        std::map<size_t, size_t> oracle_cnts;
        std::shared_ptr<MapDataAlias> map_data;
    };

}
#endif