/*
 * File: LoopProb.h
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



#ifndef MAPSELECT_LOOP_PROB_H
#define MAPSELECT_LOOP_PROB_H

#include "mapselect/maps/MapDataAlias.h"
#include "mapselect/functions/SaturatedSetFunction.h"
#include <map>
namespace mselect
{
    /*! @brief Abstract Probalistic variation of the LoopCover - derived classes implement a function
    to obtain the probability of a given map point observation
     */
    class LoopProb : public SaturatedSetFunction
    {
    public:
        virtual double custom_prob(size_t key) = 0;

        void init(std::shared_ptr<MapDataAlias> map_data_ptr, const std::vector<size_t> &oracle_keys)
        {
            map_data = map_data_ptr;

            for (size_t key : oracle_keys)
            {
                oracle_vis_scores[key] = 0;
            }
            vis.resize(map_data->n_points);
            for (size_t i = 0; i < map_data->n_points; i++)
            {
                vis[i] = custom_prob(i);
            }
        }

        double getBmax() override
        {
            double max_obs = 0;
            for (auto pair : oracle_vis_scores)
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
            double pvis = vis[key];

            for (size_t frame_key : map_obs)
            {
                auto iter = oracle_vis_scores.find(frame_key);
                if (iter != oracle_vis_scores.end())
                {
                    if (iter->second < b_sat)
                        sum += std::min<double>(pvis, b_sat - iter->second);
                }
            }
            return sum;
        }

        void update(size_t key) override
        {

            const auto &map_obs = map_data->getMappointObs(key);
            double pvis = vis[key];

            for (size_t frame_key : map_obs)
            {
                auto iter = oracle_vis_scores.find(frame_key);
                if (iter != oracle_vis_scores.end())
                {
                    iter->second += pvis;
                }
            }
        }

        void print() const override
        {
            size_t min_key = 0;
            double min_mu = 0;
            for (auto pair : oracle_vis_scores)
            {
                if (pair.second < min_mu)
                {
                    min_mu = pair.second;
                    min_key = pair.first;
                }
            }

            std::cout << "min obs " << min_mu << " @ " << min_key << std::endl;
            std::cout << "bsat" << b_sat << std::endl;
        }

        double eval() const override
        {
            double sum = 0;
            for (auto pair : oracle_vis_scores)
            {
                sum += std::min<double>(b_sat, pair.second);
            }
            return sum;
        }

    protected:
        void reset() override
        {
            for (auto &pair : oracle_vis_scores)
            {
                pair.second = 0;
            }
        }

        std::map<size_t, double> oracle_vis_scores;
        std::shared_ptr<MapDataAlias> map_data;
        std::vector<double> vis;
    };

    class LoopProbCoverPvis : public LoopProb
    {
    public:
        LoopProbCoverPvis(std::shared_ptr<MapDataAlias> map_data_ptr, const std::vector<size_t> &oracle_keys)
        {
            init(map_data_ptr, oracle_keys);
        }

        double custom_prob(size_t key) override
        {
            return map_data->getMappointPvis(key, 0, 0);
        }
    };
}
#endif