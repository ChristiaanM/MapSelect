/*
 * File: StochasticGreedy.h
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

#ifndef MAPSELECT_STOCHASTIC_GREEDY_H
#define MAPSELECT_STOCHASTIC_GREEDY_H

#include <random>
#include <limits>
#include "mapselect/functions/SetFunction.h"
#include "mapselect/utils/Timing.h"

namespace mselect
{

    /*!
       \file
       @brief Select a subset of map points using the Stochastic greedy algorithm
       @param eval_keys set of keys to select from
       @param N size of the selected subset
       @param scaling paramater to control the performance gaurentee of the algorithm
       @param not_selected optionally, return the set of keys not selected as well.
   */
    template <class V>
    void StochasticGreedy(IncrementalSetFunction<V> &func, const std::vector<size_t> &eval_keys, size_t N, double eps, std::vector<size_t> *not_selected = nullptr)
    {
        size_t N_v = eval_keys.size();
        size_t S = (size_t)ceil(log(1.0 / eps) * N_v / N);

        std::random_device rd;
        std::mt19937 gen(rd());

        std::vector<std::pair<size_t, V>> pair_vect;
        pair_vect.reserve(eval_keys.size());

        for (size_t key : eval_keys)
        {
            pair_vect.emplace_back(key, std::numeric_limits<V>::max());
        }

        TIC(stoch);

        while (func.size() < N && !pair_vect.size())
        {

            // First, place S items random in front of the vector
            for (size_t i = 0; i < S; i++)
            {
                std::uniform_int_distribution<int> dist(i, pair_vect.size() - 1);
                std::swap(pair_vect[i], pair_vect[dist(gen)]);
            }

            size_t best_idx = 0U;
            auto best_pair = pair_vect[best_idx];
            best_pair.second = func.gain(best_pair.first);

            // find the best element in the subset
            for (size_t i = 1U; i < S; i++)
            {
                auto &pair = pair_vect[i];
                if (pair.second > best_pair.second)
                {
                    pair.second = func.gain(pair.first);
                    if (pair.second > best_pair.second)
                    {
                        best_pair = pair;
                        best_idx = i;
                    }
                }
            }

            if (best_pair != pair_vect.back())
            {
                std::swap(pair_vect[best_idx], pair_vect.back());
            }

            pair_vect.resize(pair_vect.size() - 1);

            func.add(best_pair.first);

            double avg_per_iter = TIME_RUNNING(stoch) / 1e9 / N;
            if (func.size() % 100 == 0)
                std::cout << "k = " << func.size() << " of " << N << " " << avg_per_iter << std::endl;
        }

        if (not_selected)
        {
            not_selected->resize(pair_vect.size());
            for (size_t i = 0; i < pair_vect.size(); i++)
            {
                not_selected->at(i) = pair_vect[i].first;
            }
        }
    }

}

#endif