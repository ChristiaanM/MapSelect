/*
 * File: LazyGreedy.h
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


#ifndef MAPSELECT_LAZY_GREEDY_H
#define MAPSELECT_LAZY_GREEDY_H

#include <vector>

#include "mapselect/functions/SetFunction.h"
#include "mapselect/utils/StablePriorityQueue.h"


namespace mselect
{
    /*!
    \file

    @brief An implementation of the lazy greedy algorithm
    @tparam V the return function return type
    @tparam Q the priority queue implementation used by the algorithm 

    @param func function to use for lazy greedy
    @param eval_keys available keys to select from 
    @param N the number of keys to select
    @param not_selected optional pointer to a std::vector - to return the keys
    not selected by the greedy algorithm
    
    */
    template <class V = double, class Q = StablePriorityQueue<double>>
    void LazyGreedy(IncrementalSetFunction<V> &func, const std::vector<size_t> &eval_keys, size_t N = std::numeric_limits<size_t>::max(), std::vector<size_t> *not_selected = nullptr)
    {
        // Create Queue Inplace with Heapify
        std::vector<std::pair<size_t, V>> key_values;
        key_values.reserve(eval_keys.size());
        for (size_t key : eval_keys)
            key_values.emplace_back(key, func.gain(key));
        Q queue(key_values);

        // Optionally, return the keys not selected
        if (not_selected)
        {
            not_selected->reserve(eval_keys.size());
            not_selected->resize(0);
        }

        while (func.size() < N && !queue.empty())
        {
            auto top = queue.top();
            queue.pop();

            V new_gain = func.gain(top.first);

            if (queue.empty() || new_gain >= queue.top().second)
            {
                if (new_gain <= 0)
                {
                    break;
                }

                func.add(top.first);
            }
            else
            {
                if (new_gain > 0)
                    queue.emplace(top.first, new_gain);
                else
                    if (not_selected)
                        not_selected->push_back(top.first);
            }
        }

        if (not_selected)
        {
            //! @todo if we use a different queue implementation we can just
            // iterate over the structure in order, this is slower
            while (!queue.empty())
            {
                not_selected->push_back(queue.top().first);
                queue.pop();
            }
        }
    }
}

#endif