/*
 * File: RandomSet.h
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


#ifndef MAPSELECT_RANDOM_SET_H
#define MAPSELECT_RANDOM_SET_H

#include <iterator>  // std::back_inserter
#include <vector>    // std::vector
#include <algorithm> // std::copy
#include <random>    // mt19937

namespace mselect
{

    /*!
       \file
       @brief Select a random subset of size N keys, from the provided vector.
       @param rand_keys keys to select a random subset from
       @param N size of the selected subset
       @param heuristic_keys optionally, first include a defined subset of keys, before randomly selecting keys
       @param not_selected optionally, return the set of keys not selected as well.
   */
    std::vector<size_t> RandomSelection(const std::vector<size_t> &rand_keys, size_t N, const std::vector<size_t> *heuristic_keys = nullptr, std::vector<size_t> *not_selected = nullptr)
    {
        std::vector<size_t> buffer;
        size_t n_selected;

        // optionally, place heuristically chosen keys at the front of the buffer
        if (heuristic_keys)
        {
            buffer.reserve(rand_keys.size() + heuristic_keys->size());

            // add heuristic keys
            std::copy(heuristic_keys->begin(), heuristic_keys->end(), back_inserter(buffer));
            n_selected = heuristic_keys->size();
        }
        else
        {
            buffer.reserve(rand_keys.size());
        }
        n_selected = buffer.size();

        std::copy(rand_keys.begin(), rand_keys.end(), std::back_inserter(buffer));

        std::random_device rd;
        std::mt19937 gen(rd());
        for (; n_selected < N && n_selected < buffer.size(); n_selected++)
        {
            std::uniform_int_distribution<int> dist(n_selected, buffer.size() - 1);
            std::swap(buffer[n_selected], buffer[dist(gen)]);
        }

        if (not_selected)
        {
            not_selected->reserve(rand_keys.size() - n_selected);
            not_selected->assign(rand_keys.begin() + n_selected, rand_keys.end());
        }

        // clip off unselected keys
        buffer.resize(n_selected);

        return buffer;
    }
}

#endif
