/*
 * File: KeyVectorUtils.h
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



#ifndef PATH_KEY_VECTOR_UTILS
#define PATH_KEY_VECTOR_UTILS

#include <vector>
#include <algorithm>

#include <iostream>

namespace mselect
{

    /*! 
    @brief For a set represent as a unsorted std::vector of keys that are not necessarily unique, this returns both the inverse of that set of keys
    and the set with duplicates removed.

    One would think that it would be better to use a bitset to get the set O(n) vs O(nlogn), but for small sets (such as when using this for the 
    last keyframe heuristic), this implementation was faster. 

    @todo Minimise calls to this function
    */
    static void inverse_keyvector(const std::vector<size_t> &input, std::vector<size_t> &inv_set, std::vector<size_t> &key_set, size_t max_key)
    {
        key_set = std::vector<size_t>(input);
        std::sort(key_set.begin(), key_set.end());
        auto last = std::unique(key_set.begin(), key_set.end());
        key_set.resize(last - key_set.begin());

        inv_set.reserve(max_key - key_set.size());

        auto iter = key_set.begin();
        auto iter_end = key_set.end();
        for (size_t i = 0; i < max_key; i++)
        {
            while (iter != iter_end && i > *iter)
                ++iter;

            if (iter == iter_end || i < *iter)
                inv_set.push_back(i);
        }
    }

}

#endif