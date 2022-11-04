
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



#ifndef LAZY_GREEDY_H
#define LAZY_GREEDY_H

#include "mapselect/functions/SetFunction.h"
#include <vector>

template <class T>
struct KeyValueCompare
{
    bool operator()(const std::pair<size_t, T> &lhs, const std::pair<size_t, T> &rhs) const
    {
        // if (lhs.second == rhs.second)
        //    return lhs.first > rhs.second;
        return lhs.second < rhs.second;
    }
};

template <class V, class Q>
V LazyGreedy(IncrementalSetFunction<V> &func, const std::vector<size_t> &eval_keys, size_t N = std::numeric_limits<size_t>::max())
{
 
    std::vector<std::pair<size_t, V>> key_values;

    key_values.reserve(eval_keys.size());
    for (size_t key : eval_keys)
        key_values.emplace_back(key, func.gain(key));

    Q queue(key_values);

    size_t i = 0;

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

            func.add(top.first, new_gain);

        }
        else
        {
            if (new_gain > 0)
                queue.emplace(top.first, new_gain);
        }
    }

    return func.getSum();
}

#endif