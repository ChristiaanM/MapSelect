
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


#pragma once

#include <vector>
#include <algorithm>


/*
For a set represent as a unsorted std::vector of keys that are not necessarily unique, this returns both the inverse of that set of keys
and the set with duplicates removed. 

TODO: Minimise calls to this function

*/
static void inverse_keyvector(const std::vector<size_t> &input, std::vector<size_t> &inv_set, std::vector<size_t> &key_set, size_t max_key)
{
    std::vector<size_t> out;
    std::vector<size_t> key_buffer(input);
    std::sort(key_buffer.begin(), key_buffer.end());

    auto last = std::unique(key_buffer.begin(), key_buffer.end());
    key_buffer.resize(last - key_buffer.begin());

    auto search = std::lower_bound(key_buffer.begin(), key_buffer.end(), max_key);
    key_buffer.resize(search - key_buffer.begin());

    out.reserve(max_key - key_buffer.size());

    auto iter = key_buffer.begin();
    auto iter_end = key_buffer.end();
    for (size_t i = 0; i < max_key; i++)
    {
        while (iter != iter_end && i > *iter)
            ++iter;

        if (iter == iter_end || i < *iter)
            out.push_back(i);
    }

    key_set = key_buffer;
    inv_set = out;
}
