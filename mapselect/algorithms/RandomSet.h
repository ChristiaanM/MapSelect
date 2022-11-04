

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


#ifndef RANDOM_SET_H
#define RANDOM_SET_H

#include <iterator>     // std::back_inserter
#include <vector>       // std::vector
#include <algorithm>    // std::copy
#include <random>       // mt19937

/*
Select a random subset of size N keys, from the provided vector. Optionally, first include heuristic keys. 
Does no checking to ensure that keys are unique or valid.


*/
std::vector<size_t> RandomSelection(const std::vector<size_t>& rand_keys, size_t N, const std::vector<size_t> *heuristic_keys = nullptr)
{
    std::vector<size_t> buffer;
    size_t selected;


    // optionally, place heuristically chosen keys at the front of the buffer
    if (heuristic_keys)
    {
        buffer.reserve(rand_keys.size()+heuristic_keys->size());

        // add heuristic keys
        std::copy(heuristic_keys->begin(),heuristic_keys->end(),back_inserter(buffer));
        selected = heuristic_keys->size();
    }
    else
    {
        buffer.reserve(rand_keys.size());
    }    
    selected = buffer.size();  


    std::copy(rand_keys.begin(),rand_keys.end(),std::back_inserter(buffer));

    std::random_device rd;
    std::mt19937 gen(rd());
    for(; selected < N && selected < buffer.size(); selected++)
    {
        std::uniform_int_distribution<int> dist(selected,buffer.size()-1);
        std::swap(buffer[selected],buffer[dist(gen)]);
    }

    // clip off unselected keys
    buffer.resize(selected);

    return buffer;
 

}



#endif
