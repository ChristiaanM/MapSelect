
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


#include "mapselect/utils/BucketQueue.h"
#include <iostream>

using namespace std;


BucketQueue::BucketQueue(size_t max_value)
{
    buckets.resize(max_value+1);
}

BucketQueue::BucketQueue(const std::vector<std::pair<size_t,size_t>>& vect)
{
    size_t max_value = 0;
    for(size_t i=0;i<vect.size();i++)
        if(max_value < vect[i].second)
            max_value = vect[i].second;
        
    buckets.resize(max_value+1);
    for(const std::pair<size_t,size_t>& pair : vect)
        push(pair.first,pair.second);
}


bool BucketQueue::empty()
{
    return key_num == 0;
}

size_t BucketQueue::size()
{
    return key_num;
}

pair<size_t,size_t> BucketQueue::top() 
{
    size_t key  = buckets[last_bucket].front();
    return make_pair(key,last_bucket);
}
        
pair<size_t,size_t> BucketQueue::pop()
{
    size_t key = buckets[last_bucket].front();
    
    size_t val = last_bucket;

    buckets[last_bucket].pop();

    key_num--;
    while(buckets[last_bucket].empty() && last_bucket > 0)
    {
        last_bucket--;
    }
    return std::make_pair(key,val);
}


void BucketQueue::push(size_t key, size_t value)
{
    buckets[value].push(key);
    key_num++;
    if (value > last_bucket)
        last_bucket = value;
}

void BucketQueue::push(std::pair<size_t,size_t> kv_pair)
{
    push(kv_pair.first,kv_pair.second);
}

void BucketQueue::emplace(size_t key, size_t value)
{
    push(key,value);
}



