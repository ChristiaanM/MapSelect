
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

#ifndef BUCKET_QUEUE_H
#define BUCKET_QUEUE_H

#include <utility>
#include <vector>
#include <queue>
#include <stack>

/*
 A bucket queue allows very efficient queue operations but is limited to integer values
*/

class BucketQueue
{
    public:
        BucketQueue(size_t max_val);

        BucketQueue(const std::vector<std::pair<size_t,size_t>>& vect);

        bool empty();
        size_t size();

        std::pair<size_t,size_t> top();
        
        std::pair<size_t,size_t> pop();

        void push(size_t key, size_t value);
        
        void push(std::pair<size_t,size_t> kv_pair);
        void emplace(size_t key, size_t value);


    private:
        std::vector<std::queue<size_t>> buckets;
        size_t key_num = 0;
        size_t last_bucket = 0;
};




#endif