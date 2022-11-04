
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


#ifndef STABLE_PRIORITY_QUEUE
#define STABLE_PRIORITY_QUEUE


#include <queue>


/*
A stable priority queue ensures that if two elements have equal priority,
they will be popped in insertion order. This gaurentees that the selected
elements are identical regardless of queue implementation. 
*/

template<class T>
class StablePriorityQueue
{  
   
   
    struct PQEntry
    {
        size_t key;
        size_t cnt;
        T value;

        PQEntry(size_t key, T value, size_t cnt ): key(key), value(value), cnt(cnt)
        {
        }

        PQEntry(const std::pair<size_t, T>& pair, size_t cnt) : key(pair.first), value(pair.second), cnt(cnt)
        {
        }
    };


    struct PQComparator
    {
        bool operator()(const PQEntry &lhs, const PQEntry &rhs) const
        {
            if (lhs.value != rhs.value)
                return lhs.value < rhs.value;
            return lhs.cnt > rhs.cnt;
        }
    };

    typedef std::priority_queue<PQEntry,std::vector<PQEntry>, PQComparator> PQType;



    public:
        StablePriorityQueue(const std::vector<std::pair<size_t,T>>& vect)
        {
            std::vector<PQEntry> tmp;
            tmp.reserve(vect.size());
            entry_cnt = 0;

            for(const std::pair<size_t,T>& p: vect)
                tmp.emplace_back(p,entry_cnt++);

            queue = PQType(PQComparator(),tmp);      
        }


        void push(std::pair<size_t,T> pair)
        {
            queue.emplace(pair.first,pair.second,entry_cnt++);
        }

        void emplace(size_t key, T value)
        {
            queue.emplace(key,value,entry_cnt++);
        }

        void pop()
        {
            queue.pop();
        }
        
        std::pair<size_t,T> top()
        {
            auto tmp = queue.top();
            return std::make_pair(tmp.key,tmp.value);
        }

        bool empty()
        {
            return queue.empty();
        }


    protected:
       PQType queue;
       size_t entry_cnt = 0;


};

#endif