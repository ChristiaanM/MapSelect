
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


#ifndef SET_FUNCTION_H
#define SET_FUNCTION_H

#include <iostream>
#include <memory>
#include <deque>
#include <vector>

template <class T>
class IncrementalSetFunction
{
public:
    virtual T gain(size_t key) = 0;

protected:
    virtual void update(size_t key) = 0;
    virtual void reset() = 0;

public:   
    typedef std::vector<size_t> StorageType;
    typedef StorageType::const_iterator StorageIterator;

    void add(size_t key, T val)
    {
        sum_value += val;
        update(key);
        selected.push_back(key);
    }

    template<class S>
    void addBatch(S set, bool calc_sum = false)
    {
        sum_value = 0;
        for(size_t key : set )
            if (calc_sum)
                add(key,gain(key));
            else
                add(key,0);
    }

    T getSum() const
    {
        return sum_value;
    }

    
    StorageIterator begin() const
    {
        return selected.cbegin();
    }

    StorageIterator end() const
    {
        return selected.cend();
    }

    const std::vector<size_t>& getSelected() const
    {
        return selected;
    }

    void reserve(size_t size) 
    {
        selected.reserve(size);
    }

    size_t size() const
    {
        return selected.size();
    }

    bool empty() const
    {
        return selected.empty();
    }

    void clear() 
    {
        reset();
        selected.clear();
    }

protected:
    StorageType selected;
    T sum_value = 0;

};

template <class T>
class EvaluableFunction : public virtual IncrementalSetFunction<T>
{
    public: 
        virtual T eval() const = 0;
        
        T eval(const std::vector<size_t>& keys, bool reset = true)
        {
            if(reset && !this->empty())
                this->reset();
            
            this->addBatch(keys);
            return this->eval();
        }

        T eval_all(size_t N)
        {
            if(!this->empty())
                this->reset();

            for(size_t i=0;i<N;i++)
                this->add(i,0);
            return this->eval();
        }

};










#endif
