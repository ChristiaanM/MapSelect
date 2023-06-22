/*
 * File: SetFunction.h
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



#ifndef MAPSELECT_SET_FUNCTION_H
#define MAPSELECT_SET_FUNCTION_H

#include <stddef.h>
#include <limits>
#include <iostream>
#include <memory>
#include <deque>
#include <vector>

namespace mselect
{

    /*!
    @brief Base abstract class used by all incremental functions used by greedy algorithms.

    All derived classes to be able evaluate the marginal gain f(S union {x_key})
    -f(S), update the set with an additional item and clear the set (which calls
    the reset virtual function) This class manages the selected set of map
    points and provides a template for other derived functions to overwrite.

    This functions provide to checking that duplicate elements are not added to the set.
    It is up to the user to avoid adding duplicate elements to the set.
    */
    template <class T>
    class IncrementalSetFunction
    {
    public:
        // Evaluate the marginal gain of a item with the given key
        virtual T gain(size_t key) = 0;

        // Evaluate the function value for the currently selected set
        virtual T eval() const = 0;

        // Print relevant information
        virtual void print() const = 0;
    protected:
        // add a a item with a given key to the set
        virtual void update(size_t key) = 0;

    private:
        /* a virtual function which is called whenever the set of items is
            cleared. You can invoke this function, by using the clear function
         */
        virtual void reset() = 0;

    public:
        typedef std::vector<size_t> StorageType;
        typedef StorageType::const_iterator StorageIterator;

        void add(size_t key)
        {
            update(key);
            selected.push_back(key);
        }

        template <class S>
        void addBatch(S set)
        {
            for (size_t key : set)
                add(key);
        }

        StorageIterator begin() const
        {
            return selected.cbegin();
        }

        StorageIterator end() const
        {
            return selected.cend();
        }

        const std::vector<size_t> &getSelected() const
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

        // Clear the set and calls reset
        void clear()
        {
            reset();
            selected.clear();
        }


        // helper function to evaluate a set of points
        T eval(const std::vector<size_t> &keys, bool reset = true)
        {
            if (reset && !this->empty())
                this->clear();

            this->addBatch(keys);
            return this->eval();
        }

        // helper function to evaluate all keys up to N
        T eval_all(size_t N)
        {
            if (!this->empty())
                this->clear();

            for (size_t i = 0; i < N; i++)
                this->add(i, 0);
            return this->eval();
        }

    protected:
        StorageType selected;
    };

    /*!
    @brief A depreciated class which was used when not all incremental set functions required 
    the ability to evaluate the function
    */
    /*template <class T>
    class EvaluableFunction : public virtual IncrementalSetFunction<T>
    {

    };
    */

}

#endif
