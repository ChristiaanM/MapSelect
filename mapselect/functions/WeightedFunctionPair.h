/*
 * File: WeightedFunctionPair.h
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


#ifndef MAPSELECT_WSEARCH_H
#define MAPSELECT_WSEARCH_H

#include "mapselect/functions/SaturatedSetFunction.h"
#include "mapselect/algorithms/LazyGreedy.h"
#include "mapselect/utils/StablePriorityQueue.h"

namespace mselect
{
    /*! @brief A class that combines two utility functions to be used alongside
        WeightedGreedy 

        While these functions can be optimised with LazyGreedy, you should then manually 
        call the normalise functions - or else the unnormalised sum of the functions will 
        be maximised.
    */
    class WeightedFunctionPair : public IncrementalSetFunction<double>
    {
        typedef std::shared_ptr<IncrementalSetFunction<double>> FunctionType;

    public:
        WeightedFunctionPair(FunctionType func_1, FunctionType func_2, std::string f1_name = "F_1", std::string f2_name = "F_2") : func_1(func_1),
                                                                                                                                   func_2(func_2),
                                                                                                                                   f1_name(f1_name),
                                                                                                                                   f2_name(f2_name),
                                                                                                                                   f1_w(0.5),
                                                                                                                                   f2_w(0.5),
                                                                                                                                   f1_offset(0),
                                                                                                                                   f2_offset(0),
                                                                                                                                   f1_norm(1),
                                                                                                                                   f2_norm(1)
        {
        }



        double gain(size_t key) override
        {
            double gain_m = f1_w * func_1->gain(key) * f1_norm;
            double gain_c = f2_w * func_2->gain(key) * f2_norm;
            return gain_m + gain_c;
        }

        double eval() const override
        {
            double value_m = f1_w * func_1->eval() * f1_norm;
            double value_c = f2_w * func_2->eval() * f2_norm;
            return value_m + value_c;
        }

        void set_w(double w)
        {
            f1_w = std::max<double>(std::min<double>(w, 1), 0);
            f2_w = 1 - f1_w;
        }

        double get_normed_f2()
        {
            return (func_2->eval() - f2_offset) * f2_norm;
        }

        double get_normed_f1()
        {
            return (func_1->eval() - f1_offset) * f1_norm;
        }

        void print()
        {
            double normed_f1 = get_normed_f1();
            double normed_f2 = get_normed_f2();

            std::cout << "w=" << f1_w << " ";
            std::cout << f1_name << "=" << normed_f1 << " ";
            std::cout << f2_name << "=" << normed_f2 << std::endl;
        }

        void normalise(double f1_0, double f1_max, double f2_0, double f2_max)
        {
            if (f1_max - f1_0 > 0)
                f1_norm = 1.0 / (f1_max - f1_0);
            else
                f1_norm = 0;
            if (f2_max - f2_0 > 0)
                f2_norm = 1.0 / (f2_max - f2_0);
            else
                f2_norm = 0;
            f1_offset = f1_0;
            f2_offset = f2_0;
        }

        void normalise_fullset(const std::vector<size_t>& eval_keys, const std::vector<size_t>& heuristic_keys)
        {
            set_w(0);
            addBatch(heuristic_keys);
            double f1_0 = func_1->eval();
            double f2_0 = func_2->eval();
            addBatch(eval_keys);
            double f1_max = func_1->eval();
            double f2_max = func_2->eval();
            normalise(f1_0, f1_max, f2_0, f2_max);
            clear();
        }

        void normalise_greedy(const std::vector<size_t>& eval_keys, const std::vector<size_t>& heuristic_keys,size_t N)
        {
            addBatch(heuristic_keys);
            double f1_0 = func_1->eval();
            double f2_0 = func_2->eval();
            set_w(1);
            LazyGreedy<double, StablePriorityQueue<double>>(*this, eval_keys, N);
            double f1_max = func_1->eval();
            clear();
            addBatch(heuristic_keys);
            set_w(0);
            LazyGreedy<double, StablePriorityQueue<double>>(*this, eval_keys, N);
            double f2_max = func_2->eval();
            normalise(f1_0, f1_max, f2_0, f2_max);
            clear();
        }

        void print() const override
        {

        }


    protected:
        void update(size_t key) override
        {
            func_1->add(key);
            func_2->add(key);
        }
    private:
        void reset() override
        {
            func_1->clear();
            func_2->clear();
        }

    protected:
        FunctionType func_1, func_2;
        const std::string f1_name, f2_name;

    private:
        double f1_w;
        double f2_w;
        double f1_offset;
        double f2_offset;
        double f1_norm;
        double f2_norm;
    };

}

#endif
