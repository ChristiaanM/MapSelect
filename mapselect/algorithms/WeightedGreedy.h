/*
 * File: WeightedGreedy.h
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


#ifndef MAPSELECT_WEIGHT_SEARCH_H
#define MAPSELECT_WEIGHT_SEARCH_H

#include "mapselect/functions/WeightedFunctionPair.h"
#include "mapselect/utils/StablePriorityQueue.h"
#include "mapselect/algorithms/LazyGreedy.h"

namespace mselect
{
    //! @brief The normalisation scheme for WeightedBinarySearch or WeightedGreedy 
    enum WeightingNormalisationScheme
    {
        NORM_FULL_SET, // normalise using the full set of selected map points
        NORM_LAZY_GREEDY // normalise the two functions using the lazy greedy algorithm
    };

    //! @brief Additional settings for WeightedBinarySearch
    struct WSearchSettings
    {
        double eps_fc = 1e-6; // precision to check if a function is covered
        double eps_w = 1e-6;  // difference in tested weights after which to stop iterations
        double eps_fm = 0;    // differrence in function value after which to stop iteration s 
        size_t max_iters = 1000; // the maximum number of iterations
        bool verbose = true; // should we print after every iteration
        WeightingNormalisationScheme norm_scheme = WeightingNormalisationScheme::NORM_FULL_SET;
    };


    /*!
       \file
       @brief A iterative version of lazy greedy where the weight of two
       functions are varied between iterations using binary search.
       @param func The weighted pair of functions
       @param eval_keys the set to choose from
       @param heuristic_keys optionally, first include a defined subset of keys
       @param N the number of map points to select
   */
    double WeightedBinarySearch(WeightedFunctionPair &func, const std::vector<size_t> &eval_keys, const std::vector<size_t> &heuristic_keys, size_t N, WSearchSettings settings = WSearchSettings())
    {
        func.clear();
        if (settings.norm_scheme == WeightingNormalisationScheme::NORM_FULL_SET)
        {
           func.normalise_fullset(eval_keys,heuristic_keys);
        }
        else
        {
            func.normalise_greedy(eval_keys,heuristic_keys,N);
        }

        double w_min = 0;
        double w_max = 1;
        double f_m_max = 1;

        for (size_t i = 0; i < settings.max_iters; i++)
        {
            if (settings.verbose && settings.max_iters > 1)
                std::cout << "i=" << i << ' ';

            // reset and run the lazy greedy algorithm using the test value
            func.clear();
            double w_test = (w_min + w_max) / 2.0;
            func.set_w(w_test);
            func.addBatch(heuristic_keys);
            LazyGreedy<double, StablePriorityQueue<double>>(func, eval_keys, N);
            double f_m_test = func.get_normed_f1();

            if (settings.verbose)
                func.print();

            // if covered - check if we need to stop iterating
            if (std::abs(func.get_normed_f2() - 1) < settings.eps_fc)
            {
                w_min = w_test;

                // either the changes is w is small enough that we stop iterating
                // or - the solution is within eps_fm of the last infeasible solution.
                if ((std::abs(w_min - w_max) < settings.eps_w) || (std::abs(f_m_max - f_m_test) < settings.eps_fm))
                    break;
            }
            else // update upper bound of weight w
            {
                w_max = w_test;
                f_m_max = f_m_test;
            }
        }

        return w_min;
    }


    double WGreedy(WeightedFunctionPair &func, const std::vector<size_t> &eval_keys, const std::vector<size_t> &heuristic_keys, size_t N, WSearchSettings settings = WSearchSettings())
    {
        settings.max_iters = 1;
        return WeightedBinarySearch(func,eval_keys,heuristic_keys,N,settings);
    }

}
#endif