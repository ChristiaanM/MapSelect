
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

#ifndef WEIGHT_SEARCH_H
#define WEIGHT_SEARCH_H

#include "mapselect/utils/StablePriorityQueue.h"
#include "mapselect/algorithms/LazyGreedy.h"
#include "mapselect/functions/WSearchPair.h"


enum WNormScheme {
FULL_SET,
LAZY_GREEDY
};


struct WSearchSettings
{
    double eps_fc = 1e-6; //precision to check if a function is covered
    double eps_w = 1e-6; //exit the seach if the tested weights are within this value from one another
    double eps_fm = 0; // minimum 
    size_t max_iters = 1000; 
    bool verbose = true;
    WNormScheme norm_scheme = WNormScheme::FULL_SET;
};

double WBinarySearch(WSearchPair &func, const std::vector<size_t> &eval_keys, const std::vector<size_t> &heuristic_keys, size_t N, WSearchSettings settings = WSearchSettings())
{
    func.clear();
    if(settings.norm_scheme = WNormScheme::LAZY_GREEDY)
    {
        func.set_w(0);
        func.addBatch(heuristic_keys);
        double c0 = func.func_c->eval();
        double m0 = func.func_m->eval();
        func.addBatch(eval_keys);
        double c1 = func.func_c->eval();
        double m1 = func.func_m->eval();
        func.normalise(m0, m1, c0, c1);
    }
    else 
    {
        
        func.addBatch(heuristic_keys);
        double c0 = func.func_c->eval();
        double m0 = func.func_m->eval();
        func.set_w(0);
        LazyGreedy<double, StablePriorityQueue<double>>(func, eval_keys, N);
        double c1 = func.func_c->eval();
        
        func.clear();
        func.set_w(1);
        func.addBatch(heuristic_keys);
        LazyGreedy<double, StablePriorityQueue<double>>(func, eval_keys, N);
        double m1 = func.func_m->eval();
        func.normalise(m0, m1, c0, c1);
    }

    double w_min = 0;
    double w_max = 1;
    double f_m_max = 1;

    for (size_t i = 0; i < settings.max_iters; i++)
    {
        if (settings.verbose)
            std::cout << "i=" << i << ' ';

        // reset and run the lazy greedy algorithm using the test value
        func.clear();
        double w_test = (w_min + w_max) / 2.0;
        func.set_w(w_test);
        func.addBatch(heuristic_keys);
        LazyGreedy<double, StablePriorityQueue<double>>(func, eval_keys, N);
        double f_m_test = func.get_fmval();

        if (settings.verbose)
            func.print();

        // if covered - check if we need to stop iterating
        if (fabs(func.get_fcval() - 1) < settings.eps_fc)
        {
            w_min = w_test;

            // either the changes is w is small enough that we stop iterating
            // or - the solution is within eps_fm of the last infeasible solution.
            if ((fabs(w_min - w_max) < settings.eps_w) || (fabs(f_m_max - f_m_test) < settings.eps_fm))
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

#endif