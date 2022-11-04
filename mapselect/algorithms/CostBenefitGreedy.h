
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



#ifndef COST_BENEFIT_GREEDY_H
#define COST_BENEFIT_GREEDY_H

#include "mapselect/functions/SetFunction.h"
#include <vector>


template <class V, class Q>
V CostBenefitGreedy(EvaluableFunction<V> &func, const std::vector<size_t> &eval_keys, const std::vector<double>& costs, double B = std::numeric_limits<double>::max())
{
    std::vector<size_t> start_keys = func.getSelected();
    double start_cost = 0;
    for(size_t key : start_keys)
        start_cost+=costs[key];
    
    // Cost-Benefit
    
    double current_cost = start_cost;
    {

        std::vector<std::pair<size_t, V>> key_values;
        key_values.reserve(eval_keys.size());
        for (size_t key : eval_keys)
            key_values.emplace_back(key, func.gain(key) / costs[key]);

        Q queue(key_values);

        while (current_cost < B && !queue.empty())
        {
            auto top = queue.top();

            size_t top_key = top.first;
            queue.pop();

            
            double cost_benefit = func.gain(top_key) / cost;
            double next_cost_benefit = 0;
            
            if (queue.size())
                next_cost_benefit = queue.top().second;

            if (cost_benefit >= next_cost_benefit)
            {
                func.add(top_key);
                current_cost+=costs[top_key];
            }
            else if (cost_benefit > 0 && current_cost+costs[top_key] < B)
                    queue.emplace(top_key, new_gain);
        }

    }

    
    std::vector<size_t> cost_benefit_selected = func.getSelected();
    V cost_benefit_value = func.eval();


    // Reset
    func.clear();
    func.addBatch(start_keys);
    // Gains Only
    current_cost = start_cost;

    {
        std::vector<std::pair<size_t, V>> key_values;
        key_values.reserve(eval_keys.size());
        for (size_t key : eval_keys)
            key_values.emplace_back(key, func.gain(key));

        Q queue(key_values);

        while (current_cost < B && !queue.empty())
        {
            auto top = queue.top();

            size_t top_key = top.first;
            queue.pop();

            
            V gain = func.gain(top_key);
            V next_gain = 0;
            
            if (queue.size())
                next_gain = queue.top().second;

            if (gain >= next_gain)
            {
                func.add(top_key);
                current_cost+=costs[top_key];
            }
            else if (gain > 0 && current_cost+costs[top_key] < B)
                    queue.emplace(top_key, new_gain);
        }

    }    

    double gain_only_value = func.eval();

    if (cost_benefit_value > gain_only_value)
    {
        func.clear();
        func.addBatch(cost_benefit_selected);
        return cost_benefit_value;
    }
    return gain_only_value;
}


#endif