/*
 * File: SLAMInfoGTSAM.cpp
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



#include "mapselect/functions/SLAMInfoGTSAM.h"

#include <cmath>
#include <math.h>
#include <utility>
#include <iostream>

#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/EliminateableFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <Eigen/Cholesky>
#include <Eigen/SVD>

#include "mapselect/utils/Timing.h"
#include "mapselect/utils/MatFuncs.h"


namespace mselect
{

    SLAMInfoGTSAM::SLAMInfoGTSAM(std::shared_ptr<MapDataAlias> map_data_ptr, GTSAMGraphSettings gsettings) : map_data(map_data_ptr)
    {
        graph = gtsam::GaussianFactorGraph::shared_ptr(new gtsam::GaussianFactorGraph());

        SLAMGraphGTSAM gtmap(map_data_ptr, gsettings);
        gtmap.buildSelectionSLAMGraphFactors(graph, landmark_factors);
        dim = map_data->n_frames * 6;

        factors = graph->size();
        base_factors = factors;
        min_value = eval();
        reset();
    }

    double SLAMInfoGTSAM::eval() const
    {
        // double mutli_det = graph->eliminateMultifrontal()->logDeterminant();
        double seq_det = graph->eliminateSequential()->logDeterminant();
        return NegEntropyGaussian(2.0 * seq_det, dim);
    }

    double SLAMInfoGTSAM::gain(size_t key)
    {
        if (outdated)
        {
            graph->at(factors) = nullptr;
            prev_value = SLAMInfoGTSAM::eval(); // std::min<double>(,b_sat);
            outdated = false;
        }
        gtsam::GaussianFactor::shared_ptr factor = landmark_factors[key];
        graph->at(factors) = landmark_factors[key];
        double new_value = SLAMInfoGTSAM::eval(); // std::min<double>(,b_sat);
        graph->at(factors) = nullptr;
        return new_value - prev_value;
    }

    void SLAMInfoGTSAM::update(size_t key)
    {
        graph->at(factors) = landmark_factors[key];
        factors = factors + 1;
        graph->resize(factors + 1);
        outdated = true;

        if (selected.size() % 100==0)
            std::cout << "Selected" << selected.size() << std::endl;
    
    }

    void SLAMInfoGTSAM::print() const
    {
    }

    void SLAMInfoGTSAM::reset()
    {
        factors = base_factors;
        graph->resize(factors + 1);
        graph->at(factors) = nullptr;
        outdated = false;
        prev_value = min_value;
    }

}