/*
 * File: SLAMInfoGTSAM.h
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



#ifndef MAPSELECT_SLAMBASE_H
#define MAPSELECT_SLAMBASE_H
#include <stddef.h>
#include <limits>

#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include "mapselect/functions/SetFunction.h"
#include "mapselect/functions/SaturatedSetFunction.h"
#include "mapselect/slam/SLAMGraphGTSAM.h"

namespace mselect
{

    /*! @brief The class implements the information gain the map point selection problem for SLAM 
    using the GTSAM library and uses GTSAM factor graphs to calculate the information gain.
     */
    class SLAMInfoGTSAM : public IncrementalSetFunction<double>
    {
    public:
        SLAMInfoGTSAM(std::shared_ptr<MapDataAlias> map_data, GTSAMGraphSettings gsettings = GTSAMGraphSettings());

        double gain(size_t key);

        double eval() const override;

        void print() const override;

    protected:
        void update(size_t key);
        void reset() override;

        gtsam::GaussianFactorGraph::shared_ptr graph;
        std::vector<gtsam::GaussianFactor::shared_ptr> landmark_factors;

        size_t factors;
        size_t base_factors;

        double min_value;
        double max_value;

        double prev_value;
        bool outdated;

        size_t dim;

        std::shared_ptr<MapDataAlias> map_data;
    };

}

#endif
