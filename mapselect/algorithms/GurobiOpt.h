/*
 * File: GurobiOpt.h
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

#ifndef MAPSELECT_GUROBI_OPT_H
#define MAPSELECT_GUROBI_OPT_H

#include "mapselect/maps/MapDataAlias.h"

namespace mselect
{

    /*!
    \brief Paramater settings for GurobiIPCover function.

    Defines the map point selection problem of the following form
    \f[
        \min \mathbf{q} \mathbf{x} + \lambda \times  \mathbf{\zeta}
    \f]
    \f[
        \mathbf{A} \mathbf{x} +   \mathbf{\zeta} = b \\
    \f]
    \f[
        \Sigma x_i = n \\
    \f]

    where N,b, \lambda are available parameters
    */
    class GurobiSettings
    {
    public:
        double lambda = 0; /* Test B*/
        size_t b = 0;
        size_t N = 0;
        //! Heuristic keys which should always be selected, i.e.  x_i = 1 if x_i in forced_keys
        std::vector<size_t> *forced_keys;

        /*!
        The MIP threshold to accept answers
        */
        double mip_gap = 0.01;

        /*!
        Additionally, also constrain 
        \f[
             Ax > b_min
        \f],
        this optionally forces some map points to be selected for all cameras.
        */
        size_t b_min = 0;


        //! if return_deleted == true instead returns the set of map points not selected.
        bool return_deleted = false;

        //! @todo Add quadratic penalty again - current code relies on frame_keypoints
        //bool quad_penalty = true;
        double min_dist = 0.0;
    };

    /*! 
    @file Build a map point selection problem using the set multicover formulation
    and solve it using the GUROBI toolbox.

    @param data Alias of the map data
    @param settings  Paramater settings, see GurobiSettings for more detail.
    */
    std::vector<size_t> GurobiIPCover(const MapDataAlias &data, const GurobiSettings &settings);
}

#endif