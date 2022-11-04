
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



#ifndef GUROBI_OPT_H
#define GUROBI_OPT_H

#include "mapselect/maps/MapDataAlias.h"

struct GurobiQIPSettings
{
    double lambda = 0;
    size_t b = 0;
    double min_dist = 0.0;
    size_t N = 0;
    std::vector<size_t> *forced_keys;
    bool quad_penalty = true;
    double mip_gap = 0.01;
    size_t b_min = 0;

};


std::vector<size_t> GurobiCoverQIP(const MapDataAlias &data, const GurobiQIPSettings &settings);


#endif