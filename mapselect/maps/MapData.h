/*
 * File: MapData.h
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



#ifndef MAPSELECT_MAP_DATA_H
#define MAPSELECT_MAP_DATA_H

#include "mapselect/maps/Selection.h"
#include <string>


namespace mselect
{
    /*! @brief
    Map Data is an interface for different map data classes to implement methods to both
    selected the associated map points.
    */
    class MapData
    {
    public:
        virtual void select(const Selector &select) = 0;
        virtual void saveMap(std::string output) = 0;
        virtual size_t getMapSize() = 0;
    };

}

#endif