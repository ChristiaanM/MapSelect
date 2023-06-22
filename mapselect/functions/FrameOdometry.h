/*
 * File: FrameOdometry.h
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
#ifndef MAPSELECT_FRAME_ODOMETRY_H
#define MAPSELECT_FRAME_ODOMETRY_H

#include "mapselect/functions/MatrixFrameFunction.h"

namespace mselect
{
    class FrameOdometryMemory : public MatrixFrameFunction<PoseInfoMat>
    {
    public:
        FrameOdometryMemory(std::shared_ptr<MapDataAlias> map_data_ptr) : MatrixFrameFunction<PoseInfoMat>(map_data_ptr)
        {
            odometry_parents = graph.calcOdomParents(map_data_ptr);
        }

        EstMatVector *getUpdateMatrices(size_t map_key) override
        {
            update_matrices = graph.selectionOdometryMatrices(map_data, odometry_parents, map_key);
            return &update_matrices;
        }

    protected:
        EstMatVector update_matrices;
        std::vector<size_t> odometry_parents;
    };

    class FrameOdometryFast : public MatrixFrameFunction<PoseInfoMat>
    {
    public:
        FrameOdometryFast(std::shared_ptr<MapDataAlias> map_data_ptr) : MatrixFrameFunction<PoseInfoMat>(map_data_ptr)
        {
            std::vector<size_t> odometry_parents = graph.calcOdomParents(map_data_ptr);
            map_update_matrices.resize(map_data_ptr->n_points);
            for (size_t map_key = 0; map_key < map_data_ptr->n_points; map_key++)
                map_update_matrices[map_key] = graph.selectionOdometryMatrices(map_data_ptr, odometry_parents, map_key);
        }

        EstMatVector *getUpdateMatrices(size_t map_key) override
        {
            return &map_update_matrices[map_key];
        }

    protected:
        std::vector<EstMatVector> map_update_matrices;
    };
}

#endif