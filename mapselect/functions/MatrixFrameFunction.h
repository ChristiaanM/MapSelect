/*
 * File: MatrixFrameFunction.h
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
#ifndef MAPSELECT_MATRIX_FRAME_FUNCTION_H
#define MAPSELECT_MATRIX_FRAME_FUNCTION_H


#include "mapselect/functions/FrameFunction.h"
#include "mapselect/utils/MatFuncs.h"

namespace mselect
{
    /*! @brief Abstract class for FrameFunctions where the value of each frame
    is the log determinant of a matrix. 
    */
    template <class EstMat>
    class MatrixFrameFunction : public FrameFunction
    {
    public:
        typedef std::vector<EstMat, Eigen::aligned_allocator<EstMat>> EstMatVector;

        virtual EstMatVector* getUpdateMatrices(size_t map_key) = 0;

        MatrixFrameFunction(std::shared_ptr<MapDataAlias> map_data_ptr) : FrameFunction(map_data_ptr)
        {
            frame_K.resize(map_data_ptr->n_frames);
            frame_value.resize(map_data_ptr->n_frames);
            SLAMGraphSettings gsettings;
            gsettings.octave_scaling = false;
            graph = SLAMGraph(gsettings);

            clear();
        }

        bool precompute(size_t map_key) override
        {
            update_matrices_ptr = getUpdateMatrices(map_key);
            if (!update_matrices_ptr)
                return false;
            return update_matrices_ptr->size() > 0;
        }

        double evalFrame(size_t key) const override
        {
            return sym_log_det(frame_K[key]);
        }

        inline bool isZero(const EstMat& mat)
        {
            return mat(0,0) == 0;
        }

        void updateFrame(size_t map_key, size_t frame_key, size_t obs_index) override
        {
            const EstMat& update = update_matrices_ptr->operator[](obs_index);
            if (!isZero(update))
                frame_K[frame_key] += update;
        }

        double gainFrame(size_t map_key, size_t frame_key, size_t obs_index) override
        {
            const EstMat& update = update_matrices_ptr->operator[](obs_index);
            if (isZero(update))
                return 0;
            EstMat sum = frame_K[frame_key] + update;
            return sym_log_det(sum) - frame_value[frame_key];
        }

    protected:
        void reset() override
        {
            EstMat tmp =  graph.getApproxPrior();
            double prior_logdet = sym_log_det(tmp);

            for (size_t i = 0; i < map_data->n_frames; i++)
            {
                frame_K[i] = tmp;
                frame_value[i] = prior_logdet;
            }
        }
 
        EstMatVector* update_matrices_ptr;
        EstMatVector frame_K;
        SLAMGraph graph;
    };

}

#endif