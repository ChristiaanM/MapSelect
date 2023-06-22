/*
 * File: SLAMInfoDense.h
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


#ifndef MAPSELECT_SLAMDENSE_H
#define MAPSELECT_SLAMDENSE_H

#include "mapselect/functions/SetFunction.h"
#include "mapselect/functions/LoopCover.h"
#include "mapselect/maps/MapDataAlias.h"

#include "mapselect/slam/SLAMGraphGTSAMSettings.h"
#include "mapselect/slam/SLAMGraph.h"


#include <Eigen/Sparse>

namespace mselect
{
    /*! @brief A dense implementation of the map point selection problem for SLAM

    This class implements the map point selection problem using dense matrices,
    and the matrix determinant lemma. The Cholmod implementation derives from this
    base class.

    @todo 
     */
    class SLAMInfoDet : public IncrementalSetFunction<double>
    {

    typedef Eigen::SparseMatrix<double, Eigen::ColMajor> CSCMat;
    

    public:
        SLAMInfoDet(std::shared_ptr<MapDataAlias> map_data, GTSAMGraphSettings gsettings = GTSAMGraphSettings());

        SLAMInfoDet(std::shared_ptr<MapDataAlias> map_data, const SLAMGraph& graph);
        double gain(size_t key);
        double eval() const override;
        void print() const override;

        /*! Get the block from the covariance matrix (and calculate it if it hasn't been calculated). NOTE - this calculates the full covariance matrix.
        @param keys The pose keys for which to extract the matrix
        */
        MatXX getMarginalCov(std::vector<size_t> keys);
        void updateCov();

    protected:
        void update(size_t key);

        void reset() override;
        void updateK(size_t key);
        void resetK();


        /*! This allocates creates a sparse compressed column matrix with the non-zero structure
         that K would have, if we select all the landmarks. This is used for Cholmod to analyze the structure for the full problem,
         and use the same elimination ordering regardless of the selected number of map points.

         Eigen supports building CSC matrices from sets of triples, so we use that instead of forming the actual matrix.
        @todo : Just build the CSC matrix instead of using triplets.
        */
        void allocateKsparse();

        std::vector<MatXX> landmark_A;
        std::vector<std::vector<size_t>> landmark_keys;

        double origin_info = 1e6;
        double prior_info = 1e-4;

        MatXX K;
        MatXX C;
        CSCMat Ksparse;


        bool K_allocated;
        bool cov_computed;

        size_t dim;
        size_t n_frames;
    };

}

#endif