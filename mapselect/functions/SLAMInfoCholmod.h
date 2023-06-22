/*
 * File: SLAMInfoCholmod.h
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


#ifndef MAPSELECT_SLAMCHOLMOD_H
#define MAPSELECT_SLAMCHOLMOD_H

#include "mapselect/functions/SLAMInfoDense.h"
#include <Eigen/CholmodSupport>

namespace mselect
{

    typedef Eigen::SparseMatrix<double, Eigen::ColMajor> CSCMatrix;
    typedef Eigen::SparseMatrix<double, Eigen::RowMajor> CSRMatrix;
    typedef Eigen::MappedSparseMatrix<double, Eigen::ColMajor, int> CSCMap;


    /*! @brief This class handles the calls to CHOLMOD and makes use of the functionality
    provided by Eigen. We derive the class to implement additional functionality (by accessing protected members),
    while also simplifying the function calls. 
     */
    class CholmodFactor : public Eigen::CholmodSupernodalLLT<CSCMatrix, Eigen::UpLoType::Upper>
    {
    public:
        CholmodFactor() : Eigen::CholmodSupernodalLLT<CSCMatrix, Eigen::UpLoType::Upper>()
        {
        }

        void updown(bool update, CSCMatrix &c, bool permute = true);

        /*!
        Calculate  log(A+ c.T.dot(T)) - log(A)
        */
        double determinantDelta(bool update, CSCMatrix &c, bool permute = true);

        ~CholmodFactor();

        /* Checks if the underlying matrix factorisation is supernodal. 

        Cholmod doesn't use the supernodal structure of update matrices even in matrices 
        are originally factored with supernodal methods.
        */
        bool isSuper();

        /* Analyse the non-zero structure of a matrix to factorise. 
        */
        void analyzeAsym(CSCMatrix &A);
    };


    /*! @brief The class implements the information gain the map point selection problem for SLAM 
    and uses CHOLMOD to update the information matrix.
     */
    class SLAMInfoCholmod : public SLAMInfoDet
    {

    public:
        SLAMInfoCholmod(std::shared_ptr<MapDataAlias> map_data, GTSAMGraphSettings gsettings = GTSAMGraphSettings());
        SLAMInfoCholmod(std::shared_ptr<MapDataAlias> map_data, const SLAMGraph& graph);

        double gain(size_t key) override;
        double eval() const override;

    protected:
        void initialize();
        void update(size_t key) override;
        void reset() override;

        CSCMatrix landmarkUpdate(size_t key);

        CholmodFactor cholmod_factor;
        std::vector<CSCMatrix> landmark_csc;
        CSCMatrix base_csc;
        size_t evals;
    };

}

#endif
