/*
 * File: MatFuncs.h
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


#ifndef MAPSELECT_FUNCS_H
#define MAPSELECT_FUNCS_H

#include <Eigen/Core>

namespace mselect
{

    double inline NegEntropyGaussian(double logdet_K, size_t dim)
    {
        double c0 = 0.5 * dim + 0.5 * log(2 * M_PI);
        return 0.5 * logdet_K - c0;
    }


    /*
    For larger matrices it is more numerically stable to calculate the cholesky decomposition.
    and sum the log-det of the values on the triangular
    */
    inline double sym_log_det(const Eigen::Matrix<double,6,6>  &mat)
    {

        Eigen::LLT<Eigen::Matrix<double,6,6>> llt = mat.selfadjointView<Eigen::Lower>().llt();
        const Eigen::TriangularView<const Eigen::Matrix<double,6,6>, 1U> L = llt.matrixL();
        double sum = 0;
        for (int i = 0; i < 6; i++)
            sum += log(L.coeff(i, i));
        return 2 * sum;
    }

    /*
    For a 3x3 matrix, it is much faster to use the default closed form 
    determinant and take the log of that value
    */
    inline double sym_log_det(const Eigen::Matrix<double,3,3>  &mat)
    {
        return  log(mat.determinant());
    }

}

#endif