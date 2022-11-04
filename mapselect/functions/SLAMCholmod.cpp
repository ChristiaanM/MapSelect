

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


#include "mapselect/functions/SLAMCholmod.h"

typedef Eigen::Triplet<double> SparseTriplet;

void CholmodFactor::updown(bool update, CSCMatrix &c, bool permute)
{
    cholmod_common &common = cholmod();
    cholmod_sparse c_view = Eigen::viewAsCholmod(c);
    c_view.stype = 0;
    if (permute)
    {
        cholmod_sparse *c_perm = cholmod_submatrix(&c_view, (int *)m_cholmodFactor->Perm, m_cholmodFactor->n, NULL, -1, true, true, &common);
        cholmod_updown(update, c_perm, m_cholmodFactor, &common);
        cholmod_free_sparse(&c_perm, &common);
    }
    else
        cholmod_updown(update, &c_view, m_cholmodFactor, &common);
}

double CholmodFactor::determinantDelta(bool update, CSCMatrix &c, bool permute)
{
    cholmod_common &common = cholmod();
    double before = logDeterminant();
    cholmod_factor *backup = cholmod_copy_factor(m_cholmodFactor, &common);
    updown(update, c, permute);
    double after = logDeterminant();
    cholmod_free_factor(&m_cholmodFactor, &common);
    m_cholmodFactor = backup;

    return (after - before);
}

CholmodFactor::~CholmodFactor()
{
    cholmod_common &common = cholmod();
    //if (factor_copy != nullptr)
    //    cholmod_free_factor(&factor_copy, &common);
}

bool CholmodFactor::isSuper()
{
    return m_cholmodFactor->is_super;
}

void CholmodFactor::analyzeAsym(CSCMatrix &A)
{
    cholmod_common &common = cholmod();
    if (m_cholmodFactor)
    {
        cholmod_free_factor(&m_cholmodFactor, &common);
        m_cholmodFactor = 0;
    }
    cholmod_sparse A_view = Eigen::viewAsCholmod(A);
    // Asymetrical!
    A_view.stype = 0;

    m_cholmodFactor = cholmod_analyze(&A_view, &common);

    this->m_isInitialized = true;
    this->m_info = Eigen::Success;
    m_analysisIsOk = true;
    m_factorizationIsOk = false;
}

SLAMEntropyCholmod::SLAMEntropyCholmod(std::shared_ptr<MapDataAlias> map_data_ptr, GTSAMGraphSettings gsettings) : SLAMEntropyDet(map_data_ptr, gsettings)
{

    // TODO : Avoid dense K allocations

    landmark_csc.reserve(landmark_factors.size());
    for (size_t i = 0; i < landmark_factors.size(); i++)
    {
        landmark_csc.push_back(landmarkUpdate(i));
    }

    for (size_t i = 0; i < map_data_ptr->n_points; i++)
    {
        updateK(i);
    }

    cholmod_factor.analyzePattern(K.sparseView());
    resetK();
    cholmod_factor.compute(K.sparseView());
}

void SLAMEntropyCholmod::reset()
{
    // TODO : Avoid dense K allocations
    resetK();
    cholmod_factor.compute(K.sparseView());
}

CSCMatrix SLAMEntropyCholmod::landmarkUpdate(size_t key)
{
    const Eigen::MatrixXd &A = landmark_A[key];
    const std::vector<size_t> &keys = landmark_keys[key];
    size_t r = A.rows();

    size_t n = dim;
    CSCMatrix csc(n, r);
    if (r == 0)
        return csc;
    csc.reserve(Eigen::VectorXi::Constant(r, keys.size() * 6));

    for (size_t i = 0; i < r; i++)
    {
        for (size_t k = 0; k < keys.size(); k++)
        {
            size_t kj_A = k * 6;
            size_t kj_K = keys[k] * 6;

            for (size_t j = 0; j < 6; j++)
            {
                csc.insert(kj_K + j, i) = A(i, kj_A + j);
            }
        }
    }
    return csc;
}

double SLAMEntropyCholmod::gain(size_t key)
{
    CSCMatrix csc = landmark_csc[key];
    if (csc.cols() == 0)
        return 0.0;
    ++evals;
    // TIC(ldet);
    double gain = cholmod_factor.determinantDelta(true, csc, true) * 0.5;
    // TOC(ldet);
    // time_llt+=TIME(ldet);
    return gain;
}
void SLAMEntropyCholmod::update(size_t key)
{

    CSCMatrix csc = landmark_csc[key];
    if (csc.cols())
    {
        cholmod_factor.updown(true, csc, true);
    }

    // We don't need K, but can't release it earlier due to the nested constructor
    // so we do it here instead.

    // TODO : Do this better.
    releaseK();
    evals = 0;
}

double SLAMEntropyCholmod::eval() const
{
    // std::cout << "cholmod eval" << std::endl;
    return NegEntropyGaussian(cholmod_factor.logDeterminant(), dim);
}
