
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


#include "mapselect/functions/SLAMBase.h"
#include "mapselect/utils/Timing.h"



#include <math.h>
#include <utility>
#include <iostream>


#include <gtsam/inference/Symbol.h>
//#include "cblas.h"

#include <Eigen/Cholesky>
#include <Eigen/SVD>


#include <gtsam/inference/EliminateableFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianBayesTree.h>



/*
extern "C" void dgetrf_(int *dim1, int *dim2, double *a, int *lda, int *ipiv, int *info);
extern "C" void dgetrs_(char *TRANS, int *N, int *NRHS, double *A, int *LDA, int *IPIV, double *B, int *LDB, int *INFO);
extern "C" void dgetri_(int *N, double *A, int *LDA, int *IPIV, double *WORK, int *LWORK, int *INFO);

int mat_inv(double *mat, int dim)
{
    char trans = 'N';
    int nrhs = 1;
    int LDA = dim;
    int LDB = dim;
    int info;
    int ipiv[dim + 1];
    int lwork = dim * dim;
    double *work = new double[lwork];

    dgetrf_(&dim, &dim, mat, &LDA, ipiv, &info);
    if (info != 0)
        return info;

    //dgetrs_(&trans, &dim, &nrhs, mat, &LDA, ipiv, mat, &LDB, &info);
    dgetri_(&dim, mat, &dim, ipiv, work, &lwork, &info);

    delete work;
    return info;
}
*/



SLAMEntropy::SLAMEntropy(std::shared_ptr<MapDataAlias> map_data_ptr, GTSAMGraphSettings gsettings) : map_data(map_data_ptr)
{
    graph = gtsam::GaussianFactorGraph::shared_ptr(new gtsam::GaussianFactorGraph());

    SLAMGraphGTSAM gtmap(map_data_ptr, gsettings);
    gtmap.landmarkSelection(graph, landmark_factors);
    dim = map_data->n_frames*6;
        
    factors = graph->size();
    base_factors = factors;
    graph->reserve(factors+landmark_factors.size());
    graph->resize(factors+1);

    min_value = eval();
    max_value = eval_all(map_data_ptr->n_points);
    b_sat = max_value;
    SLAMEntropy::reset();

    //std::cout << graph->hessian().first.block<6,6>(0,0) << std::endl;

    

}


double SLAMEntropy::eval() const
{
    return NegEntropyGaussian(2.0*graph->eliminateSequential()->logDeterminant(),dim);
}

double SLAMEntropy::gain(size_t key)
{
    if (outdated)
    {
        graph->at(factors) = nullptr;
        prev_value = SLAMEntropy::eval(); //std::min<double>(,b_sat);
        outdated = false;
    }
    gtsam::GaussianFactor::shared_ptr factor = landmark_factors[key];
    graph->at(factors) = landmark_factors[key];
    double new_value = SLAMEntropy::eval();// std::min<double>(,b_sat);
    graph->at(factors) = nullptr;
    return new_value-prev_value;
}

void SLAMEntropy::update(size_t key)
{
    graph->at(factors) = landmark_factors[key];
    factors = factors + 1;
    graph->resize(factors + 1);
    outdated = true;
}


double SLAMEntropy::getBmin() const 
{
    return min_value;
}

double SLAMEntropy::getBmax() const 
{
    return max_value;
}

void SLAMEntropy::print() const 
{

}

void SLAMEntropy::reset() 
{
    factors = base_factors;
    graph->resize(factors + 1);
    graph->at(factors) = nullptr;
    outdated = false;
    prev_value = min_value;
}


gtsam::Matrix extract_cov(const gtsam::KeyVector &kv, const gtsam::Matrix &mat)
{
    size_t D = kv.size() * 6;
    gtsam::Matrix c(D, D);
    for (size_t i = 0; i < kv.size(); i++)
    {
        gtsam::Key ki = kv[i];

        for (size_t j = 0; j < kv.size(); j++)
        {
            gtsam::Key kj = kv[j];
            size_t ci = i * 6;
            size_t cj = j * 6;

            size_t Ci = gtsam::Symbol(ki).index() * 6;
            size_t Cj = gtsam::Symbol(kj).index() * 6;

            c.block<6, 6>(ci, cj) = mat.block<6, 6>(Ci, Cj);
        }
    }
    return c;
}


Eigen::MatrixXd extract_block(const std::vector<size_t> &kv, const Eigen::MatrixXd &mat)
{
    size_t D = kv.size() * 6;
    Eigen::MatrixXd c(D, D);
    for (size_t i = 0; i < kv.size(); i++)
    {
        size_t ki = kv[i];

        for (size_t j = 0; j < kv.size(); j++)
        {
            size_t kj = kv[j];
            size_t ci = i * 6;
            size_t cj = j * 6;

            size_t Ci = ki * 6;
            size_t Cj = kj * 6;

            c.block<6, 6>(ci, cj) = mat.block<6, 6>(Ci, Cj);
        }
    }
    return c;
}





gtsam::Matrix extract_cov_row(const gtsam::KeyVector &kv, const gtsam::Matrix &mat)
{
    size_t D = kv.size() * 6;
    size_t N = mat.cols();
    gtsam::Matrix c(D, N);
    
    for (size_t i = 0; i < kv.size(); i++)
    {
        
        gtsam::Key ki = kv[i];
        size_t ci = i*6;
        size_t Ci = gtsam::Symbol(ki).index() * 6;
        for(size_t ii=0;ii<6;ii++)
        {
            
            c.row(ci+ii) = mat.row(Ci+ii);
        }
    }
    return c;
}



#include <cmath>
SLAMEntropyDet::SLAMEntropyDet(std::shared_ptr<MapDataAlias> map_data_ptr, GTSAMGraphSettings gsettings): SLAMEntropy(map_data_ptr, gsettings)
{
    landmark_A.resize(map_data_ptr->n_points);
    landmark_keys.resize(map_data_ptr->n_points);

    for(size_t i=0;i<map_data_ptr->n_points;i++)
    {
        auto factor = landmark_factors[i];
        if (factor)
        {
            landmark_A[i] = factor->jacobian().first;
            std::vector<size_t> keys;
            for(gtsam::Key ki : factor->keys())
            {
                keys.push_back(gtsam::Symbol(ki).index());
            }       
            landmark_keys[i] = keys;       
        }
    
        
    }
    
    
    SLAMEntropyDet::reset();

}

void SLAMEntropyDet::resetK()
{
    K = Eigen::MatrixXd::Zero(dim,dim);
    for(size_t i=0;i<6;i++)
        K(i,i) = origin_info;
    for(size_t i=6;i<dim;i++)
        K(i,i) = prior_info;
}

void SLAMEntropyDet::releaseK()
{
    K.resize(0,0);
}

double SLAMEntropyDet::eval() const
{
    Eigen::LDLT<Eigen::MatrixXd> ldlt(K);
    auto diag = ldlt.vectorD();
    double logsum = 0;
    for(size_t i=0;i<diag.rows();i++)
        logsum+=log(diag(i));

    return NegEntropyGaussian(logsum,dim);
}


void SLAMEntropyDet::reset() 
{
    SLAMEntropy::reset();
    resetK();
    cov_computed = false;
}


void SLAMEntropyDet::updateCov()
{
    if (!cov_computed)
    {
        C= K.inverse();

        // We used to use the BLAS functions, as apposed to eigen, but this is bad for portability.....
        //       
        //int error =mat_inv(C.data(), C.rows());
        //if (error != 0)
        //    throw "Covariance Calculation Failed";
        cov_computed = true;
        
    }
}


double SLAMEntropyDet::gain(size_t key)
{
    updateCov();

    const auto& A = landmark_A[key];
    const auto& kv = landmark_keys[key];
    if (A.rows()==0)
        return 0.0;
    
    
    //TIC(llt);
    Eigen::MatrixXd c = extract_block(kv,C);
    Eigen::MatrixXd mat =(Eigen::MatrixXd::Identity(A.rows(), A.rows()) + A * c * A.transpose());
    Eigen::LLT<Eigen::MatrixXd> llt(mat);
    double gain_val = llt.matrixL().toDenseMatrix().diagonal().array().log().sum(); // x 2.0 for logdet x 0.5 for entropy = 1.0
    //TOC(llt);
    //time_llt+=TIME(llt);
    //time_svd++;

    return gain_val; 
    
}

void SLAMEntropyDet::updateK(size_t key)
{
    Eigen::MatrixXd info = landmark_A[key].transpose()*landmark_A[key];
    size_t key_len = landmark_keys[key].size();

    for(size_t i=0;i<key_len;i++)
        for(size_t j=0;j<key_len;j++)
        {   
            size_t ki = landmark_keys[key][i]; 
            size_t kj = landmark_keys[key][j]; 
            Eigen::MatrixXd tmp = info.block<6,6>(i*6,j*6);
            K.block<6,6>(ki*6,kj*6) += tmp;
        }

}


void SLAMEntropyDet::update(size_t key)
{
    updateK(key);
    
    cov_computed = false;

    //time_llt= 0;
    //time_svd = 0;
    //time_inv = 0;


}


 Eigen::MatrixXd SLAMEntropyDet::getMarginalCov(std::vector<size_t> keys)
 {
     updateCov();
     return extract_block(keys,C);
 }





