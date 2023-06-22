/*
 * File: SLAMInfoDense.cpp
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


#include "mapselect/functions/SLAMInfoDense.h"
#include "mapselect/slam/SLAMGraphGTSAM.h"

#include "mapselect/utils/Timing.h"
#include "mapselect/utils/MatFuncs.h"

#include <gtsam/inference/Symbol.h>
#include <Eigen/SparseCore>

namespace mselect
{

    /*
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


    gtsam::Matrix extract_cov_row(const gtsam::KeyVector &kv, const gtsam::Matrix &mat)
    {
        size_t D = kv.size() * 6;
        size_t N = mat.cols();
        gtsam::Matrix c(D, N);

        for (size_t i = 0; i < kv.size(); i++)
        {

            gtsam::Key ki = kv[i];
            size_t ci = i * 6;
            size_t Ci = gtsam::Symbol(ki).index() * 6;
            for (size_t ii = 0; ii < 6; ii++)
            {

                c.row(ci + ii) = mat.row(Ci + ii);
            }
        }
        return c;
    }
    */
    void SLAMInfoDet::print() const
    {
    }

    MatXX extract_block(const std::vector<size_t> &kv, const MatXX &mat)
    {
        size_t D = kv.size() * 6;
        MatXX c(D, D);
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

    SLAMInfoDet::SLAMInfoDet(std::shared_ptr<MapDataAlias> map_data_ptr, GTSAMGraphSettings graph_settings)
    {

        SLAMGraphGTSAM graph(map_data_ptr, graph_settings);
        graph.buildSelectionSLAMMatrices(landmark_A,landmark_keys);
        dim = map_data_ptr->n_frames * 6;
        n_frames = map_data_ptr->n_frames;
    }
    
    SLAMInfoDet::SLAMInfoDet(std::shared_ptr<MapDataAlias> map_data, const SLAMGraph& graph)
    {
        graph.buildSelectionSLAM(*map_data,landmark_A,landmark_keys);
        dim = map_data->n_frames*6;
        n_frames = map_data->n_frames;
    }
    

    

    void SLAMInfoDet::resetK()
    {
        K = MatXX::Zero(dim, dim);
        K_allocated = true;
        for (size_t i = 0; i < 6; i++)
            K(i, i) = origin_info;
        for (size_t i = 6; i < dim; i++)
            K(i, i) = prior_info;
        cov_computed = false;
    }

    double SLAMInfoDet::eval() const
    {
        Eigen::LDLT<MatXX> ldlt(K);
        auto diag = ldlt.vectorD();
        double logsum = 0;
        for (size_t i = 0; i < diag.rows(); i++)
            logsum += log(diag(i));

        return NegEntropyGaussian(logsum, dim);
    }

    void SLAMInfoDet::reset()
    {
        resetK();
    }

    void SLAMInfoDet::updateCov()
    {
        if (!cov_computed)
        {
            //TIC(inverse);
            C = K.inverse();
            //TOC(inverse);
            //TIME_PRINT(inverse);

            //TIC(inverse2);
            //C = K.selfadjointView<Eigen::Lower>().llt().solve(MatXX::Identity(K.rows(),K.cols()));
            //TOC(inverse2);
            //TIME_PRINT(inverse2);
            cov_computed = true;
        }
    }

    double SLAMInfoDet::gain(size_t key)
    {
        if (!K_allocated)
            resetK();

        updateCov();

        const auto &A = landmark_A[key];
        const auto &kv = landmark_keys[key];
        if (A.rows() == 0)
            return 0.0;

        MatXX c = extract_block(kv, C);
        MatXX mat = (MatXX::Identity(A.rows(), A.rows()) + A * c * A.transpose());

        // calculate the log-determinant
        Eigen::LLT<mselect::MatXX> llt = mat.selfadjointView<Eigen::Lower>().llt();
        const Eigen::TriangularView<const MatXX, 1U> L = llt.matrixL();
        double log_sum=0;
        for(size_t i=0;i<6;i++)
            log_sum+=log(L.coeff(i,i));

        return log_sum; // x 2.0 for logdet x 0.5 for entropy = 1.0

    }

    void SLAMInfoDet::allocateKsparse()
    {

        typedef Eigen::Triplet<double> T;
        std::vector<T> triplets;

        std::vector<std::set<size_t>> frame_covisibility(n_frames);
        for(size_t l=0;l<landmark_keys.size();l++)
        {
            std::vector<size_t>& obs = landmark_keys[l]; 
            for(size_t frame1 : obs)
            {
                for(size_t frame2 : obs)
                {
                    frame_covisibility[frame1].insert(frame2);
                    frame_covisibility[frame2].insert(frame1);
                }
            }

        }
        
        
        for(size_t i=0;i<frame_covisibility.size();i++)
        {
            size_t ki = i;
            for(size_t obs : frame_covisibility[i])
            {
                // add a 6x6 block of zeros.
                size_t kj = obs;
                for (size_t ii = 0; ii < 6; ii++)
                    for (size_t jj = 0; jj < 6; jj++)
                        triplets.emplace_back(ki * 6 + ii, kj * 6 + jj, 0);
            }
        }


        // populate the diagonal.
        for (size_t i = 0; i < 6; i++)
            triplets.emplace_back(i, i, origin_info);
        for (size_t i = 6; i < dim; i++)
            triplets.emplace_back(i, i, prior_info);
        
        Ksparse = CSCMat(dim,dim);
        Ksparse.setFromTriplets(triplets.begin(), triplets.end());
        Ksparse.makeCompressed();
    


        
    }



    void SLAMInfoDet::updateK(size_t key)
    {
        if (!K_allocated)
            resetK();

        MatXX A = landmark_A[key];
        size_t key_len = landmark_keys[key].size();
        size_t rows = A.rows();

        for (size_t i = 0; i < key_len; i++)
            for (size_t j = 0; j < key_len; j++)
            {
                    MatXX Ai = A.block(0,i*6,rows,6);
                    MatXX Aj = A.block(0,j*6,rows,6);
                    Mat66 delta = Ai.transpose()*Aj;

                    size_t ki = landmark_keys[key][i];
                    size_t kj = landmark_keys[key][j];

                    K.block<6,6>(ki*6,kj*6) +=delta;
            }
        /*
        MatXX info = landmark_A[key].transpose() * landmark_A[key];
        size_t key_len = landmark_keys[key].size();
        for (size_t i = 0; i < key_len; i++)
        {
            size_t ki = landmark_keys[key][i];
            auto Ai = landmark_A[key].block<6, 6>(0, i * 6);
            for (size_t j = 0; j < key_len; j++)
            {
                size_t kj = landmark_keys[key][j];
                auto Aj = landmark_A[key].block<6, 6>(0, j * 6);
                K.block<6, 6>(ki * 6, kj * 6) += Ai.transpose() * Aj;
            }
        }
        */
    }

    void SLAMInfoDet::update(size_t key)
    {
        updateK(key);

        if (selected.size() % 100==0)
            std::cout << "Selected" << selected.size() << std::endl;
    
        cov_computed = false;
    }

    MatXX SLAMInfoDet::getMarginalCov(std::vector<size_t> keys)
    {
        updateCov();
        return extract_block(keys, C);
    }

}