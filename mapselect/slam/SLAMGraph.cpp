
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

#include "mapselect/slam/SLAMGraph.h"
#include "mapselect/slam/Derivatives.h"
#include "mapselect/utils/Timing.h"

namespace mselect
{



    inline double SLAMGraph::inverseSigma(size_t oct) const
    {
        if (!gsettings.octave_scaling)
            return inv_sigma_by_octave[0];
        return inv_sigma_by_octave[oct];
    }


    bool SLAMGraph::weightedJacobianStereo(const AliasKeypoint &keypoint, const AliasFrame &frame, const AliasMappoint &mappoint, Mat33 &A_m, Mat36 &A_x) const
    {
        bool is_ok = A_stereo(frame.Tcw, mappoint.pos, frame.fx, frame.fy, frame.bf, A_m, A_x);
        A_m *= inverseSigma(keypoint.oct);
        A_x *= inverseSigma(keypoint.oct);
        return is_ok;
    }

    bool SLAMGraph::weightedJacobianMono(const AliasKeypoint &keypoint, const AliasFrame &frame, const AliasMappoint &mappoint, Mat23 &A_m, Mat26 &A_x) const
    {

        bool is_ok = A_mono(frame.Tcw, mappoint.pos, frame.fx, frame.fy, A_m, A_x);
        A_m *= inverseSigma(keypoint.oct);
        A_x *= inverseSigma(keypoint.oct);
        return is_ok;
    }

    bool SLAMGraph::weightedJacobianStereoLocal(const AliasKeypoint &keypoint, const AliasFrame &frame, const AliasMappoint &mappoint, Mat36 &A_x) const
    {
        bool is_ok = A_stereo_local(frame.Tcw, mappoint.pos, frame.fx, frame.fy, frame.bf, A_x);
        A_x *= inverseSigma(keypoint.oct);
        return is_ok;
    }

    bool SLAMGraph::weightedJacobianMonoLocal(const AliasKeypoint &keypoint, const AliasFrame &frame, const AliasMappoint &mappoint, Mat26 &A_x) const
    {
        bool is_ok = A_mono_local(frame.Tcw, mappoint.pos, frame.fx, frame.fy, A_x);
        A_x *= inverseSigma(keypoint.oct);
        return is_ok;
    }

    void SLAMGraph::infoLocal(const AliasKeypoint &keypoint, const AliasFrame &frame, const AliasMappoint &mappoint, Mat66 &K_x) const
    {

        if (keypoint.disp > 0)
        {
            Mat36 A;
            bool is_ok = weightedJacobianStereoLocal(keypoint, frame, mappoint, A);
            if (is_ok)
                K_x = A.transpose() * A;
            else
                K_x = Mat66::Zero();
        }
        else
        {
            Mat26 A;
            bool is_ok = weightedJacobianMonoLocal(keypoint, frame, mappoint, A);
            if (is_ok)
                K_x = A.transpose() * A;
            else
                K_x = Mat66::Zero();
        }
    }

    void SLAMGraph::infoOdometry(const AliasKeypoint &keypoint1, const AliasFrame &frame1, const AliasKeypoint &keypoint2, const AliasFrame &frame2, const AliasMappoint &mappoint, Mat66 &K_x) const
    {
        Mat33 A_m1, A_m2;
        Mat36 A_x1, A_x2;
        // Get the Jakobian matrices
        bool is_ok1 = weightedJacobianStereo(keypoint1, frame1, mappoint, A_m1, A_x1);
        bool is_ok2 = weightedJacobianStereo(keypoint2, frame2, mappoint, A_m2, A_x2);
        if (is_ok1 && is_ok2)
        {

            // Build necessary elements of the information matrix of the measurements of keypoint 1 & 2.
            // Since we immediately condition on the x1 states, we only need the rows & cols associated with x2 and m
            Mat33 K_m = A_m1.transpose() * A_m1 + A_m2.transpose() * A_m2;
            Mat66 K_x2 = A_x2.transpose() * A_x2;
            Mat36 K_mx2 = A_m2.transpose() * A_x2;

            // Since this is a 3x3 matrix, the eigen can use the closed form formula to evaluate the inverse quickly.
            Mat33 Cov_m = K_m.inverse();

            // Marginalise m to obtain the factor over x2
            K_x = K_x2 - K_mx2.transpose() * Cov_m * K_mx2;

            // double det = K_x.determinant();
            // Eigen::JacobiSVD<Mat66, Eigen::ComputeFullU | Eigen::ComputeFullV> svd(K_x);
            // std::cout << "s vals " << svd.singularValues() << std::endl;
            // std::cout << "det " << det << std::endl;
        }
        else
            K_x = Mat66::Zero();

        // std::cout << K_x << std::endl;
    }

    bool SLAMGraph::isPointOk(std::shared_ptr<MapDataAlias> map_data, size_t map_key) const
    {
        const AliasKeypointVector& keypoints =  map_data->map_keypoints[map_key];
        size_t cnt = keypoints.size();
        if (cnt >= 3)
            return true;
        for(size_t i=0;i<keypoints.size();i++)
        {
            if (keypoints[i].disp > 0)
                cnt++;
            if (cnt >= 3)
                return true;
        }
        return false;
    }

    std::vector<size_t> SLAMGraph::calcOdomParents(std::shared_ptr<MapDataAlias> map_data) const
    {
        std::vector<std::map<size_t, size_t>> covis = map_data->computeCovisCounts();
        std::vector<size_t> parents(map_data->n_frames, 0);
        for (size_t f = 0; f < map_data->n_frames; f++)
        {
            size_t parent = 0;
            size_t parent_weight = 0;

            std::map<size_t, size_t> &covis_map = covis[f];
            for (auto pair : covis_map)
            {
                if (pair.second > parent_weight && pair.first < f)
                {
                    parent = pair.first;
                    parent_weight = pair.second;
                }
            }
            parents[f] = parent;
        }
        return parents;
    }

    /*
    Search through the vector of observations from a map point to find the index
    of the observation that correspond to the parent frame. If the current observation
    is not a stereo observation, or the matching observation is not a stereo observation
    return the current index to signal no match was found.
    */
    size_t getParentObsIndex(const std::vector<size_t>& obs, const std::vector<AliasKeypoint>& keypoints, size_t parent_key, size_t index)
    {
        if (keypoints[index].disp <= 0)
            return index;

        for (size_t search_index = 0; search_index < index; search_index++)
        {
            if (obs[search_index] == parent_key) 
            {
                if  (keypoints[search_index].disp > 0)
                    return search_index;
                else 
                    return index;
            }
        }     
        return index;
    }
    /*
    PoseInfoMat SLAMGraph::odometryFactors(const std::vector<size_t>& obs,const std::vector<AliasKeypoint> &keypoints, const std::vector<size_t>& parents, size_t point_key)
    {
        std::vector<PoseInfoMat> landmark_factors;
        for (size_t i_idx = 0; i_idx < obs.size(); i_idx++)
        {
            // search for a valid parent
            size_t parent = parents[obs[i_idx]];
            size_t p_idx = 0;
            for (; p_idx < i_idx; p_idx++)
            {
                if (obs[p_idx] == parent)
                {
                    break;
                }
            }
            if (p_idx != i_idx && keypoints[i_idx].disp > 0 && keypoints[p_idx].disp > 0)
            {

                Mat66 info;
                size_t p_key = obs[p_idx];
                size_t i_key = obs[i_idx];
                infoOdometry(keypoints[p_idx], map_data->frames[p_key], keypoints[i_idx], map_data->frames[i_key], map_data->mappoints[point_key], info);
                landmark_factors[point_key].push_back(info);
            }
            else
            {
                landmark_factors[point_key].push_back(PoseInfoMat::Zero());
            }
        }
        return landmark_factors;

    }
*/

    PoseMatVector SLAMGraph::selectionLocalisationMatrices(std::shared_ptr<MapDataAlias> map_data, size_t point_key) const
    {
        PoseMatVector factors;
        const auto &obs = map_data->map_obs[point_key];
        const auto &keypoints = map_data->map_keypoints[point_key];

        if (!isPointOk(map_data, point_key))
            return factors;
        factors.resize(obs.size());
        for (size_t i = 0; i < obs.size(); i++)
        {
            size_t frame_key = obs[i];
            const AliasKeypoint &keypoint = keypoints[i];
            infoLocal(keypoint, map_data->frames[frame_key], map_data->mappoints[point_key], factors[i]);
        }
        return factors;
    }

    PoseInfoMat SLAMGraph::getApproxPrior() const
    {
        return  PoseInfoMat::Identity() * gsettings.approx_prior;
    }

    PoseMatVector SLAMGraph::selectionOdometryMatrices(std::shared_ptr<MapDataAlias> map_data, const std::vector<size_t> &parents, size_t point_key ) const
    {
        PoseMatVector factors;
        const auto &obs = map_data->map_obs[point_key];
        const auto &keypoints = map_data->map_keypoints[point_key];

        if (!isPointOk(map_data, point_key))
            return factors;

        factors.reserve(obs.size());
        for (size_t i_idx = 0; i_idx < obs.size(); i_idx++)
        {
            // search for a valid parent
            size_t parent = parents[obs[i_idx]];
            size_t p_idx = getParentObsIndex(obs,keypoints,parent,i_idx);
            if (p_idx != i_idx)
            {

                Mat66 info;
                size_t p_key = obs[p_idx];
                size_t i_key = obs[i_idx];

                infoOdometry(keypoints[p_idx], map_data->frames[p_key], keypoints[i_idx], map_data->frames[i_key], map_data->mappoints[point_key], info);
                factors.push_back(info);
            }
            else
            {
                factors.push_back(PoseInfoMat::Zero());
            }
        }
        return factors;
    }

    void SLAMGraph::buildSelectionOdometry(std::shared_ptr<MapDataAlias> map_data, PoseInfoMat &prior, std::vector<PoseMatVector> &landmark_factors) const
    {
        std::vector<size_t> parents = calcOdomParents(map_data);

        landmark_factors.resize(map_data->n_points);
        for (size_t point_key = 0; point_key < map_data->n_points; point_key++)
        {
            landmark_factors[point_key] = selectionOdometryMatrices(map_data,parents,point_key);
        }
        prior = getApproxPrior();
    }

    void SLAMGraph::buildSelectionLocalisation(std::shared_ptr<MapDataAlias> map_data, PoseInfoMat &prior, std::vector<PoseMatVector> &landmark_factors) const
    {
        PoseInfoMat zero = PoseInfoMat::Zero();
        landmark_factors.resize(map_data->n_points);
        for (size_t point_key = 0; point_key < map_data->n_points; point_key++)
        {
            landmark_factors[point_key] = selectionLocalisationMatrices(map_data,point_key);
        }
        prior = getApproxPrior();
    }

    MatXX SLAMGraph::calcSLAMInfoA(const MapDataAlias &map_data, size_t map_key, std::vector<size_t> &A_keys) const
    {
        typedef std::vector<Mat36, Eigen::aligned_allocator<Mat36>> Mat36Vector;
        typedef std::vector<Mat33, Eigen::aligned_allocator<Mat33>> Mat33Vector;

        AliasMappoint mappoint = map_data.mappoints[map_key];
        const std::vector<size_t> &map_obs = map_data.map_obs[map_key];
        const std::vector<AliasKeypoint> &map_keypoints = map_data.map_keypoints[map_key];

        /* Due to imperfect matching, we me might observe the same map point multiple times in the same frame.
        The current GTSAM implementation also just adds the multiple factors together, so this code will do the same.
        */

        // Find the unique keys
        A_keys = std::vector<size_t>(map_obs);
        std::sort(A_keys.begin(), A_keys.end());
        auto last = std::unique(A_keys.begin(), A_keys.end());
        A_keys.resize(std::distance(A_keys.begin(), last));

        // Populate obs_index with the indices of elements in the new array
        // we use linear search since it should be faster for the very small arrays we expect here.
        std::vector<size_t> obs_index;
        for (size_t i = 0; i < map_obs.size(); i++)
        {
            for (size_t j = 0; j < A_keys.size(); j++)
            {
                if (A_keys[j] == map_obs[i])
                    obs_index.push_back(j);
            }
        }

        size_t N = A_keys.size();
        size_t dim = 6 * N;

        Mat33 K_m = Mat33::Zero();
        Mat36Vector A_X;
        Mat33Vector A_M;

        A_X.resize(N, Mat36::Zero());
        A_M.resize(N, Mat33::Zero());

        // Rather than constructing the information matrix for all the observations of the map point, which is sparse, we store only the relevant information
        MatXX info = MatXX::Zero(dim, dim);

        for (size_t i = 0; i < map_obs.size(); i++)
        {

            size_t frame_key = map_obs[i];
            AliasFrame frame = map_data.frames[frame_key];
            const AliasKeypoint &keypoint = map_keypoints[i];
            size_t j = obs_index[i];

            if (keypoint.disp > 0)
            {
                Mat33 tmp_m;
                Mat36 tmp_x;
                weightedJacobianStereo(keypoint, frame, mappoint, tmp_m, tmp_x);
                A_M[j] += tmp_m;
                A_X[j] += tmp_x;
            }
            else
            {
                Mat23 tmp_m;
                Mat26 tmp_x;
                weightedJacobianMono(keypoint, frame, mappoint, tmp_m, tmp_x);
                A_M[j].topRows<2>() += tmp_m;
                A_X[j].topRows<2>() += tmp_x;
            }
            K_m += A_M[j].transpose() * A_M[j];
        }

        Mat33 Cov_m;
        double det;
        bool inv_ok;
        K_m.computeInverseAndDetWithCheck(Cov_m, det, inv_ok);

        // Now directly calculate the information matrix after marginalising the elements that correspond with the mappoint states.
        if (inv_ok)
        {
            for (size_t r = 0; r < N; r++)
                for (size_t c = 0; c < N; c++)
                {
                    if (r == c)
                    {
                        info.block<6, 6>(r * 6, r * 6) += A_X[r].transpose() * A_X[r];
                    }
                    Mat63 K_xm = A_X[r].transpose() * A_M[r];
                    Mat36 K_mx = A_M[c].transpose() * A_X[c];
                    info.block<6, 6>(r * 6, c * 6) -= K_xm * Cov_m * K_mx;
                }

            /*
            MatXX info2(dim + 3, dim + 3);
            info2 = MatXX::Zero(dim + 3, dim + 3);
            size_t li = dim;
            size_t lii = dim + 3;

            std::cout << lii << std::endl;

            for (size_t j = 0; j < N; j++)
            {
                std::cout << j << ":" << N << std::endl;
                std::cout << "A_X" << std::endl << A_X[j] << std::endl;
                std::cout << "A_M" << std::endl << A_M[j] << std::endl;
                info2.block<6, 6>(j * 6, j * 6) += A_X[j].transpose() * A_X[j];
                info2.block<3, 3>(li, li) += A_M[j].transpose() * A_M[j];
                info2.block<3, 6>(li, j * 6) += A_M[j].transpose() * A_X[j];
                info2.block<6, 3>(j * 6, li) += A_X[j].transpose() * A_M[j];
            }

            std::cout << "info2" << std::endl <<  info2 << std::endl;

            Eigen::Matrix<double, -1, -1> A = info2.topLeftCorner(li, li);
            std::cout << "A" << std::endl << A << std::endl;
            Eigen::Matrix<double, -1, 3>  B = info2.topRightCorner(li,3);
            std::cout << "B" << std::endl << B << std::endl;


            Eigen::Matrix<double, 3, 3>   C = info2.block<3,3>(li, li);
            std::cout << "C" << std::endl << C << std::endl;
            Eigen::Matrix<double, 3, -1>  D = info2.bottomLeftCorner(3,li);
            std::cout << "D" << std::endl << D << std::endl;

            std::cout << A.rows() << std::endl;
            std::cout << B.rows() << std::endl;
            std::cout << D.cols() << std::endl;




            MatXX info3 = A - B * C.inverse() * D;

            std::cout << "Info3" << std::endl
                      << info3 << std::endl;
            */

            // Our current implementation requires the jakobian of the information matrix, that is K = (A.T)(A)
            // We use the low-rank eigenvalue decomposition to get a suitable matrix A

            Eigen::SelfAdjointEigenSolver<MatXX> ev(info);
            Eigen::VectorXd eigenvalues = ev.eigenvalues();
            MatXX eigenvectors = ev.eigenvectors();

            // std::cout << eigenvalues << std::endl;
            //  Get the Rank
            double tolerance = 1e-9;
            size_t rank = (eigenvalues.array() > tolerance).count();
            // A_keys = std::vector<size_t>(map_obs);
            // Eigen orders values from low to high - so we need the last values & right columns for a low rank approx
            MatXX lowRank = (eigenvectors.rightCols(rank) * eigenvalues.tail(rank).cwiseSqrt().asDiagonal()).transpose();
            // std::cout << lowRank << std::endl;

            assert(lowRank.cols() == dim && "Wrong Dimentions in A-approximation");

            return lowRank;
        }

        A_keys = std::vector<size_t>();
        return MatXX();
    }

    void SLAMGraph::buildSelectionSLAM(const MapDataAlias &map_data, std::vector<MatXX> &landmark_A, std::vector<std::vector<size_t>> &landmark_keys) const
    {
        TIC(slamgraph);
        landmark_A.reserve(map_data.n_points);
        landmark_keys.resize(map_data.n_points);
        for (size_t i = 0; i < map_data.n_points; i++)
        {
            landmark_A.push_back(calcSLAMInfoA(map_data, i, landmark_keys[i]));
        }
        TOC(slamgraph);
        TIME_PRINT(slamgraph);
    }
}
