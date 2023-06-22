/*
 * File: SLAMGraph.h
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


#ifndef INFO_GRAPH_H
#define INFO_GRAPH_H

#include "mapselect/maps/MapDataAlias.h"

#include <Eigen/Eigenvalues>

namespace mselect
{

    struct SLAMGraphSettings
    {  
        bool octave_scaling = false; 
        size_t octaves = 8;
        double sigma_obs = 1;
        double scale_fact = 1.2;
        double approx_prior = 1e-6;
    };


    /*! @brief Class responsible for calculating the information matrices associated with various map point selection approaches.

    @todo Add support for a wider range of graph settings. 
    */
    class SLAMGraph
    {

    public:
        SLAMGraph(SLAMGraphSettings settings = SLAMGraphSettings()) : gsettings(settings)
        {

            inv_sigma_by_octave.resize(settings.octaves);
            double current_sigma = settings.sigma_obs;
            for(size_t i=0;i < settings.octaves;i++)
            {
                inv_sigma_by_octave[i] = 1.0/current_sigma;
                current_sigma*=settings.scale_fact;
            }
        }

        /*! Use the localisation approximation to formulate the map point selection problem. 
        @param map_data Alias of the current map data
        @param prior Output information matrix to store the prior information matrix associated with each pose 
        @param landmark_factors Output the information matrices associated with each map point observation
        */
        void buildSelectionLocalisation(std::shared_ptr<MapDataAlias> map_data, PoseInfoMat &prior, std::vector<PoseMatVector> &landmark_factors) const;

        /*! Return the information matrices associated with a specific map point using the localisation approximation
        @param map_data Alias of the current map data
        @param parents The parents of each pose needed for the odometry approximation. 
        @param point_key The map point for which the calculate the matrices
        */
        PoseMatVector selectionLocalisationMatrices(std::shared_ptr<MapDataAlias> map_data, size_t map_key) const;

        /*! Return the information matrices associated with a specific map point using the odometry approximation
        @param map_data Alias of the current map data
        @param parents The parents of each pose needed for the odometry approximation. 
        @param point_key The map point for which the calculate the matrices
        */
        PoseMatVector selectionOdometryMatrices(std::shared_ptr<MapDataAlias> map_data, const std::vector<size_t> &parents, size_t point_key ) const;

        /*! Use the odometry approximate to formulate the map point selection problem. 
        @param map_data Alias of the current map data
        @param prior Output information matrix to store the prior information matrix associated with each pose 
        @param landmark_factors Output the information matrices associated with each map point observation
        */
        void buildSelectionOdometry(std::shared_ptr<MapDataAlias> map_data, PoseInfoMat &prior, std::vector<PoseMatVector> &landmark_factors) const;

        /*!
        Unsupported Reimplementation of the SLAM approach
        */
        void buildSelectionSLAM(const MapDataAlias &map_data, std::vector<MatXX> &landmark_A, std::vector<std::vector<size_t>> &landmark_keys) const;

        /*!
        Get the approximate prior - this avoids hardcoding the matrices
        */
        PoseInfoMat getApproxPrior() const;


        /*!
        Calculate the parents for the odometry approximation for a given map_data
        @param map_data alias of map data for which to perform the calculation
        */
        std::vector<size_t> calcOdomParents(std::shared_ptr<MapDataAlias> map_data) const;

        /*!
        A common heuristic to decide whether to include a map point in the estimation.
        @param map_data alias of map data for which to perform the calculation
        @param map_key key for the map point
        */
        bool isPointOk(std::shared_ptr<MapDataAlias> map_data, size_t map_key) const;

    private:
        inline double inverseSigma(size_t oct) const;

        bool weightedJacobianStereo(const AliasKeypoint &keypoint, const AliasFrame &frame, const AliasMappoint &mappoint, Mat33 &A_m, Mat36 &A_x) const;
        bool weightedJacobianMono(const AliasKeypoint &keypoint, const AliasFrame &frame, const AliasMappoint &mappoint, Mat23 &A_m, Mat26 &A_x) const;
        bool weightedJacobianStereoLocal(const AliasKeypoint &keypoint, const AliasFrame &frame, const AliasMappoint &mappoint, Mat36 &A_x) const;
        bool weightedJacobianMonoLocal(const AliasKeypoint &keypoint, const AliasFrame &frame, const AliasMappoint &mappoint, Mat26 &A_x) const;
        void infoLocal(const AliasKeypoint &keypoint, const AliasFrame &frame, const AliasMappoint &mappoint, Mat66 &K_x) const;
        void infoOdometry(const AliasKeypoint &keypoint1, const AliasFrame &frame1, const AliasKeypoint &keypoint2, const AliasFrame &frame2, const AliasMappoint &mappoint, Mat66 &K_x) const;
        
        MatXX calcSLAMInfoA(const MapDataAlias &map_data, size_t map_key, std::vector<size_t> &A_keys) const;

    private:
        std::vector<double> inv_sigma_by_octave;
        
        //double sigma_obs;
        //double scale_fact;
        //double inv_sigma_obs;
        //double inv_scale_fact;
        SLAMGraphSettings gsettings;

        //double approx_prior;
    };

}

#endif