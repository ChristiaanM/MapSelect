/*
 * File: Derivatives.h
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


#ifndef SLAM_DERIVATIVES_H
#define SLAM_DERIVATIVES_H

#include <cmath>
#include <Eigen/Core>
#include "mapselect/maps/MapDataAlias.h" // Matrix Definitions

namespace mselect
{
    /*! @file A collection of derivatives for use with SLAM without the dependencies of using the associated libraries.
    */

    bool A_stereo(const Mat34 &Tcw_est, const Mat31 &m_est, double fx, double fy, double bf, Mat33 &A_m, Mat36 &A_x)
    {
        auto R = Tcw_est.block<3, 3>(0, 0);
        auto t = Tcw_est.block<3, 1>(0, 3);
        Mat31 m_local = R * m_est + t;

        double x = m_local(0, 0);
        double y = m_local(1, 0);
        double z = m_local(2, 0);
        double z_2 = z * z;

        bool chirality_check = (z>0);
        if (!chirality_check)
        {
            A_m = Mat33::Zero();
            A_x = Mat36::Zero();
            return false;
        }

        A_m(0, 0) = -fx * R(0, 0) / z + fx * x * R(2, 0) / z_2;
        A_m(0, 1) = -fx * R(0, 1) / z + fx * x * R(2, 1) / z_2;
        A_m(0, 2) = -fx * R(0, 2) / z + fx * x * R(2, 2) / z_2;

        A_m(1, 0) = -fy * R(1, 0) / z + fy * y * R(2, 0) / z_2;
        A_m(1, 1) = -fy * R(1, 1) / z + fy * y * R(2, 1) / z_2;
        A_m(1, 2) = -fy * R(1, 2) / z + fy * y * R(2, 2) / z_2;

        A_m(2, 0) = A_m(0, 0) - bf * R(2, 0) / z_2;
        A_m(2, 1) = A_m(0, 1) - bf * R(2, 1) / z_2;
        A_m(2, 2) = A_m(0, 2) - bf * R(2, 2) / z_2;

        A_x(0, 0) = x * y / z_2 * fx;
        A_x(0, 1) = -(1 + (x * x / z_2)) * fx;
        A_x(0, 2) = y / z * fx;
        A_x(0, 3) = -1. / z * fx;
        A_x(0, 4) = 0;
        A_x(0, 5) = x / z_2 * fx;

        A_x(1, 0) = (1 + y * y / z_2) * fy;
        A_x(1, 1) = -x * y / z_2 * fy;
        A_x(1, 2) = -x / z * fy;
        A_x(1, 3) = 0;
        A_x(1, 4) = -1. / z * fy;
        A_x(1, 5) = y / z_2 * fy;

        A_x(2, 0) = A_x(0, 0) - bf * y / z_2;
        A_x(2, 1) = A_x(0, 1) + bf * x / z_2;
        A_x(2, 2) = A_x(0, 2);
        A_x(2, 3) = A_x(0, 3);
        A_x(2, 4) = 0;
        A_x(2, 5) = A_x(0, 5) - bf / z_2;

        return true;
    }

    bool A_stereo_local(const Mat34 &Tcw_est, const Mat31 &m_est, double fx, double fy, double bf, Mat36 &A_x)
    {
        auto R = Tcw_est.block<3, 3>(0, 0);
        auto t = Tcw_est.block<3, 1>(0, 3);
        Mat31 m_local = R * m_est + t;

        double x = m_local(0, 0);
        double y = m_local(1, 0);
        double z = m_local(2, 0);
        double z_2 = z * z;

        bool chirality_check = (z>0);
        if (!chirality_check)
        {
            A_x = Mat36::Zero();
            return false;
        }

        A_x(0, 0) = x * y / z_2 * fx;
        A_x(0, 1) = -(1 + (x * x / z_2)) * fx;
        A_x(0, 2) = y / z * fx;
        A_x(0, 3) = -1. / z * fx;
        A_x(0, 4) = 0;
        A_x(0, 5) = x / z_2 * fx;

        A_x(1, 0) = (1 + y * y / z_2) * fy;
        A_x(1, 1) = -x * y / z_2 * fy;
        A_x(1, 2) = -x / z * fy;
        A_x(1, 3) = 0;
        A_x(1, 4) = -1. / z * fy;
        A_x(1, 5) = y / z_2 * fy;

        A_x(2, 0) = A_x(0, 0) - bf * y / z_2;
        A_x(2, 1) = A_x(0, 1) + bf * x / z_2;
        A_x(2, 2) = A_x(0, 2);
        A_x(2, 3) = A_x(0, 3);
        A_x(2, 4) = 0;
        A_x(2, 5) = A_x(0, 5) - bf / z_2;

        return true;
    }

    bool A_mono(const Mat34 &Tcw_est, const Mat31 &m_est, double fx, double fy, Mat23 &A_m, Mat26 &A_x)
    {
        auto R = Tcw_est.block<3, 3>(0, 0);
        auto t = Tcw_est.block<3, 1>(0, 3);
        Mat31 m_local = R * m_est + t;

        double x = m_local(0, 0);
        double y = m_local(1, 0);
        double z = m_local(2, 0);

        double z_2 = z * z;

        bool chirality_check = (z>0);
        if (!chirality_check)
        {
            A_m = Mat23::Zero();
            A_x = Mat26::Zero();
            return false;
        }

        A_m(0, 0) = -fx * R(0, 0) / z + fx * x * R(2, 0) / z_2;
        A_m(0, 1) = -fx * R(0, 1) / z + fx * x * R(2, 1) / z_2;
        A_m(0, 2) = -fx * R(0, 2) / z + fx * x * R(2, 2) / z_2;

        A_m(1, 0) = -fy * R(1, 0) / z + fy * y * R(2, 0) / z_2;
        A_m(1, 1) = -fy * R(1, 1) / z + fy * y * R(2, 1) / z_2;
        A_m(1, 2) = -fy * R(1, 2) / z + fy * y * R(2, 2) / z_2;

        A_x(0, 0) = x * y / z_2 * fx;
        A_x(0, 1) = -(1 + (x * x / z_2)) * fx;
        A_x(0, 2) = y / z * fx;
        A_x(0, 3) = -1. / z * fx;
        A_x(0, 4) = 0;
        A_x(0, 5) = x / z_2 * fx;

        A_x(1, 0) = (1 + y * y / z_2) * fy;
        A_x(1, 1) = -x * y / z_2 * fy;
        A_x(1, 2) = -x / z * fy;
        A_x(1, 3) = 0;
        A_x(1, 4) = -1. / z * fy;
        A_x(1, 5) = y / z_2 * fy;

        return true;
    }

    bool A_mono_local(const Mat34 &Tcw_est, const Mat31 &m_est, double fx, double fy, Mat26 &A_x)
    {
        auto R = Tcw_est.block<3, 3>(0, 0);
        auto t = Tcw_est.block<3, 1>(0, 3);
        Mat31 m_local = R * m_est + t;

        double x = m_local(0, 0);
        double y = m_local(1, 0);
        double z = m_local(2, 0);

        double z_2 = z * z;

        bool chirality_check = (z>0);
        if (!chirality_check)
        {
            A_x = Mat26::Zero();
            return false;
        }


        A_x(0, 0) = x * y / z_2 * fx;
        A_x(0, 1) = -(1 + (x * x / z_2)) * fx;
        A_x(0, 2) = y / z * fx;
        A_x(0, 3) = -1. / z * fx;
        A_x(0, 4) = 0;
        A_x(0, 5) = x / z_2 * fx;

        A_x(1, 0) = (1 + y * y / z_2) * fy;
        A_x(1, 1) = -x * y / z_2 * fy;
        A_x(1, 2) = -x / z * fy;
        A_x(1, 3) = 0;
        A_x(1, 4) = -1. / z * fy;
        A_x(1, 5) = y / z_2 * fy;

        return true;
    }
}

#endif
