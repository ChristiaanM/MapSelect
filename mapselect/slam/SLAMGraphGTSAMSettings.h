/*
 * File: SLAMGraphGTSAMSettings.h
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



#ifndef MAPSELECT_GRAPHSETTINGS_H
#define MAPSELECT_GRAPHSETTINGS_H

namespace mselect
{

    const double POSE_PRIOR = 1e-4;
    const double ORIGIN_PRIOR = 1e6;

    /*! @brief Struct containing different settings for the SLAM graph built for various implementations
    of the SLAM utility function.  See SLAMGraphSLAM for more information. 

    This struct is included in an separate header file to allow these settings to be passed as a paramater,
    without requiring inclusions of relevant GTSAM headers. 
    */
    struct GTSAMGraphSettings
    {
        bool robust = false;
        bool stereo_only = false;
        bool remove_outliers = false;
        bool smart = false;
        bool remove_limited_obs = true;
        bool octave_scaling = false;
    };

}
#endif