/*
 * File: demo.cpp
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

#include "mapselect/utils/Timing.h"
#include "mapselect/maps/OSMapAlias.h"

//
#include "mapselect/functions/FrameOdometry.h"
#include "mapselect/functions/FrameLocal.h"
#include "mapselect/algorithms/LazyGreedy.h"
//#include "mapselect/unsupported/functions/FrameEstimationLegacy.h"
#ifdef CHOLMOD_FEATURES
#include "mapselect/functions/SLAMInfoCholmod.h"
#endif

using namespace mselect;

/*! @file This binary provides a basic example of using the library.
    For a more detailed commandline binary, see ./mselect.cpp
*/
int main(int argc, char const *argv[])
{

    if (argc != 2)
    {
        std::cout << "Usage : ./demo $path_to_osmap_yaml" << std::endl;
        return 1;
    }

    /*
     A design choice made by MapSelect is to assume that the data
     describing the SLAM problem for selection is packed into a specific
     internal class a MapDataAlias. This assumes that all map points have
     keys (going from 0 ~ N), and all frames have keys going from 0~T. It
     further assumes map point observations are sorted. This simplifies
     the design of selection algorithms. To enable use of these algorithms
     on practical data, the internal IDs associated with each map point,
     and their observations must first be converted to keys. See 
     MapDataAlias.h for more detail.
    */

    // First, setup the data, here we load it from a file passed through commandline
    OSMapData raw_data(argv[1]);

    // Then build the MapData alias from the raw data
    // we make a shared pointer since this is used by functions
    auto map_data_ptr = std::make_shared<OSMapDataAlias>(raw_data);

    /*
    This alias is passed as a paramater to any of the provided functions.
    Below we discuss some potential choices of functions. For additional
    functions, see the mapselect/functions/ directory
    */
    //std::shared_ptr<IncrementalSetFunction<double>> func_ptr;

    TIC(timing); // time the duration selection takes

#ifdef CHOLMOD_FEATURES
// For the SLAM implementation, you need to link to Cholmod,
// and we provide only one implementation.
// SLAMInfoCholmod (map_data_ptr);
#endif

    /*
    For the odometry and localisation approximations, we provide a few different
    implementations. It is recommended that you either use the "Fast" or "Memory"
    implementations. These implementations return identical results, but have 
    different software implementations. 
    */
    //

    /* Fastest implementations (but they require more memory)*/
    FrameOdometryFast odometry_fast(map_data_ptr);
    //FrameLocalFast local_fast(map_data_ptr);
    /*  Slightly slower, more memory efficient implementations. */
    //FrameOdometryMemory odometry_memory(map_data_ptr);
    //FrameLocalMemory local_memory(map_data_ptr);

    // Number of map points to select
    IncrementalSetFunction<double>& func_ref = odometry_fast;
    // If you uncomment a different function ^, just change this reference.
    
    size_t N = 20000;

    // For more functions - see mapselect/functions

    // First, we need to generate all the keys to select.
    std::vector<size_t> eval_keys, heuristic_keys;
    map_data_ptr->lastFramesHeuristic(1, eval_keys, heuristic_keys);
    func_ref.addBatch(heuristic_keys);
    std::cout << "heuristic keys" << heuristic_keys.size() << std::endl;

    // Run the lazy greedy algorithm, selecting keys from the set "eval_keys", until a maximum of N points are in the set

    LazyGreedy<>(func_ref, eval_keys, N);
    TOC(timing);

    // These functions store the currently selected set of map points as part of the function object.
    std::vector<size_t> selected_keys = func_ref.getSelected();

    // In practice, we are often interested in the selected IDs - not the internal keys that this library uses
    // You can convert keys back to their IDs as follows
    std::vector<size_t> selected_ids = map_data_ptr->convertMappointKeysToIDs(selected_keys);

    // NOTE this timing will be wrong uncomment more than one function above
    std::cout << "Selected " << selected_ids.size() << " of " << map_data_ptr->getMapSize() << "map points in " << TIME_SECONDS(timing) << " seconds" << std::endl;

    // See ./mselect.cpp for a binary that allows choosing functions and algorithms 
}