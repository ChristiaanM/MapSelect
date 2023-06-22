/*
 * File: OSMapData.h
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



#ifndef MAPSELECT_OSMAP_DATA_H
#define MAPSELECT_OSMAP_DATA_H

#include "mapselect/maps/MapData.h"
#include "mapselect/maps/Selection.h"
#include "thirdparty/osmap/osmap.pb.h"

#include <unordered_map>
#include <vector>
#include <memory> // shared_ptr
#include <bitset>

namespace mselect
{

    /*! @brief  responsible for operating on the serialised OSMAP files.
    */
    class OSMapData : public MapData
    {
    public:
        friend class OSMapDataAlias;

        typedef std::shared_ptr<OSMapData> shared_ptr;

        OSMapData(std::string input_yaml);

        void select(const Selector &select);

        void saveMap(std::string yaml_path);

        size_t getMapSize();

    protected:
        std::vector<size_t> mappoint_ids;
        std::vector<size_t> keyframe_ids; 
    
        //std::vector<OSMapPoint> mappoints;
        //std::vector<OSMapFrame> frames;

    private:
        void parseMappoints();
        void parseKeyframes();
        void parseFeatures();

        bool saveMappoints(std::string output_path);
        bool saveKeyframes(std::string output_path);
        bool saveFeatures(std::string output_path);
        bool saveTracking(std::string ouput_path);

        void selectMappoints(const Selector &selector);
        void selectKeyframes(const Selector &selector);
        void selectFeatures(const Selector &selector);

        int nMappoints, nKeyframes, nFeatures, nTackingFrames;
        std::string mappointsFile, keyframesFile, featuresFile, trackingFile;

        SerializedMappointArray protoc_mappointsArray;
        SerializedKeyframeArray protoc_keyframesArray;
        SerializedKeyframeFeaturesArray protoc_featuresArray;
        SerializedTrackingFramesArray protoc_trackingArray;

        std::bitset<32> options;
        std::vector<std::string> options_str;
        std::vector<double> cx, cy, fx, fy, baseline;
    };
}

#endif // MAP_DATA_H