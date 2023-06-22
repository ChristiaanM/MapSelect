/*
 * File: OSMapData.cpp
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


#include "mapselect/maps/OSMapData.h"
#include "mapselect/utils/PathUtils.h"

#include <algorithm>
#include <assert.h>
#include <bitset>
#include <fstream>
#include <iostream>
#include <string>

#include <google/protobuf/util/delimited_message_util.h>
#include <opencv2/core/core.hpp> // yaml

using namespace std;

namespace mselect
{

    // Map options as defined by osmap
    enum Options
    {
        FEATURES_FILE_DELIMITED = 0,
        FEATURES_FILE_NOT_DELIMITED = 1
    };

    size_t OSMapData::getMapSize()
    {
        return nMappoints;
    }

    void OSMapData::parseMappoints()
    {
        mappoint_ids.reserve(getMapSize());
        for(size_t i = 0; i < protoc_mappointsArray.mappoint_size(); i++)
        {
            const SerializedMappoint &mappoint = protoc_mappointsArray.mappoint(i);
            mappoint_ids.push_back(mappoint.id());
        }
        assert(mappoint_ids.size() == nMappoints);
        /*
        
        mappoints.clear();
        mappoints.reserve(nMappoints);

        for (size_t i = 0; i < protoc_mappointsArray.mappoint_size(); i++)
        {
            const SerializedMappoint &mappoint = protoc_mappointsArray.mappoint(i);
            const SerializedPosition &position = mappoint.position();

            OSMapPoint point;
            point.map_id = mappoint.id();
            point.x = position.x();
            point.y = position.y();
            point.z = position.z();
            point.vis = mappoint.visible();
            point.found = mappoint.found();
            mappoints.push_back(std::move(point));
        }
        */
    }

    void OSMapData::parseKeyframes()
    {

        for (size_t i = 0; i < protoc_keyframesArray.keyframe_size(); i++)
        {
            const SerializedKeyframe &keyframe = protoc_keyframesArray.keyframe(i);
            keyframe_ids.push_back(keyframe.id());
        }
        assert(keyframe_ids.size() == nKeyframes);
        
        /*
        frames.clear();
        for (size_t i = 0; i < protoc_keyframesArray.keyframe_size(); i++)
        {
            const SerializedKeyframe &keyframe = protoc_keyframesArray.keyframe(i);
            OSMapFrame frame;
            frame.frame_id = keyframe.id();
            frames.push_back(std::move(frame));
        }
        */
    }

    void OSMapData::parseFeatures()
    {
        assert(protoc_featuresArray.feature_size() == nKeyframes);
        /*
        
        size_t N = protoc_featuresArray.feature_size();

        // Read and build KF obs
        for (size_t i = 0; i < N; i++)
        {
            const SerializedKeyframeFeatures keyframe = protoc_featuresArray.feature(i);
            size_t frame_id = keyframe.keyframe_id();
            assert(frames[i].frame_id == frame_id);

            for (size_t k = 0; k < keyframe.feature_size(); k++)
            {
                const SerializedFeature feature = keyframe.feature(k);
                const SerializedKeypoint keypoint = feature.keypoint();

                size_t map_id = feature.mappoint_id();
                if (map_id)
                {
                    OSMapStereoKeypoint kp;
                    kp.px = keypoint.ptx();
                    kp.py = keypoint.pty();
                    if (feature.uright())
                        kp.disp = keypoint.ptx() - feature.uright();
                    else
                        kp.disp = -1.0;
                    kp.oct = keypoint.octave();

                    frames[i].obs.push_back(map_id);
                    frames[i].keypoints.push_back(std::move(kp));
                }
            }
        }
        */        
    }

    OSMapData::OSMapData(std::string input_yaml)
    {

        cv::FileStorage yaml_data = cv::FileStorage(input_yaml, cv::FileStorage::READ);

        yaml_data["nMappoints"] >> nMappoints;
        yaml_data["nKeyframes"] >> nKeyframes;
        yaml_data["nFeatures"] >> nFeatures;
        yaml_data["nFrames"] >> nTackingFrames;

        yaml_data["mappointsFile"] >> mappointsFile;
        yaml_data["keyframesFile"] >> keyframesFile;
        yaml_data["featuresFile"] >> featuresFile;
        yaml_data["trackingFile"] >> trackingFile;

        {
            auto node = yaml_data["cameraMatrices"];
            for (auto iter = node.begin(); iter != node.end(); ++iter)
            {
                fx.push_back((double)(*iter)["fx"]);
                fy.push_back((double)(*iter)["fy"]);
                cx.push_back((double)(*iter)["cx"]);
                cy.push_back((double)(*iter)["cy"]);
                baseline.push_back((double)(*iter)["baseline"]);
                // std::cout << "cx" << (double)(*iter)["cy"] << std::endl;
                // std::cout << "BASELINE" << (double)(*iter)["baseline"] << std::endl;
            }
        }

        {
            auto node = yaml_data["Options descriptions"];
            for (auto iter = node.begin(); iter != node.end(); ++iter)
            {
                if ((*iter).string().size())
                    options_str.push_back((*iter).string());
            }
        }

        {
            int tmp_int = 0;
            yaml_data["Options"] >> tmp_int;
            options = tmp_int;
        }

        // mappointsFile, keyframesFile, featuresFile, trackingFile;

        string map_directory = strpath_directory(input_yaml);
        string mappoints_path = strpath_join(map_directory, mappointsFile);
        string keyframes_path = strpath_join(map_directory, keyframesFile);
        string features_path = strpath_join(map_directory, featuresFile);
        string tracking_path = strpath_join(map_directory, trackingFile);

        {
            fstream input(mappoints_path, ios::in | ios::binary);
            if (!protoc_mappointsArray.ParseFromIstream(&input))
            {
                cerr << "Failed to parse mappoints file: " << mappointsFile << endl;
            }
        }

        {
            fstream input(keyframes_path, ios::in | ios::binary);

            if (!protoc_keyframesArray.ParseFromIstream(&input))
            {
                cerr << "Failed to parse keyframes file: " << keyframesFile << endl;
            }
        }

        {

            fstream input(features_path, ios::in | ios::binary);
            if (options[1])
                if (!protoc_featuresArray.ParseFromIstream(&input))
                {
                    cerr << "Failed to parse features file: " << featuresFile << endl;
                }

            if (options[0])
            {
                auto raw_input = new ::google::protobuf::io::IstreamInputStream(&input);
                auto coded_input = new google::protobuf::io::CodedInputStream(raw_input);

                bool eof;
                while (google::protobuf::util::ParseDelimitedFromCodedStream(&protoc_featuresArray, coded_input, &eof))
                {
                }

                if (!eof)
                {
                    cerr << "Failed to parse features file: " << featuresFile;
                }

                delete coded_input;
                delete raw_input;
            }
        }

        {

            fstream input(tracking_path, ios::in | ios::binary);
            if (!protoc_trackingArray.ParseFromIstream(&input))
            {
                cerr << "Failed to parse tracking file: " << trackingFile;
            }
        }

        parseKeyframes();
        parseMappoints();
        parseFeatures();
    }

    bool OSMapData::saveMappoints(string output_path)
    {
        fstream output(output_path, ios::out | ios::binary);
        return protoc_mappointsArray.SerializeToOstream(&output);
    }

    bool OSMapData::saveKeyframes(string output_path)
    {
        fstream output(output_path, ios::out | ios::binary);
        return protoc_keyframesArray.SerializeToOstream(&output);
    }

    bool OSMapData::saveFeatures(string output_path)
    {
        fstream output(output_path, ios::out | ios::binary);
        if (options[1])
            return protoc_featuresArray.SerializeToOstream(&output);

        if (options[0])
        {
            size_t i = 0, N = protoc_featuresArray.feature_size();
            bool ok = true;
            while (i < N && ok)
            {
                size_t fsum = 0;
                size_t LIMIT = 1000000;

                auto message = SerializedKeyframeFeaturesArray();

                for (; i < N && fsum < LIMIT; i++)
                {
                    const SerializedKeyframeFeatures &ref_data = protoc_featuresArray.feature(i);
                    fsum += ref_data.feature_size();
                    message.add_feature()->MergeFrom(ref_data);
                }
                ok = google::protobuf::util::SerializeDelimitedToOstream(message, &output);
            }
            return ok;
        }
        return false;
    }

    bool OSMapData::saveTracking(string ouput_path)
    {
        fstream output(ouput_path, ios::out | ios::binary);
        return protoc_trackingArray.SerializeToOstream(&output);
    }

    void OSMapData::selectMappoints(const Selector &selector)
    {
        SerializedMappointArray selected_mappoints;
        size_t cnt = 0;

        for (size_t i = 0, N = protoc_mappointsArray.mappoint_size(); i < N; i++)
        {
            const SerializedMappoint &ref_data = protoc_mappointsArray.mappoint(i);
            if (selector.select_mappoint(ref_data.id()))
            {
                auto new_point = selected_mappoints.add_mappoint();
                new_point->CopyFrom(ref_data);
                cnt++;
            }
        }

        nMappoints = cnt;

        protoc_mappointsArray = std::move(selected_mappoints);
    }

    void OSMapData::selectKeyframes(const Selector &selector)
    {
        SerializedKeyframeArray selected_keyframes;
        size_t cnt = 0;
        for (size_t i = 0, N = protoc_keyframesArray.keyframe_size(); i < N; i++)
        {
            const SerializedKeyframe &ref_data = protoc_keyframesArray.keyframe(i);
            if (selector.select_keyframe(ref_data.id()))
            {
                auto new_keyframe = selected_keyframes.add_keyframe();
                new_keyframe->CopyFrom(ref_data);
                cnt++;
            }
        }

        nKeyframes = cnt;
        protoc_keyframesArray = std::move(selected_keyframes);
    }

    void OSMapData::selectFeatures(const Selector &selector)
    {
        SerializedKeyframeFeaturesArray selected_features;
        size_t cnt = 0;
        bool flag = false;

        for (size_t i = 0, N = protoc_featuresArray.feature_size(); i < N; i++)
        {
            const SerializedKeyframeFeatures &ref_keyframe = protoc_featuresArray.feature(i);
            size_t kf_id = ref_keyframe.keyframe_id();
            if (selector.select_keyframe(kf_id))
            {
                auto new_keyframe = selected_features.add_feature(); // very poorly named variables in protoc file - add feature is intended
                new_keyframe->set_keyframe_id(kf_id);

                for (size_t j = 0, K = ref_keyframe.feature_size(); j < K; j++)
                {
                    const SerializedFeature &ref_feature = ref_keyframe.feature(j);
                    size_t map_id = ref_feature.mappoint_id();
                    bool select_mappoint = selector.select_mappoint(map_id);
                    bool select_feature = selector.select_feature(kf_id, map_id);

                    if (select_feature || select_mappoint)
                    {
                        SerializedFeature *new_feature = new_keyframe->add_feature();

                        new_feature->CopyFrom(ref_feature);
                        if (!select_mappoint)
                        {
                            new_feature->clear_mappoint_id();
                            flag = true;
                        }
                        if (!select_feature)
                        {
                            new_feature->clear_briefdescriptor();
                            flag = true;
                        }
                        cnt++;
                    }
                }
            }
        }
        if (flag)
            cerr << "WARNING DELETED SOMETHING" << endl;

        nFeatures = cnt;
        protoc_featuresArray = std::move(selected_features);
    }

    void OSMapData::select(const Selector &select)
    {
        selectMappoints(select);
        selectKeyframes(select);
        selectFeatures(select);
    }

    void OSMapData::saveMap(string yaml_path)
    {
        string path_dir = strpath_directory(yaml_path);
        string path_filename = strpath_filename_noext(yaml_path);
        string path_noext = strpath_noext(yaml_path);

        mappointsFile = path_filename + ".mappoints";
        keyframesFile = path_filename + ".keyframes";
        featuresFile = path_filename + ".features";
        trackingFile = path_filename + ".tracking";

        cv::FileStorage yaml_out(yaml_path, cv::FileStorage::WRITE);

        yaml_out << "mappointsFile" << mappointsFile;
        yaml_out << "nMappoints" << nMappoints;
        yaml_out << "keyframesFile" << keyframesFile;
        yaml_out << "nKeyframes" << nKeyframes;
        yaml_out << "featuresFile" << featuresFile;
        yaml_out << "nFeatures" << nFeatures;
        yaml_out << "trackingFile" << trackingFile;
        yaml_out << "nFrames" << nTackingFrames;

        yaml_out << "Options" << (int)options.to_ullong();
        yaml_out << "Options descriptions"
                 << "[:";
        for (auto s : options_str)
            yaml_out << s;
        yaml_out << "]";

        yaml_out << "cameraMatrices"
                 << "[";
        for (size_t i = 0; i < cx.size(); i++)
        {
            yaml_out << "{"
                     << "fx" << fx[i] << "fy" << fy[i] << "cx" << cx[i] << "cy" << cy[i] << "baseline" << baseline[i] << "}";
        }
        yaml_out.release();

        saveMappoints(strpath_join(path_dir, mappointsFile));
        saveKeyframes(strpath_join(path_dir, keyframesFile));
        saveFeatures(strpath_join(path_dir, featuresFile));
        saveTracking(strpath_join(path_dir, trackingFile));
    }

}