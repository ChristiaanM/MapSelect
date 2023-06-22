/*
 * File: mselect.cpp
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


#include "mapselect/algorithms/LazyGreedy.h"
#include "mapselect/algorithms/RandomSet.h"
#include "mapselect/algorithms/StochasticGreedy.h"
#include "mapselect/algorithms/WeightedGreedy.h"
#include "mapselect/functions/LoopCover.h"
#include "mapselect/functions/LoopProb.h"


#include "mapselect/functions/FrameLocal.h"
#include "mapselect/functions/FrameOdometry.h"

#include "mapselect/functions/SaturatedSetFunction.h"
#include "mapselect/functions/SetCover.h"
#include "mapselect/functions/SetFunction.h"
#include "mapselect/functions/SLAMInfoDense.h"
#include "mapselect/functions/SLAMInfoGTSAM.h"
#include "mapselect/slam/SLAMGraphGTSAM.h"
#include "mapselect/functions/WeightedFunctionPair.h"
#include "mapselect/maps/MapDataAlias.h"
#include "mapselect/maps/OSMapAlias.h"
#include "mapselect/utils/BucketQueue.h"
#include "mapselect/utils/PathUtils.h"
#include "mapselect/utils/Timing.h"
#include "mapselect/utils/Disclaimer.h"

#include "mapselect/unsupported/functions/CoverSat.h"
#include "mapselect/unsupported/algorithms/CostBenefitGreedy.h"
#include "mapselect/unsupported/functions/FrameEstimationLegacy.h"


#ifdef CHOLMOD_FEATURES
#include "mapselect/functions/SLAMInfoCholmod.h"
#endif

#ifdef GUROBI_FEATURES
#include "mapselect/algorithms/GurobiOpt.h"
#endif

#include <fstream>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <string>

#include <opencv2/core/core.hpp> // yaml
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace mselect;

namespace po = boost::program_options;

#define ENUM_TAG(enum_type) _##enum_type

#define ENUM_STR_RESERVE(enum_type, size)                    \
    namespace ENUM_TAG(enum_type)                            \
    {                                                        \
        std::vector<std::string> enum2str(size);             \
        std::unordered_map<std::string, enum_type> str2enum; \
        std::vector<std::string> enum2desc(size);            \
    }

#define ENUM_STR(enum_type, enum, str, desc)              \
    ENUM_TAG(enum_type)::enum2str[enum_type::enum] = str; \
    ENUM_TAG(enum_type)::str2enum[str] = enum_type::enum; \
    ENUM_TAG(enum_type)::enum2desc[enum_type::enum] = desc
#define ENUM2STR(enum_type, enum) ENUM_TAG(enum_type)::enum2str[enum]
#define STR2ENUM(enum_type, str) ENUM_TAG(enum_type)::str2enum[str]
#define ENUM_OPTIONS(enum_type) ENUM_TAG(enum_type)::enum2str
#define ENUM_DESC(enum_type) ENUM_TAG(enum_type)::enum2desc

enum FunctionOption
{
    FUNCTION_ERROR = 0,
    FUNCTION_G2O_ODOM,
    FUNCTION_G2O_LOCAL,
    FUNCTION_WG2OODOM_COVER,
    FUNCTION_G2O_LOCAL_MEMORY,
    FUNCTION_G2O_ODOM_MEMORY,
    
    //FUNCTION_WODOM_PROB,
    //FUNCTION_WODOM_COVER,

#ifdef CHOLMOD_FEATURES
    FUNCTION_SLAM_CHOLMOD,
#endif

    FUNCTION_SLAM_INFO,
    FUNCTION_SLAM_DETLEMMA,
    FUNCTION_LOCAL_LEGACY,
    FUNCTION_ODOM_LEGACY,
    
    FUNCTION_COVER,
    FUNCTION_CACHED_COVER,

    //FUNCTION_ODOMETRY_COVER,

    FUNCTION_WG2OODOM_PROB,

#ifdef GUROBI_FEATURES
    FUNCTION_GUROBI_IPCOVER,
    //FUNCTION_GUROBI_QIPCOVER,
#endif



    FUNCTION_SIZE // not a function
};

enum AlgorithmOption
{
    ALGORITHM_ERROR = 0,
    ALGORITHM_LAZY_GREEDY_HEAP,
    ALGORITHM_LAZY_GREEDY_BUCKET,
    ALGORITHM_STOCHASTIC_GREEDY,
    ALGORITHM_RANDOM,
    ALGORITHM_WSEARCH,
    ALGORITHM_WGREEDY,
    ALGORITHM_IP,

    ALGORITHM_LOAD,
    ALGORITHM_DEBUG_COPY,
    ALGORITHM_ESSENTIAL_MAP,
    //ALGORITHM_SATURATE,
    //ALGORITHM_COST_BENEFIT,


    ALGORITHM_SIZE // not a algorithm
};

enum MapTypeOption
{
    MAPTYPE_ERROR = 0,
    MAPTYPE_OSMAP,
    //MAPTYPE_BUNDLER,
    MAPTYPE_SIZE
};

ENUM_STR_RESERVE(FunctionOption, FUNCTION_SIZE)
ENUM_STR_RESERVE(AlgorithmOption, ALGORITHM_SIZE)
ENUM_STR_RESERVE(MapTypeOption, MAPTYPE_SIZE)

std::string options2string(std::string prefix, std::vector<std::string> &options, std::vector<std::string> &desc)
{
    ostringstream oss;

    oss << "--------------------------------------------" << std::endl;
    oss << prefix << std::endl;
    oss << "--------------------------------------------" << std::endl;
        
    size_t max = 0;
    for(size_t i = 0 ; i < options.size();i++)
        if (options[i].size() > max)
            max = options.size();

    for (size_t i = 1; i < options.size() && i < desc.size(); i++)
    {

        oss << std::left << std::setw(max) << options[i] << "   " << desc[i] << std::endl;
    }
    return oss.str();
}

void enum_setup(std::string &algoptions, std::string &foptions, std::string &moptions)
{

    ENUM_STR(FunctionOption, FUNCTION_G2O_LOCAL, "local-fast","Localisation approximation");
    ENUM_STR(FunctionOption, FUNCTION_G2O_ODOM, "odometry-fast","Odometry approximation");
    ENUM_STR(FunctionOption, FUNCTION_G2O_LOCAL_MEMORY, "local-mem","Alternative local-fast that uses less memory");
    ENUM_STR(FunctionOption, FUNCTION_G2O_ODOM_MEMORY, "odometry-mem","Alternative odometry-fast approximation that uses less memory");
    ENUM_STR(FunctionOption, FUNCTION_LOCAL_LEGACY, "local-legacy","Localisation approximation using gtsam (legacy)");
    ENUM_STR(FunctionOption, FUNCTION_ODOM_LEGACY, "odometry-legacy","Odometry approximation using gtsam (legacy)");

#ifdef CHOLMOD_FEATURES
    ENUM_STR(FunctionOption, FUNCTION_SLAM_CHOLMOD, "slam-cholmod", "SLAM info gain using CholMod (less slow, very expensive)");
#endif
    ENUM_STR(FunctionOption, FUNCTION_SLAM_DETLEMMA, "slam-dense", "SLAM info gain using matrix det lemma (for debugging - exceptionally slow)");
    ENUM_STR(FunctionOption, FUNCTION_SLAM_INFO, "slam-gtsam", "SLAM info gain using gtsam (for debugging - exceptionally slow)");

    
    ENUM_STR(FunctionOption, FUNCTION_WG2OODOM_COVER,"wodom+cover","Combined odometry+cover formulation, use with wgreedy to normalise properly. Additionally, loop closure frames must be provided.");
    ENUM_STR(FunctionOption, FUNCTION_WG2OODOM_PROB,"wodom+prob","Combined odometry+cover formulation, use with wgreedy to normalise properly. Additionally, loop closure frames must be provided.");

    ENUM_STR(FunctionOption, FUNCTION_COVER, "cover","frame coverage");
    ENUM_STR(FunctionOption, FUNCTION_CACHED_COVER, "cover-alt","alternate implementation of frame coverage");

    ENUM_STR(AlgorithmOption, ALGORITHM_LAZY_GREEDY_HEAP, "greedy","Classic lazy greedy algorithm");
    ENUM_STR(AlgorithmOption, ALGORITHM_LAZY_GREEDY_BUCKET, "bucket","Lazy greedy algorithm for integer objectives");
    ENUM_STR(AlgorithmOption, ALGORITHM_STOCHASTIC_GREEDY, "stochastic","Stochastic greedy, adjust -eps to trade-off execution time and function value.");
    ENUM_STR(AlgorithmOption, ALGORITHM_RANDOM, "random","Randomly select map points");
    ENUM_STR(AlgorithmOption, ALGORITHM_ESSENTIAL_MAP, "essential","Debugging - remove all non-map point data");
    ENUM_STR(AlgorithmOption, ALGORITHM_DEBUG_COPY, "copy","Debugging - Just copy a map");
    ENUM_STR(AlgorithmOption, ALGORITHM_LOAD, "load","Debugging - load a selection");

    // ENUM_STR(AlgorithmOption, ALGORITHM_SATURATE, "saturate");
    ENUM_STR(AlgorithmOption, ALGORITHM_WGREEDY, "wgreedy","Use with combined odometry+cover formulations");
    ENUM_STR(AlgorithmOption, ALGORITHM_WSEARCH, "wsearch","Unsupported - search over weights for combined formulations");

    ENUM_STR(MapTypeOption, MAPTYPE_OSMAP, "osmap","OSMAP file type");

#ifdef GUROBI_FEATURES
    ENUM_STR(FunctionOption, FUNCTION_GUROBI_IPCOVER, "ip_cover","Integer programming approach - use with iprogram algorithm");
    ENUM_STR(AlgorithmOption, ALGORITHM_IP, "iprogram","Gurobi Solver - use with ip-cover");
#endif

    foptions = "Function to optimise. Use --help for more details.";
    algoptions = "Algorithm implementation. Use --help for more details.";
    moptions = "MapType - Only osmap is currently supported in public release";

 
}

istream &operator>>(istream &in, FunctionOption &func)
{
    string token;
    in >> token;
    transform(token.begin(), token.end(), token.begin(), ::tolower);
    FunctionOption opt = STR2ENUM(FunctionOption, token);
    if (opt == FunctionOption::FUNCTION_ERROR)
    {
        throw boost::program_options::validation_error(boost::program_options::validation_error::invalid_option, "Invalid Function");
    }
    func = opt;
    return in;
}

istream &operator>>(istream &in, AlgorithmOption &algorithm)
{
    string token;
    in >> token;
    transform(token.begin(), token.end(), token.begin(), ::tolower);
    AlgorithmOption opt = STR2ENUM(AlgorithmOption, token);
    if (opt == AlgorithmOption::ALGORITHM_ERROR)
    {
        throw boost::program_options::validation_error(boost::program_options::validation_error::invalid_option, "Invalid Algorithm");
    }
    algorithm = opt;
    return in;
}

istream &operator>>(istream &in, MapTypeOption &maptype)
{
    string token;
    in >> token;
    transform(token.begin(), token.end(), token.begin(), ::tolower);
    MapTypeOption opt = STR2ENUM(MapTypeOption, token);
    if (opt == MapTypeOption::MAPTYPE_ERROR)
    {
        throw boost::program_options::validation_error(boost::program_options::validation_error::invalid_option, "Invalid Map Type");
    }
    maptype = opt;
    return in;
}

int main(int argc, char const *argv[])
{

    std::cout << DISCLAIMER_TXT << std::endl;

    string algoptions, foptions, moptions;
    enum_setup(algoptions, foptions, moptions);

    po::options_description description("Selection Test Usage");
    MapTypeOption maptype_option;
    AlgorithmOption algorithm_option; // = vm["function"].as<QueueOption>();
    FunctionOption func_option;       // = vm["queue"].as<FunctionOption>();

    string output_map_path;
    string input_map_path;
    string out_ext;

    string heuristic_path;
    // = vm["input"].as<std::string>();
    size_t N; // =  vm["cardinality"].as<size_t>();
    size_t B; // = vm["coverage"].as<size_t>();
    size_t B_min;
    double l_double; // = vm["lambda"].as<size_t>();
    size_t l_uint;
    bool f_uint = false;

    size_t last_frames; // = vm["tracking_frames"].as<std::size_t>();

    bool verbose, aux, no_output, overwrite, descriptors;
    double eps;

    std::vector<size_t> cover_frames;

    double ratio;

    description.add_options()("help,h", "Display this description of usage options")                                                                                                               //
        ("input,i", po::value<std::string>(&input_map_path)->required(), "Input file")                                                                                                             //
        ("maptype,m", po::value<MapTypeOption>(&maptype_option)->default_value(MAPTYPE_OSMAP), moptions.c_str())                                                                                   //
        ("output,o", po::value<std::string>(&output_map_path), "Ouput file path")                                                                                                                  //
        ("function,f", po::value<FunctionOption>(&func_option)->default_value(FUNCTION_ERROR), foptions.c_str())                                                                                   //
        ("algorithm,a", po::value<AlgorithmOption>(&algorithm_option)->default_value(ALGORITHM_LAZY_GREEDY_HEAP), algoptions.c_str())                                                              //
        ("lambda,l", po::value<double>()->default_value(0), "Lambda value for scoring observations. Use l=0 for classic coverage, else obs score = (l+1)*(uncovered frames) + 1*(covered frames)") //
        ("cardinality,N", po::value<size_t>(&N)->default_value(0), "Number of mappoints to select")                                                                                                //
        ("ratio,R", po::value<double>(&ratio)->default_value(0), "Number of mappoints as a ratio of the input map")                                                                                //
        ("coverage,B", po::value<size_t>()->default_value(0), "The number of observations before a frame is covered")                                                                              //
        ("tracking_frames,T", po::value<size_t>()->default_value(1), "Enable a heuristic to select all observations from the last t frames.")                                                      //
        ("bmin", po::value<size_t>()->default_value(0), "Lower bound obs for QIP models")                                                                                                          //
        ("verbose,v", "Print to commandline")                                                                                                                                                      //
        ("auxfiles", "(depreciated  - .timing and .select files are now always generated")                                                                                                         //
        ("no_output", "Disable the creation of a output map")                                                                                                                                      //
        ("overwrite", "Overwrite files at target location. Otherwise a number is appended.")                                                                                                       //
        ("eps", po::value<double>()->default_value(0.01), "Epsilon used for stachastic greedy")                                                                                                    //
        ("cover_frames", po::value<std::vector<size_t>>(&cover_frames)->multitoken(), "Frames to be covered by oracle functions")                                                                  //
        ("heuristic", po::value<std::string>(&heuristic_path), "Heuristic path")                                                                                                                   //
        ("descriptors", "Only delete descriptors");

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(description).run(), vm);

    if (vm.count("help") || argc == 1)
    {
        std::cout << description << std::endl;
        std::cout << options2string("Function Options (-f)",ENUM_OPTIONS(FunctionOption),ENUM_DESC(FunctionOption));
        std::cout << options2string("Algorithm Options (-a)",ENUM_OPTIONS(AlgorithmOption),ENUM_DESC(AlgorithmOption));

        return 0;
    }

    po::notify(vm);

    verbose = vm.count("verbose");
    aux = true; // vm.count("auxfiles");
    no_output = vm.count("no_output");
    overwrite = vm.count("overwrite");
    descriptors = vm.count("descriptors");
    B = vm["coverage"].as<size_t>();
    B_min = vm["bmin"].as<size_t>();
    last_frames = vm["tracking_frames"].as<size_t>();
    eps = vm["eps"].as<double>();

    {
        l_double = vm["lambda"].as<double>();
        double z;
        double frac = std::modf(l_double, &z);
        if (z >= 0.0 && frac < 1e-9)
        {
            f_uint = true;
            l_uint = round(l_double);
        }
    }
    /*
    if (N==0 && ratio==0)
    {
        std::cout << "N not set - either define the number of mappoints N or define it as a ratio R of the input map" << std::endl;
        return 1;
    }
    */
    /*if (maptype_option == MAPTYPE_BUNDLER)
    {
        last_frames = 0;
        out_ext = ".out";
    }
    else*/
    out_ext = ".yaml";

    if (algorithm_option == ALGORITHM_LAZY_GREEDY_BUCKET && !f_uint)
    {
        std::cout << "Can't use bucket queues with non-integer lambdas" << endl;
        return 1;
    }

    if (algorithm_option == ALGORITHM_LAZY_GREEDY_BUCKET && (func_option != FUNCTION_COVER && func_option != FUNCTION_CACHED_COVER))
    {
        std::cout << "Can't use bucket queues with real valued functions!" << endl;
        return 1;
    }

    if (algorithm_option == ALGORITHM_RANDOM || algorithm_option == ALGORITHM_LOAD)
    {
        func_option = FUNCTION_ERROR;
        l_double = 0.0;
        B = 0;
    }

    if (algorithm_option == ALGORITHM_DEBUG_COPY || algorithm_option == ALGORITHM_ESSENTIAL_MAP)
    {
        func_option = FUNCTION_ERROR;
        N = 0;
        l_double = 0.0;
        B = 0;
        last_frames = 0;
    }

    if (!vm.count("output"))
    {
        ostringstream oss;
        oss << strpath_filename_noext(input_map_path) << "_" << ENUM2STR(FunctionOption, func_option) << ENUM2STR(AlgorithmOption, algorithm_option);
        if (N && ratio == 0)
            oss << 'N' << N;
        else
            oss << 'R' << round(ratio * 100);
        output_map_path = oss.str();
        if (!overwrite)
        {
            size_t i = 0, MAX_CHECK = 1000;

            for (i = 0U; i < MAX_CHECK; i++)
            {
                ostringstream prefix_ss;
                prefix_ss << output_map_path << '_' << setfill('0') << std::setw(3) << i;
                std::ifstream infile(prefix_ss.str() + out_ext);
                if (!infile.good())
                {
                    output_map_path = prefix_ss.str();
                    break;
                }
            }
            if (i == MAX_CHECK)
            {
                cout << "Failed to find a open map name!" << endl;
                return 1;
            }
        }
    }

    if (verbose)
    {
        cout << "Input:\t" << input_map_path << endl;
        if (!no_output)
            cout << "Ouput:\t" << output_map_path << endl;
        if (N)
            cout << "N:\t" << N << endl;
        if (B)
            cout << "B:\t" << B << endl;
        if (l_double)
            cout << "l:\t" << l_double << endl;
        if (last_frames)
            cout << "T:\t" << last_frames << endl;
        if (cover_frames.size())
        {
            cout << "Cover Frames:\t";

            for (size_t frame : cover_frames)
            {
                cout << frame << " ";
            }
            cout << endl;
        }

        cout << std::string(10, '-') << endl;
    }

    size_t time_fsetup, time_algo, time_select, time_load, time_write, time_alias, time_opt, time_split, time_dealias;

    std::shared_ptr<MapData> raw_map_data;
    std::shared_ptr<MapDataAlias> map_data;

    if (maptype_option == MAPTYPE_OSMAP)
    {
        TIC(map_load);
        auto osmap_data = std::make_shared<OSMapData>(input_map_path);
        TOC(map_load);
        time_load = TIME(map_load);

        TIC(alias);
        map_data = std::make_shared<OSMapDataAlias>(*osmap_data);
        TOC(alias);
        time_alias = TIME(alias);

        raw_map_data = osmap_data;
    }
    /*else if (maptype_option == MAPTYPE_BUNDLER)
    {
        TIC(map_load);
        auto bundler_data = std::make_shared<BundlerData>(input_map_path);
        TOC(map_load);
        time_load = TIME(map_load);

        TIC(alias);
        map_data = std::make_shared<BundlerAlias>(*bundler_data);
        TOC(alias);
        time_alias = TIME(alias);

        raw_map_data = bundler_data;
    }*/

    if (ratio)
    {
        if (N != 0)
        {
            N = N * ratio;
        }
        else
            N = map_data->n_points * ratio;
    }

    TIC(algo);
    TIC(split);
    std::vector<size_t> valid_keys, heuristic_keys;
    if (vm.count("heuristic"))
    {

        ifstream heuristic_file(heuristic_path);
        std::cout << "Heuristic File:\t" << heuristic_path << std::endl;
        if (!heuristic_file.is_open())
        {
            std::cout << "Failed to open select file" << std::endl;
            return 1;
        }

        size_t number;
        size_t cnt = 0;
        std::vector<size_t> ids;

        while (heuristic_file >> number && (N == 0 || cnt < N))
        {
            ids.push_back(number);
            cnt++;
        }
        std::cout << "max" << map_data->n_points << std::endl;
        inverse_keyvector(map_data->map_alias->ids2keys(ids), valid_keys, heuristic_keys, map_data->n_points);
    }

    else
    {
        if (verbose)
            std::cout << "Using last frame heuristic with " << last_frames << " frames." << std::endl;
        map_data->lastFramesHeuristic(last_frames, valid_keys, heuristic_keys);
    }

    TOC(split);
    time_split = TIME(split);

    if (verbose)
        std::cout << "Heuristic Tracking Points " << heuristic_keys.size() << std::endl;

    size_t N_greedy = N;
    if (N == 0)
        N_greedy = std::numeric_limits<size_t>::max();

    double value = 0;
    std::vector<size_t> selected_ids;

    std::shared_ptr<IncrementalSetFunction<double>> func_d;
    std::shared_ptr<IncrementalSetFunction<size_t>> func_i;

    TIC(fsetup);
    switch (func_option)
    {
    case FUNCTION_COVER:
        if (f_uint)
            func_i = std::make_shared<FrameCoverage<size_t>>(map_data, B, l_uint);
        else
            func_d = std::make_shared<FrameCoverage<double>>(map_data, B, l_double);
        break;
    case FUNCTION_CACHED_COVER:
        if (f_uint)
            func_i = std::make_shared<CachedFrameCoverage<size_t>>(map_data, B, l_uint);
        else
            func_d = std::make_shared<CachedFrameCoverage<double>>(map_data, B, l_double);
        break;
    case FUNCTION_LOCAL_LEGACY:
        func_d = std::make_shared<FrameEstimationLegacy>(map_data, FrameEstimationFunction::LEGACY_GTSAM_LOCAL);
        f_uint = false;
        break;
    case FUNCTION_ODOM_LEGACY:
        func_d = std::make_shared<FrameEstimationLegacy>(map_data, FrameEstimationFunction::LEGACY_GTSAM_ODOMETRY);
        f_uint = false;
        break;
    /*case FUNCTION_ODOMETRY_COVER:
    {
        {
            auto func_tmp = std::make_shared<LoopCover>(map_data, cover_frames);
            auto func_odom = std::make_shared<FrameEstimationLegacy>(map_data, FrameEstimationFunction::LEGACY_GTSAM_ODOMETRY, true);
            auto joined_func = std::make_shared<TwinSaturatedSetFunction<LoopCover, FrameEstimationLegacy>>(func_tmp, l_double, func_odom, 10e3);
            func_d = joined_func;
            f_uint = false;
        }
        break;
    }*/
    /*case FUNCTION_WODOM_COVER:
    {
        {
            auto func_odom = std::make_shared<FrameEstimationLegacy>(map_data, FrameEstimationFunction::LEGACY_GTSAM_ODOMETRY, true);
            func_odom->changeB(func_odom->getBmax());
            auto func_cover = std::make_shared<LoopCover>(map_data, cover_frames);
            func_cover->changeB(B);
            auto wfunc = std::make_shared<WeightedFunctionPair>(func_odom, func_cover);
            func_d = wfunc;
            f_uint = false;
        }
        break;
    }*/
    case FUNCTION_WG2OODOM_COVER:
    {
        {
            auto func_odom = std::make_shared<FrameEstimationLegacy>(map_data, FrameEstimationFunction::LEGACY_G2O_ODOM, true);
            func_odom->changeB(func_odom->getBmax());
            auto func_cover = std::make_shared<LoopCover>(map_data, cover_frames);
            func_cover->changeB(B);
            auto wfunc = std::make_shared<WeightedFunctionPair>(func_odom, func_cover);
            func_d = wfunc;
            f_uint = false;
        }
        break;
    }

    case FUNCTION_WG2OODOM_PROB:
    {
        {
            auto func_odom = std::make_shared<FrameEstimationLegacy>(map_data, FrameEstimationFunction::LEGACY_G2O_ODOM, true);
            func_odom->changeB(func_odom->getBmax());
            auto func_cover = std::make_shared<LoopProbCoverPvis>(map_data, cover_frames);
            func_cover->changeB(B);
            auto wfunc = std::make_shared<WeightedFunctionPair>(func_odom, func_cover);
            func_d = wfunc;
            f_uint = false;
        }
        break;
    }

    /*case FUNCTION_WODOM_PROB:
    {
        {
            auto func_odom = std::make_shared<FrameEstimationLegacy>(map_data, FrameEstimationFunction::LEGACY_GTSAM_ODOMETRY, true);
            func_odom->changeB(func_odom->getBmax());
            auto func_cover = std::make_shared<LoopProbCoverPvis>(map_data, cover_frames);
            func_cover->changeB(B);
            auto wfunc = std::make_shared<WeightedFunctionPair>(func_odom, func_cover);
            func_d = wfunc;
            f_uint = false;
        }
        break;
    }*/
    case FUNCTION_SLAM_INFO:
        func_d = std::make_shared<SLAMInfoGTSAM>(map_data);
        f_uint = false;
        break;
    case FUNCTION_SLAM_DETLEMMA:
        func_d = std::make_shared<SLAMInfoDet>(map_data);
        f_uint = false;
        break;
    
    case FUNCTION_G2O_ODOM:
        func_d = std::make_shared<FrameEstimationLegacy>(map_data, FrameEstimationFunction::LEGACY_G2O_ODOM);
        f_uint = false;
        break;

    case FUNCTION_G2O_LOCAL:
        func_d = std::make_shared<FrameEstimationLegacy>(map_data, FrameEstimationFunction::LEGACY_G2O_LOCAL);
        f_uint = false;
        break;

    case FUNCTION_G2O_ODOM_MEMORY:
        func_d = std::make_shared<FrameOdometryMemory>(map_data);
        f_uint = false;
        break;

    case FUNCTION_G2O_LOCAL_MEMORY:
        func_d = std::make_shared<FrameLocalMemory>(map_data);
        f_uint = false;
        break;

#ifdef CHOLMOD_FEATURES
    case FUNCTION_SLAM_CHOLMOD:
        func_d = std::make_shared<SLAMInfoCholmod>(map_data);
        f_uint = false;
        break;
#endif
    default:
        break;
    }

    TOC(fsetup);
    time_fsetup = TIME(fsetup);

    if (algorithm_option == ALGORITHM_DEBUG_COPY)
        raw_map_data->select(DummySelector());
    else if (algorithm_option == ALGORITHM_ESSENTIAL_MAP)
        raw_map_data->select(EssentialSelector(map_data->getLastFrameId()));
    else if (algorithm_option == ALGORITHM_LOAD)
    {
        string path_noext = strpath_noext(input_map_path);

        string settings_path = path_noext + "_select.yaml";
        cv::FileStorage settings_file(settings_path, cv::FileStorage::READ);
        std::cout << "Settings File:\t" << settings_path << std::endl;
        if (!settings_file.isOpened())
        {
            std::cout << "Failed to open settings file" << std::endl;
            return 1;
        }

        string select_path = path_noext + ".select";
        ifstream select_file(select_path);
        std::cout << "Select File:\t" << select_path << std::endl;
        if (!select_file.is_open())
        {
            std::cout << "Failed to open select file" << std::endl;
            return 1;
        }

        string funcstr;
        string algostr;

        settings_file["Function"] >> funcstr;
        settings_file["Algorithm"] >> algostr;

        func_option = STR2ENUM(FunctionOption, funcstr);
        algorithm_option = STR2ENUM(AlgorithmOption, algostr);

        std::cout << "Algorithm:\t" << algostr << std::endl;
        std::cout << "Function:\t" << funcstr << std::endl;

        int number;
        int cnt = 0;
        while (select_file >> number && (N == 0 || cnt < N))
        {
            selected_ids.push_back(number);
            cnt++;
        }

        raw_map_data->select(MappointSelector(selected_ids, map_data->getLastFrameId()));
    }

    else
    {
        TIC(opt);
        if (func_i)
        {
            func_i->addBatch(heuristic_keys);

            switch (algorithm_option)
            {
            case ALGORITHM_LAZY_GREEDY_HEAP:
                LazyGreedy<size_t, StablePriorityQueue<size_t>>(*func_i, valid_keys, N_greedy);
                break;
            case ALGORITHM_LAZY_GREEDY_BUCKET:
                LazyGreedy<size_t, BucketQueue>(*func_i, valid_keys, N_greedy);
                break;
            case ALGORITHM_STOCHASTIC_GREEDY:
                StochasticGreedy(*func_i, valid_keys, N_greedy, eps);
                break;

            default:
                cout << ENUM2STR(AlgorithmOption, algorithm_option) << " not a valid algorithm for " << ENUM2STR(FunctionOption, func_option) << endl;
                return 1;
                break;
            }

            TIC(dealias);
            selected_ids = map_data->convertMappointKeysToIDs(func_i->getSelected());
            TOC(dealias);
            time_dealias = TIME(dealias);
        }
        else if (func_d)
        {
            func_d->addBatch(heuristic_keys);

            switch (algorithm_option)
            {
            case ALGORITHM_LAZY_GREEDY_HEAP:
                LazyGreedy<double, StablePriorityQueue<double>>(*func_d, valid_keys, N_greedy);
                break;
            case ALGORITHM_STOCHASTIC_GREEDY:
                StochasticGreedy(*func_d, valid_keys, N_greedy, eps);
                break;
            /*case ALGORITHM_COST_BENEFIT:
                {
                    //auto func_deval = std::dynamic_pointer_cast<EvaluableFunction<double>>(func_d);
                    std::vector<double> costs;
                    for(size_t i=0;i<map_data->n_points;i++)
                        costs.push_back(map_data->getMappointCost(i));
                    if (func_d)
                        CostBenefitGreedy<double,StablePriorityQueue<double>>(*func_d,valid_keys,costs,(double) N);
                }
                break;*/
            case ALGORITHM_WSEARCH:
            {
                std::shared_ptr<WeightedFunctionPair> func_search = std::dynamic_pointer_cast<WeightedFunctionPair>(func_d);
                if (func_search)
                {
                    WSearchSettings settings;
                    settings.norm_scheme = WeightingNormalisationScheme::NORM_LAZY_GREEDY;
                    WeightedBinarySearch(*func_search, valid_keys, heuristic_keys, N,settings);
                }
                else
                {
                    cout << ENUM2STR(AlgorithmOption, algorithm_option) << " not a valid algorithm for " << ENUM2STR(FunctionOption, func_option) << endl;
                    return 1;
                }
            }
            break;
            break;
            case ALGORITHM_WGREEDY:
            {
                WSearchSettings settings_one_iter;
                settings_one_iter.max_iters = 1;
                settings_one_iter.norm_scheme = WeightingNormalisationScheme::NORM_FULL_SET;
                std::shared_ptr<WeightedFunctionPair> func_search = std::dynamic_pointer_cast<WeightedFunctionPair>(func_d);
                if (func_search)
                {
                    WeightedBinarySearch(*func_search, valid_keys, heuristic_keys, N,settings_one_iter);
                }
                else
                {
                    cout << ENUM2STR(AlgorithmOption, algorithm_option) << " not a valid algorithm for " << ENUM2STR(FunctionOption, func_option) << endl;
                    return 1;
                }
            }
            break;

            default:
                cout << ENUM2STR(AlgorithmOption, algorithm_option) << " not a valid algorithm for " << ENUM2STR(FunctionOption, func_option) << endl;
                return 1;
                break;
            }

            TIC(dealias);
            selected_ids = map_data->convertMappointKeysToIDs(func_d->getSelected());
            TOC(dealias);
            time_dealias = TIME(dealias);
        }
        else
        {
            std::vector<size_t> selected_keys;
#ifdef GUROBI_FEATURES
            if (algorithm_option == ALGORITHM_IP)
            {
                if (func_option != FUNCTION_GUROBI_IPCOVER) //and func_option != FUNCTION_GUROBI_QIPCOVER)
                {
                    cout << ENUM2STR(AlgorithmOption, algorithm_option) << " not a valid algorithm for " << ENUM2STR(FunctionOption, func_option) << endl;
                    return 1;
                }

                GurobiSettings ipsettings;
                ipsettings.b = B;
                ipsettings.b_min = B_min;
                ipsettings.lambda = l_double;
                ipsettings.N = N;
                //ipsettings.quad_penalty = (func_option == FUNCTION_GUROBI_QIPCOVER);
                ipsettings.forced_keys = &heuristic_keys;
                selected_keys = GurobiIPCover(*map_data, ipsettings);
            }
            else
#endif
                if (algorithm_option == ALGORITHM_RANDOM)
            {
                selected_keys = RandomSelection(valid_keys, N, &heuristic_keys);
            }
            else
            {
                cout << "Invalid algorithm and function combination" << endl;
                return 1;
            }
            TIC(dealias);
            selected_ids = map_data->convertMappointKeysToIDs(selected_keys);
            TOC(dealias);
            time_dealias = TIME(dealias);
        }

        TOC(opt);
        time_opt = TIME(opt) + time_fsetup - time_dealias;

        TIC(select);
        if (!descriptors)
            raw_map_data->select(MappointSelector(selected_ids, map_data->getLastFrameId()));
        else
            raw_map_data->select(MappointDescriptorSelector(selected_ids, map_data->getLastFrameId()));
        TOC(select);
        time_select = TIME(select);
    }

    TOC(algo);
    time_algo = TIME(algo) + time_alias - time_select;

    if (verbose)
    {
        size_t before = map_data->getMapSize();
        size_t after = selected_ids.size();
        double comp_ms = (double)(time_opt) / 1e6;
        double algo_ms = (double)(time_algo) / 1e6;
        double p = ((double)(after)*100) / before;
        cout << "Selected " << after << " (" << setprecision(4) << p << "%) mappoints from " << before << " within " << setprecision(6) << comp_ms << "ms" << endl
             << setprecision(6) << algo_ms << "ms including overhead but not file read/write" << endl;
    }

    if (!no_output)
    {
        if (verbose)
            cout << "Writing output file" << endl;

        TIC(map_write);
        raw_map_data->saveMap(output_map_path + out_ext);
        TOC(map_write);
        time_write = TIME(map_write);
        {
            cv::FileStorage select_file(output_map_path + "_select.yaml", cv::FileStorage::WRITE);
            select_file << "Function" << ENUM2STR(FunctionOption, func_option);
            select_file << "Algorithm" << ENUM2STR(AlgorithmOption, algorithm_option);
            select_file << "N" << (int)N;
            select_file << "N_select" << (int)selected_ids.size();
            select_file << "N_map" << (int)raw_map_data->getMapSize();
            select_file << "L" << l_double;
            select_file << "LisInt" << f_uint;
            select_file << "B" << (int)B;
            select_file << "T" << (int)last_frames;
            select_file.release();
        }
    }

    if (aux)
    {
        if (verbose)
            cout << "Writing aux files..." << endl;

        fstream select_file(output_map_path + ".select", ios::out);
        for (size_t i = 0, I = selected_ids.size(); i < I; i++)
        {
            select_file << selected_ids[i];
            if (i % 100 == 0 && i != 0)
                select_file << endl;
            else
                select_file << ' ';
        }

        fstream cmd_file(output_map_path + ".cmd", ios::out);
        for (size_t i = 0; i < argc; i++)
        {
            cmd_file << argv[i] << ' ';
        }

        fstream timing_file(output_map_path + ".timing", ios::out);
        {
            timing_file << timing::time_string("full-algorithm", time_algo) << endl;
            timing_file << timing::time_string("-opt", time_opt) << endl;
            timing_file << timing::time_string("-alias", time_alias + time_dealias) << endl;
            timing_file << timing::time_string("-split", time_split) << endl;

            timing_file << timing::time_string("IO", time_load + time_write + time_select) << endl;
            timing_file << timing::time_string("-read", time_load) << endl;
            timing_file << timing::time_string("-select", time_select) << endl;
            timing_file << timing::time_string("-write", time_write) << endl;
        }
    }

    if (verbose)
    {
        if (func_d)
            func_d->print();
        if (func_i)
            func_i->print();
    }

    if (verbose)
        cout << "Done!" << endl;

    return 0;
}
