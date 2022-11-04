
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


#ifndef SET_COVER_FUNCTIONS_H
#define SET_COVER_FUNCTIONS_H

#include "mapselect/functions/SetFunction.h"
#include "mapselect/maps/MapDataAlias.h"

#include <algorithm>
#include <math.h>



template <class T>
class GreedyCover : public EvaluableFunction<T>
{
public:
    GreedyCover(std::shared_ptr<MapDataAlias> map_data_ptr, size_t b, T l = 0) : map_data(map_data_ptr), B(b), lambda(l), frame_obs_cnt(map_data->getFrameSize(), 0U)
    {
    }

    T gain(size_t key)
    {
        T sum;
        const auto& map_obs = map_data->mapKeyToFrameObs(key);
        if (lambda)
        {
            sum  = map_obs.size();
            for(size_t frame_key : map_obs)
                if (frame_obs_cnt[frame_key] < B)
                    sum += lambda;
        }
        else 
        {
            sum = 0U;
            for(size_t frame_key : map_obs)
                if (frame_obs_cnt[frame_key] < B)
                    sum++;
        }
        return sum;
    }


    T eval() const override
    {
        T sum = 0;
        for(size_t frame_key = 0; frame_key < map_data->n_frames; frame_key++)
        {
            size_t frame_obs = frame_obs_cnt[frame_key];
            T coverage = std::min<size_t>(frame_obs,B);
            if (lambda)
                sum += lambda*coverage + frame_obs; 
            else 
                sum+=frame_obs_cnt[frame_key];
        }
        return sum;
    }
 

protected:
    void reset() override
    {
        std::fill(frame_obs_cnt.begin(),frame_obs_cnt.end(),0);
    }


    void update(size_t key)
    {
        for (size_t frame_key : map_data->mapKeyToFrameObs(key))
        {
            frame_obs_cnt[frame_key]++;
        }
    }

    std::shared_ptr<MapDataAlias> map_data;
    std::vector<size_t> frame_obs_cnt;
    size_t B;
    T lambda;
};




template <class T>
class CachedGreedyCover : public GreedyCover<T>
{
public:
    CachedGreedyCover(std::shared_ptr<MapDataAlias> map_data_ptr, size_t B, T l = 0) : GreedyCover<T>(map_data_ptr, B, l)
    {

        size_t N = map_data_ptr->getMapSize();

        cached_scores = std::vector<T>(N, 0);
        for (size_t map_key = 0U; map_key < N; map_key++)
        {
            cached_scores[map_key] = map_data_ptr->mapKeyToFrameObs(map_key).size() * (l + 1);
        }
    }

    T gain(size_t key)
    {
        // std::cout << "GAIN" << std::endl;
        return cached_scores[key];
    }

protected:
    void reset() override
    {
        std::fill(cached_scores.begin(),cached_scores.end(),0);
    }


    void updateCoverage(size_t key)
    {
        if (this->lambda)
        {
            for (size_t frame_key : this->map_data->mapKeyToFrameObs(key))
            {
                if (++(this->frame_obs_cnt[frame_key]) == this->B)
                {
                    for(size_t map_key : this->map_data->frameKeyToMapObs(frame_key))
                        cached_scores[map_key] -= this->lambda;
                }
            }
        }
        else 
        {
            for (size_t frame_key : this->map_data->mapKeyToFrameObs(key))
            {
                if (++(this->frame_obs_cnt[frame_key]) == this->B)
                {
                    for(size_t map_key : this->map_data->frameKeyToMapObs(frame_key))
                        cached_scores[map_key]--;
                }
            }

        }
    }

    void update(size_t item)
    {
        // std::cout << "UPDATE" << std::endl;
        updateCoverage(item);
    }

    std::vector<T> cached_scores;
};




#endif
