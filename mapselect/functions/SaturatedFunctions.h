
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


#ifndef SATURATED_FUNCTIONS_H
#define SATURATED_FUNCTIONS_H


class CoverSat : public SaturatedSetFunction
{
    public:

    CoverSat(std::shared_ptr<MapDataAlias> map_data_ptr) : map_data(map_data_ptr)
    {
        frame_obs_cnt.resize(map_data_ptr->n_frames, 0U);
    }

    double getBmax() const override
    {
        double max_obs = 0;
        for (const auto &obs : map_data->frame_obs)
        {
            if (obs.size() > max_obs)
                max_obs = obs.size();
        }
        return max_obs;
    }

    double getBmin() const override
    {
        return 0;
    }

    double gain(size_t key) override
    {
        const auto &map_obs = map_data->mapKeyToFrameObs(key);
        double sum = 0;
        for (size_t frame_key : map_obs)
            if (frame_obs_cnt[frame_key] < b_sat)
            {
                sum += std::min<double>(1.0,b_sat-frame_obs_cnt[frame_key]);
            }
        //std::cout << "min" <<  sum << std::endl;
        //std::cout << b_sat << std::endl;
        return sum;
    }

    void update(size_t key) override
    {
        const auto &map_obs = map_data->mapKeyToFrameObs(key);
        for (size_t frame_key : map_obs)
        {
            frame_obs_cnt[frame_key]++;
        }
    }

    void print() const override
    {
        size_t min_key = 0;
        size_t min_obs = frame_obs_cnt[0];
        for (size_t i = 1; i < frame_obs_cnt.size(); i++)
        {
            if (frame_obs_cnt[i] < min_obs)
            {
                min_obs = frame_obs_cnt[i];
                min_key = i;
            }
        }
        std::cout << "min obs " << min_obs << " @ " << min_key << std::endl;
        std::cout << "cover-sat" << b_sat << std::endl;
    }

    protected:

        void reset() override
            {
                std::fill(frame_obs_cnt.begin(), frame_obs_cnt.end(), 0);
            }

            
        std::shared_ptr<MapDataAlias> map_data;
        std::vector<size_t> frame_obs_cnt;
};
class LoopCover : public SaturatedSetFunction, public EvaluableFunction<double>
{
    public:

    LoopCover(std::shared_ptr<MapDataAlias> map_data_ptr, const std::vector<size_t>& oracle_keys) : map_data(map_data_ptr)
    {
        for(size_t key : oracle_keys)
        {
            oracle_cnts[key] = 0;
        }
    }

    double getBmax() const override
    {
        double max_obs = 0;
        for(auto pair : oracle_cnts)
        {
             size_t obs = map_data->frame_obs[pair.first].size();
             if (obs > max_obs)
                max_obs = obs;
        }
        return max_obs;
    }

    double getBmin() const override
    {
        return 0;
    }

    double gain(size_t key) override
    {
        const auto &map_obs = map_data->mapKeyToFrameObs(key);
        double sum = 0;
        for (size_t frame_key : map_obs)
        {
            auto iter = oracle_cnts.find(frame_key);
            if (iter != oracle_cnts.end())
            {
                if (iter->second < b_sat )
                    sum += std::min<double>(1.0,b_sat-iter->second);
            }
        }
        return sum;
    }

    void update(size_t key) override
    {
        const auto &map_obs = map_data->mapKeyToFrameObs(key);
        double sum = 0;
        for (size_t frame_key : map_obs)
        {
            auto iter = oracle_cnts.find(frame_key);
            if (iter != oracle_cnts.end())
            {
                iter->second++;
            }
        }
    }

    void print() const override
    {
        size_t min_key = std::numeric_limits<size_t>::max();
        size_t min_obs = std::numeric_limits<size_t>::max();
        for(auto pair : oracle_cnts)
        {
            if (pair.second < min_obs)
            {
                min_obs = pair.second;
                min_key = pair.first;
            }
        }
        
        std::cout << "min obs " << min_obs << " @ " << min_key << std::endl;
        std::cout << "bsat" << b_sat << std::endl;
    }

    double eval() const override
    {
        double sum = 0;
        for(auto pair : oracle_cnts)
        {
            sum+=std::min<double>(b_sat,pair.second);
        }
        return sum;
    }

    protected:

        void reset() override
            {
                for(auto &pair : oracle_cnts)
                {
                    pair.second = 0;
                }

            }

        std::map<size_t,size_t> oracle_cnts;
        std::shared_ptr<MapDataAlias> map_data;
        
};

class LoopProb : public SaturatedSetFunction, public EvaluableFunction<double>
{
    public:


    virtual double custom_prob(size_t key)=0;

    void init(std::shared_ptr<MapDataAlias> map_data_ptr, const std::vector<size_t>& oracle_keys)
    {
        map_data = map_data_ptr;

        for(size_t key : oracle_keys)
        {
            oracle_vis_scores[key] = 0;
        }
        vis.resize(map_data->n_points);
        for(size_t i=0;i<map_data->n_points;i++)
        {
            vis[i] = custom_prob(i);
        }
    }

    double getBmax() const override
    {
        double max_obs = 0;
        for(auto pair : oracle_vis_scores)
        {
             size_t obs = map_data->frame_obs[pair.first].size();
             if (obs > max_obs)
                max_obs = obs;
        }
        return max_obs;
    }

    double getBmin() const override
    {
        return 0;
    }

    
    double gain(size_t key) override
    {
        const auto &map_obs = map_data->mapKeyToFrameObs(key);
        double sum = 0;
        double pvis = vis[key];

        for (size_t frame_key : map_obs)
        {
            auto iter = oracle_vis_scores.find(frame_key);
            if (iter != oracle_vis_scores.end())
            {
                if (iter->second < b_sat )
                    sum += std::min<double>(pvis,b_sat-iter->second);
            }
        }
        return sum;
    }

    void update(size_t key) override
    {

        const auto &map_obs = map_data->mapKeyToFrameObs(key);
        double sum = 0;
        double pvis = vis[key];
        
        for (size_t frame_key : map_obs)
        {
            auto iter = oracle_vis_scores.find(frame_key);
            if (iter != oracle_vis_scores.end())
            {
                iter->second+=pvis;
            }
        }
    }

    void print() const override
    {
        size_t min_key = 0;
        double min_mu = 0;
        for(auto pair : oracle_vis_scores)
        {
            if (pair.second < min_mu)
            {
                min_mu = pair.second;
                min_key = pair.first;
            }
        }
        
        std::cout << "min obs " << min_mu << " @ " << min_key << std::endl;
        std::cout << "bsat" << b_sat << std::endl;
    }


    double eval() const override
    {
        double sum = 0;
        for(auto pair : oracle_vis_scores)
        {
            sum+=std::min<double>(b_sat,pair.second);
        }
        return sum;
    }


    protected:

        void reset() override
            {
                for(auto &pair : oracle_vis_scores)
                {
                    pair.second = 0;
                }

            }

        std::map<size_t,double> oracle_vis_scores;
        std::shared_ptr<MapDataAlias> map_data;
        std::vector<double> vis;
        
};




class OracleVisCover : public LoopProb
{
    public:

    OracleVisCover(std::shared_ptr<MapDataAlias> map_data_ptr, const std::vector<size_t>& oracle_keys) 
    {
        init(map_data_ptr,oracle_keys);
    }

    double custom_prob(size_t key) override
    {
        return map_data->getMappointPvis(key,0,0);
    }

};




#endif