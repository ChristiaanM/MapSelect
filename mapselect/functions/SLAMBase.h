
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


#ifndef SLAMBASE_H
#define SLAMBASE_H



#include "mapselect/functions/SetFunction.h"
#include "mapselect/functions/SaturatedSetFunction.h"
#include "mapselect/slam/SLAMGraphGTSAM.h"

#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>


double inline NegEntropyGaussian(double logdet_K, size_t dim)
{
    double c0 =0.5*dim+0.5*log(2*M_PI);
    return 0.5*logdet_K-c0;  
}

class SLAMEntropy: public SaturatedSetFunction, public EvaluableFunction<double>
{
    public:
        SLAMEntropy(std::shared_ptr<MapDataAlias> map_data, GTSAMGraphSettings gsettings = GTSAMGraphSettings());
       

        double gain(size_t key);
        
        double eval() const override;
        
        double getBmin() const override;
        double getBmax() const override;
        void print() const override;
        
            
    protected:
        void update(size_t key);
        void reset()  override;

        //std::map<size_t,double> cached_scores;
        
        gtsam::GaussianFactorGraph::shared_ptr graph;
        std::vector<gtsam::GaussianFactor::shared_ptr> landmark_factors;
        
        size_t factors;
        size_t base_factors;

        double min_value;
        double max_value;

        double prev_value;
        bool outdated;

        size_t dim;

        std::shared_ptr<MapDataAlias> map_data;

};





class SLAMEntropyDet: public SLAMEntropy 
{
     public : 
        SLAMEntropyDet(std::shared_ptr<MapDataAlias> map_data, GTSAMGraphSettings gsettings  = GTSAMGraphSettings());

        double gain(size_t key);
        double eval() const override;
        Eigen::MatrixXd getMarginalCov(std::vector<size_t> keys);
        void updateCov();
         
    protected:


        void update(size_t key);

        void reset()  override;
        void updateK(size_t key);
        void releaseK();
        void resetK();

        std::vector<Eigen::MatrixXd> landmark_A;
        std::vector<std::vector<size_t>> landmark_keys;

        double origin_info =1e6;
        double prior_info = 1e-4;

        Eigen::MatrixXd K;
        Eigen::MatrixXd C;

        bool cov_computed;


};




#endif
