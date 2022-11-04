
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


#ifndef WSEARCH_H
#define WSEARCH_H


#include "mapselect/functions/SaturatedSetFunction.h"

class WSearchPair : public IncrementalSetFunction<double>
{
    typedef std::shared_ptr<EvaluableFunction<double>> FMaxType;
    typedef std::shared_ptr<EvaluableFunction<double>> FCoverType;

    public:
        WSearchPair(FMaxType fm, FCoverType fc): func_m(fm),func_c(fc)
        {
            offset_m = 0;
            offset_c = 0;
            norm_c = 1.0;
            norm_m = 1.0;
        }


       double gain(size_t key) override
       {
           double gain_m=w_m*func_m->gain(key)*norm_m;
           double gain_c=w_c*func_c->gain(key)*norm_c;
           return gain_m+gain_c;
       }
        
        void set_w(double w)
        {
            w_m = std::max<double>(std::min<double>(w,1),0);
            w_c = 1-w_m;
        }

        
        double get_fcval()
        {
            return (func_c->eval()-offset_c)*norm_c;
        } 

        double get_fmval()
        {
            return (func_m->eval() -offset_m)*norm_m;
        }

        void print()
        {
            double normed_m = get_fmval();
            double normed_c = get_fcval();

            std::cout << "w=" << w_m <<  " F_m=" << normed_m  << " F_c=" << normed_c << std::endl;
        }

        void normalise(double m0, double m1, double c0,  double c1)
        {
            norm_m = 1.0/(m1-m0);
            norm_c = 1.0/(c1-c0);
            offset_m =  m0;
            offset_c =  c0;
        }

    FMaxType func_m;
    FCoverType func_c;


    protected:
        void update(size_t key) override
        {
            func_m->add(key,0);
            func_c->add(key,0);
        }

        void reset() override
        {
            func_m->clear();
            func_c->clear();
        }

    private:
        double w_m;
        double offset_m;
        double offset_c;
        double w_c;

        double norm_m;
        double norm_c;
       

};

#endif
