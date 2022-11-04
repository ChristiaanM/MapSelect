

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

#ifndef SLAMCHOLMOD_H
#define SLAMCHOLMOD_H


#include "mapselect/functions/SLAMBase.h"
#include "mapselect/slam/SLAMGraphGTSAM.h"



extern "C" {

#include "cholmod.h"

}


#include <Eigen/CholmodSupport>

typedef Eigen::SparseMatrix<double, Eigen::ColMajor> CSCMatrix;
typedef Eigen::SparseMatrix<double, Eigen::RowMajor> CSRMatrix;
typedef Eigen::MappedSparseMatrix<double,Eigen::ColMajor,int> CSCMap;


class CholmodFactor: public Eigen::CholmodSupernodalLLT<CSCMatrix,Eigen::UpLoType::Upper>
{
    public:
        CholmodFactor() : Eigen::CholmodSupernodalLLT<CSCMatrix,Eigen::UpLoType::Upper>()
        {
            //factor_copy = nullptr;
        }

        void updown(bool update, CSCMatrix &c, bool permute=true);
        
        double determinantDelta(bool update, CSCMatrix &c, bool permute=true);
        
        ~CholmodFactor();
        
        bool isSuper();
        
        void analyzeAsym(CSCMatrix& A);
             
    //protected:
    //cholmod_factor * factor_copy;
};



class SLAMEntropyCholmod : public SLAMEntropyDet
{

public:
    SLAMEntropyCholmod(std::shared_ptr<MapDataAlias> map_data, GTSAMGraphSettings gsettings = GTSAMGraphSettings());

    double gain(size_t key) override;
    double eval() const override;


protected:
    void update(size_t key) override;
    void reset() override;


    CSCMatrix landmarkUpdate(size_t key);

    CholmodFactor cholmod_factor;
    std::vector<CSCMatrix> landmark_csc;
    CSCMatrix base_csc;
    size_t evals;
};

#endif
