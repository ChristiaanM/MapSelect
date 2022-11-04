
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



#ifndef TIMING_H
#define TIMING_H

#include <bits/stdc++.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <sstream>
#include <ctime>

namespace timing
{
    typedef std::chrono::_V2::system_clock::time_point TimepointType;


    inline std::string time_string(const char * c_str,size_t time)
    {
        std::ostringstream sstream;
        sstream << std::left << std::setw(30) << c_str << std::setw(30) << std::setprecision(9) << std::right << time   ;
        return sstream.str();
    }

    inline TimepointType now()
    {
        return std::chrono::high_resolution_clock::now();
    }

    inline size_t time_difference(TimepointType t0,TimepointType t1 )
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
    }


}




#define TIC(label) auto ___timestart_##label =  timing::now()
#define TOC(label) auto ___timeend_##label = timing::now()
#define TIME_RUNNING(label) timing::time_difference(___timestart_##label,timing::now())
#define TIME(label) timing::time_difference(___timestart_##label,___timeend_##label)
#define TIME_STRING(label) timing::time_string(#label,TIME(label))
#define TIME_PRINT(label) std::cout << TIME_STRING(label) <<std::endl



#endif