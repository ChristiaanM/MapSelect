/*
 * File: Tolerance.h
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



#ifndef TOLERANCE_H
#define TOLERANCE_H

namespace mselect
{

    inline bool is_close(double a, double b, double rtol = 1e-9, double atol = 0.0)
    {
        return fabs(a - b) <= std::max<double>(rtol * std::max<double>(fabs(a), fabs(b)), atol);
    }

    bool is_close_relative(double x, double y, double eps = 1e-6)
    {
        return fabs(x - y) <= eps * std::max<double>(fabs(x), fabs(y));
    }

}

#endif