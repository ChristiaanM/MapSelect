/*
 * File: PathUtils.h
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


#ifndef PATH_UTILS_H
#define PATH_UTILS_H

#include <string>

namespace mselect
{

    std::string strpath_join(const std::string &p0, const std::string &p1)
    {
        if (p0.empty() || p1.empty())
            return p0 + p1;
#ifdef _WIN32
        return p0 + '\\' + p1;
#else
        return p0 + '/' + p1;
#endif
    }

    size_t filename_start_pos(const std::string &path)
    {
        size_t pos = path.find_last_of("/\\");
        if (pos == std::string::npos)
            return 0;
        return pos + 1;
    }

    size_t ext_dot_pos(const std::string &path)
    {
        size_t pos = path.rfind('.');
        if (pos == std::string::npos)
            return path.size();
        if (path.size() > pos + 1 && (path[pos + 1] == '/' || path[pos + 1] == '\\'))
            return path.size();
        if (pos > 0 && path[pos - 1] == '.')
            return path.size();
        return pos;
    }

    std::string strpath_directory(const std::string &path)
    {
        size_t pos = path.find_last_of("/\\");
        if (pos == std::string::npos)
            return "";
        return path.substr(0, pos);
    }

    inline std::string substring2(const std::string &path, std::size_t start, std::size_t end = std::string::npos)
    {
        return path.substr(start, std::max<size_t>(end - start, 0));
    }

    std::string strpath_filename(const std::string &path)
    {
        return substring2(path, filename_start_pos(path));
    }

    std::string strpath_filename_noext(const std::string &path)
    {
        return substring2(path, filename_start_pos(path), ext_dot_pos(path));
    }

    std::string strpath_noext(const std::string &path)
    {
        return substring2(path, 0, ext_dot_pos(path));
    }

    std::string strpath_ext(const std::string &path)
    {
        return substring2(path, ext_dot_pos(path) + 1);
    }

    bool caseless_str_equal(const std::string &s0, const std::string &s1)
    {
        if (s0.size() != s1.size())
            return false;
        for (size_t i = 0; i < s0.size(); i++)
            if (std::toupper(s0[i]) != std::toupper(s1[i]))
                return false;
        return true;
    }

}

#endif