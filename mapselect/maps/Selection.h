
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

#ifndef SELECTION_H
#define SELECTION_H

#include <iostream>
#include <vector>
#include <unordered_set>

//using namespace std;



class Selector
{
    public: 
        virtual bool select_mappoint(size_t map_id) const = 0; 
        virtual bool select_keyframe(size_t keyframe_id) const = 0;
        virtual bool select_feature(size_t keyframe_id, size_t map_id) const = 0;
};

class DummySelector : public Selector
{
    public: 
        DummySelector() {}

        bool select_mappoint(size_t map_id) const 
        {
            return true;
        } 
        bool select_keyframe(size_t keyframe_id) const 
        {
            return true;
        };

        bool select_feature(size_t keyframe_id, size_t map_id) const 
        {
            return true;
        }

};


class EssentialSelector : public Selector
{
    public: 
        EssentialSelector(size_t last_frame): last_frame(last_frame)
        {}
        bool select_mappoint(size_t map_id) const 
        {
            return true;
        } 
        bool select_keyframe(size_t keyframe_id) const 
        {
            return true;
        };

        bool select_feature(size_t keyframe_id, size_t map_id) const 
        {
            return  map_id; //keyframe_id == last_frame ||
        }

    protected: 
        size_t last_frame;
};


class MappointSelector : public EssentialSelector
{

    public: 
        MappointSelector(const std::vector<size_t> &ids, size_t last_frame) : EssentialSelector(last_frame), hashset(ids.begin(),ids.end())
        {
        }

        bool select_mappoint(size_t map_id) const 
        {
            return hashset.find(map_id) != hashset.end();
        } 
        bool select_feature(size_t keyframe_id, size_t map_id) const 
        {
            return (hashset.find(map_id) != hashset.end());
        }


    protected:
        std::unordered_set<size_t> hashset;

};


class MappointDescriptorSelector : public MappointSelector 
{

    public: 
         MappointDescriptorSelector(const std::vector<size_t> &ids, size_t last_frame) : MappointSelector(ids,last_frame)
        {
        }

        bool select_mappoint(size_t map_id) const 
        {
            return true;
        } 
};



#endif