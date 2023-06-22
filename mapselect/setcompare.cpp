/*
 * File: setcompare.cpp
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


#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <algorithm>


std::vector<size_t> load_ids(std::string key_path)
{
    std::vector<size_t> out;
    std::ifstream infile(key_path);
    size_t value;
    while (infile >> value)
        out.push_back(value);
    return out;
}

bool check_keyvectors(std::vector<size_t> kv1, std::vector<size_t> kv2)
{
    // don't use normal == since we want to customise error messages
    if (kv1.size() != kv2.size())
    {
        std::cout << "number of selected keys do not match (" << kv1.size() << "!=" << kv2.size() << ")" << std::endl;
        return false;
    }
    for (size_t i = 0; i < kv1.size(); i++)
        if (kv1[i] != kv2[i])
        {
            std::cout << "selected ID " << kv1[i] << "!=" << kv2[i] << " (at pos " << i << ")" << std::endl;
            return false;
        }

    return true;
}

struct KeyVectComp
{
    size_t set_intersect;
    size_t set_difference;
    size_t set_union;
    size_t seq_match;
    int mismatch_key1 = 0;
    int mismatch_key2 = 0;


    void print()
    {
        std::cout << "union:\t" << set_union << std::endl;
        std::cout << "intersec:\t" << set_intersect << std::endl;
        std::cout << "differance\t" << set_difference << std::endl;
        if (seq_match != set_union)
        {
            std::cout << "mismatch @ " << seq_match << std::endl;
            if (mismatch_key1 ==-1)
                std::cout << "vector1 only contains " << seq_match << " elements." << std::endl;
            else 
                std::cout << "vector1 @ " << seq_match << "=" << mismatch_key1 << std::endl;

            if (mismatch_key2 ==-1)
                std::cout << "vector2 only contains " << seq_match << " elements." << std::endl;
            else 
                std::cout << "vector2 @ " << seq_match << "=" << mismatch_key2 << std::endl;

        }
    }
};

KeyVectComp compareKeyvectors(const std::vector<size_t> &kv1, const std::vector<size_t> &kv2)
{
    std::vector<size_t> sorted1(kv1.begin(), kv1.end());
    std::vector<size_t> sorted2(kv2.begin(), kv2.end());

    std::sort(sorted1.begin(), sorted1.end());
    std::sort(sorted2.begin(), sorted2.end());

    size_t buffer_size = kv1.size()+kv2.size();


    std::vector<size_t> iset(buffer_size), dset(buffer_size), uset(buffer_size);

    auto iiter = std::set_intersection(sorted1.begin(), sorted1.end(), sorted2.begin(), sorted2.end(), iset.begin());

    auto diter = std::set_symmetric_difference(sorted1.begin(), sorted1.end(), sorted2.begin(), sorted2.end(), dset.begin());


    auto uiter = std::set_union(sorted1.begin(), sorted1.end(), sorted2.begin(), sorted2.end(), uset.begin());

    KeyVectComp results;
    results.set_intersect = std::distance(iset.begin(), iiter);
    results.set_difference = std::distance(dset.begin(), diter);
    results.set_union = std::distance(uset.begin(), uiter);


    size_t i = 0;
    while (i < kv1.size() && i < kv2.size())
    {
        if (kv1[i] != kv2[i])
            break;
        i++;
    }

    results.seq_match = i;
    if (i < kv1.size())
        results.mismatch_key1 = kv1[i]; 
    else 
        results.mismatch_key1 = -1;

    if (i < kv2.size())
        results.mismatch_key2 = kv2[i];
    else 
        results.mismatch_key1 = -1;

    
    return results;
}


/*! @file 
This is a simple binary used to compare the selected IDs (stored in .select)
from running  selection algorithms using the ./mselect binary
*/
int main(int argc, char const *argv[])
{
    if(argc != 3)
        return -1;

    std::string s1(argv[1]);
    std::string s2(argv[2]);

    std::cout << "Reading " << s1 << std::endl;
    std::vector<size_t> ids1 = load_ids(s1);
    std::cout << "Reading " << s2 << std::endl;
    std::vector<size_t> ids2 = load_ids(s2);
    std::cout << "Comparing ..." << std::endl;
    compareKeyvectors(ids1,ids2).print();

    return 0;
}