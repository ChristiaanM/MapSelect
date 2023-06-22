/*
 * File: SaturatedSetFunction.h
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


#ifndef MAPSELECT_SATURATED_SET_FUNCTION_H
#define MAPSELECT_SATURATED_SET_FUNCTION_H

#include "mapselect/functions/SetFunction.h"

namespace mselect
{

    /*! @brief An abstract class for functions with an adjustable paramater b for functions with the form f(S)=\min(g(S),b_{sat})
     */
    class SaturatedSetFunction : public virtual IncrementalSetFunction<double>
    {
    public:
        virtual double getBmax() = 0;
        virtual double getBmin() = 0;
    public:
        void changeB(double b_new)
        {
            b_sat = b_new;
        }
        double getB()
        {
            return b_sat;
        }

    protected:
        double b_sat = std::numeric_limits<double>::max();
    };
    

    /*! @brief A class that takes two saturated functions, chooses a static value for the saturation paramater 
    of the first function, while allowing the adjustment of the second function, or
        f_{combined} = w*f_1(S,b=b1) + f_2(S,b=b)
    */
    template <class F1, class F2>
    class TwinSaturatedSetFunction : public SaturatedSetFunction
    {
    public:
        TwinSaturatedSetFunction(std::shared_ptr<F1> func1, double b1_min, std::shared_ptr<F2> func2, double w = 1.0) : func1(func1),
                                                                                                                             func2(func2),
                                                                                                                             b1_min(b1_min),
                                                                                                                             w(w)
        {
        }

        ~TwinSaturatedSetFunction()
        {
        }

        double getBmin() override
        {
            return func2->getBmin();
        }

        double getBmax() override
        {
            return func2->getBmax();
        }

        double gain(size_t key) override
        {
            return w * func1->gain(key) + func2->gain(key);
        }

        double eval() const override
        {
            return w * func1->eval() + func2->eval();
        }

        void update(size_t key) override
        {
            func1->update(key);
            func2->update(key);
        }

        void print() const override
        {
            func1->print();
            func2->print();
        }

    protected:
        void reset() override
        {
            func1->changeB(b1_min);
            func2->changeB(b_sat);
        }

        double b1_min;
        double w;
        std::shared_ptr<F1> func1;
        std::shared_ptr<F2> func2;
    };



    /*! @brief A class that takes two saturated functions and allows sequential adjustment of the saturation paramaters
    */
    template <class F1, class F2>
    class TwinStagedSetFunction : public SaturatedSetFunction
    {
    public:
        TwinStagedSetFunction(std::shared_ptr<F1> func1, double b1_th, std::shared_ptr<F2> func2) : func1(func1),
                                                                                                    func2(func2),
                                                                                                    b1_th(b1_th),
                                                                                                    b2_min(func2->getBmin())
        {
        }

        ~TwinStagedSetFunction()
        {
        }

        double getBmin() const override
        {
            return func1->getBmin() + func2->getBmin();
        }

        double getBmax() const override
        {
            return b1_th + func2->getBmax();
        }

        double gain(size_t key) override
        {
            double gain1 = func1->gain(key);
            double gain2 = func2->gain(key);
            // std::cout << "gains" << gain1 << " " << gain2 << std::endl;

            return gain1 + gain2;
        }

        double eval () const override
        {
            return func1->eval()+func2->eval();
        }

        void update(size_t key) override
        {
            func1->update(key);
            func2->update(key);
        }

        void print() const override
        {
            func1->print();
            func2->print();
        }

    protected:
        void reset() override
        {
            if (b_sat > b1_th)
            {
                func1->changeB(b1_th);
                func2->changeB(b_sat - b1_th);
            }
            else
            {
                func1->changeB(b_sat);
                func2->changeB(b2_min);
            }
        }

        double b1_th;
        double b2_min;
        std::shared_ptr<F1> func1;
        std::shared_ptr<F2> func2;
    };

}

#endif
