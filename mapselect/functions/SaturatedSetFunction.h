
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



#ifndef SATURATED_SET_FUNCTION_H
#define SATURATED_SET_FUNCTION_H

class SaturatedSetFunction : public virtual IncrementalSetFunction<double>
{
    public:
        virtual double getBmax() const = 0;
        virtual double getBmin() const = 0;
        virtual void print() const = 0;

    public:
        void changeB(double b_new)
        {
            b_sat = b_new;
            clear();
        }

    protected:
        double b_sat = 0;
 
};

template<class F>
class SaturationFunctionWrapper : public SaturatedSetFunction
{
    public:
        SaturationFunctionWrapper(std::shared_ptr<F> func, double b_max, double b_min =0 ):
            func_base(func),
            b_min(b_min),
            b_max(b_max)
        {
            reset();
        }

        ~SaturationFunctionWrapper()
        {

        }

        double getBmin() const override
        {
            return b_min;
        }

        double getBmax() const override
        {
            return b_max;
        }

        double gain(size_t key)  override
        {
            return func_copy->gain(key);
        }

        void update(size_t key)  override
        {
            func_copy->update(key);
        }

        void print() const override
        {
            std::cout << "Saturated Function" << std::endl << "b_sat" << b_sat << std::endl;
        }
        


        protected:
            void reset()  override
            {
                func_copy = std::make_shared<F>(*func_base);
            }

            std::shared_ptr<F> func_copy;
            std::shared_ptr<F> func_base;
            double b_min, b_max; 

};


template<class F1, class F2>
class DualSaturatedSetFunction : public SaturatedSetFunction
{
    public:
        DualSaturatedSetFunction(std::shared_ptr<F1> func1, double b1_min, std::shared_ptr<F2> func2, double lambda = 1.0):
        func1(func1),
        func2(func2),
        b1_min(b1_min),
        lambda(lambda)
        {

        }

        ~DualSaturatedSetFunction()
        {

        }

        double getBmin() const  override
        {
            return func2->getBmin();
        }

        double getBmax() const override
        {
            return func2->getBmax();
        }

        double gain(size_t key)  override
        {
             return lambda*func1->gain(key) + func2->gain(key);
        }

        void update(size_t key)  override
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
        void reset()  override
        {
            func1->changeB(b1_min);
            func2->changeB(b_sat);
        }


        double b1_min;
        double lambda;
        std::shared_ptr<F1> func1;
        std::shared_ptr<F2> func2;

};


template<class F1, class F2>
class DualStagedSetFunction : public SaturatedSetFunction
{
     public:
        DualStagedSetFunction(std::shared_ptr<F1> func1, double b1_th, std::shared_ptr<F2> func2):
        func1(func1),
        func2(func2),
        b1_th(b1_th),
        b2_min(func2->getBmin())
        {
             
        }

        ~DualStagedSetFunction()
        {
        
        }

        double getBmin() const override 
        {
            return func1->getBmin()+func2->getBmin();
        }

        double getBmax() const override
        {
            return b1_th+func2->getBmax();
        }

        double gain(size_t key)  override
        {
            double gain1= func1->gain(key);
            double gain2 = func2->gain(key);
            //std::cout << "gains" << gain1 << " " << gain2 << std::endl;

            return  gain1+gain2; 
        }

        void update(size_t key)  override
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
        void reset()  override
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










#endif
