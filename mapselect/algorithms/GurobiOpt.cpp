/*
 * File: GurobiOpt.cpp
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


#include "mapselect/algorithms/GurobiOpt.h"

#include <math.h>

#include "gurobi_c++.h"

#include "mapselect/utils/Timing.h"

namespace mselect
{

    size_t computeMaxObs(const MapDataAlias &data)
    {
        size_t N = data.getMapSize();
        if (N == 0)
            return 0;

        size_t max = data.getMappointObs(0).size();
        for (size_t i = 1; i < N; i++)
        {
            size_t tmp = data.getMappointObs(i).size();
            if (tmp > max)
                max = tmp;
        }
        std::cout << "max obs" << max << std::endl;

        return max;
    }

    std::vector<size_t> GurobiIPCover(const MapDataAlias &data, const GurobiSettings &settings)
    {
        std::vector<size_t> vals;

        try
        {

            size_t N_map = data.getMapSize();
            size_t N_frames = data.getFrameSize();

            GRBEnv env = GRBEnv();
            GRBModel model = GRBModel(env);
            model.set(GRB_DoubleParam_MIPGap, settings.mip_gap);
            model.set(GRB_IntParam_Threads, 1);

            size_t max_obs = computeMaxObs(data);
            model.set(GRB_IntParam_Method, GRB_METHOD_DUAL); //

            GRBVar *x = model.addVars(N_map, GRB_BINARY);
            GRBVar *zeta = nullptr;

            if (settings.lambda)
                zeta = model.addVars(N_frames, GRB_INTEGER);

            for (size_t i = 0; i < N_frames; i++)
            {

                auto obs_keys = data.frame_obs[i];
                size_t N_obs = obs_keys.size();
                size_t N_arr = N_obs;
                GRBVar *obs_vars = new GRBVar[N_obs + 1];

                for (size_t j = 0; j < N_obs; j++)
                {
                    obs_vars[j] = x[obs_keys[j]];
                }

                if (zeta)
                {
                    obs_vars[N_obs] = zeta[i];
                    N_arr = N_obs + 1;
                }
                GRBLinExpr obs_expr;
                obs_expr.addTerms(NULL, obs_vars, N_arr);
                model.addConstr(obs_expr >= std::min<size_t>(settings.b, N_obs));
                if (settings.b_min)
                {
                    GRBLinExpr obs_expr2;
                    obs_expr2.addTerms(NULL, obs_vars, N_obs);
                    model.addConstr(obs_expr2 >= std::min<size_t>(settings.b_min, N_obs));
                }

                delete obs_vars;
            }

            if (settings.forced_keys && settings.forced_keys->size())
                for (size_t key : *settings.forced_keys)
                {
                    x[key].set(GRB_DoubleAttr_LB, 1);
                }

            GRBLinExpr num_expr;
            num_expr.addTerms(NULL, x, N_map);
            if (settings.N)
                model.addConstr(num_expr == settings.N);

            double *q = new double[N_map];
            for (size_t i = 0; i < N_map; i++)
            {
                q[i] = (max_obs - data.getMappointObs(i).size());
            }

            double *l = new double[N_frames];
            for (size_t i = 0; i < N_frames; i++)
            {
                l[i] = settings.lambda;
            }

            /*
            if (settings.quad_penalty)
            {
                TIC(quad_vals);
                size_t QUAD_TERMS = 0;
                GRBQuadExpr obj_expr;
                obj_expr.addTerms(q, x, N_map);
                if (zeta)
                    obj_expr.addTerms(l, zeta, N_frames);
                for (size_t i = 0; i < N_frames; i++)
                {

                    auto obs_keys = data.frame_obs[i];
                    auto obs_kp = data.frame_keypoints[i];
                    size_t obs_N = obs_keys.size();

                    GRBVar obs_vars[obs_N];
                    for (size_t j = 0; j < obs_N; j++)
                    {
                        obs_vars[j] = x[obs_keys[j]];
                    }

                    for (size_t j = 0; j < obs_N; j++)
                    {
                        auto kp_j = obs_kp[j];
                        // Only add terms in the upper triangle, but with x2 the weight
                        // since Q is symmetric
                        for (size_t k = j + 1; k < obs_N; k++)
                        {
                            auto kp_k = obs_kp[k];
                            double dx = kp_j.px - kp_k.px;
                            double dy = kp_j.py - kp_k.py;
                            double dist_2 = dx * dx + dy * dy;
                            if (dist_2 < 40 * 40)
                            {
                                double penalty = (40.0 - sqrt(dist_2)) / 40.0 / N_frames;
                                obj_expr.addTerm(penalty, obs_vars[j], obs_vars[k]);
                                QUAD_TERMS++;
                            }
                        }
                    }
                }

                TOC(quad_vals);
                std::cout << "Added " << QUAD_TERMS << " in " << TIME_SECONDS(quad_vals) << " seconds " << std::endl;
                std::cout << "QUAD_VALS " << TIME(quad_vals) << std::endl;
                std::cout << "QUAD TERMS " << QUAD_TERMS << std::endl;
                model.setObjective(obj_expr);
            }*/
            //else
            //{
                GRBLinExpr obj_expr;
                obj_expr.addTerms(q, x, N_map);
                if (zeta)
                    obj_expr.addTerms(l, zeta, N_frames);
                model.setObjective(obj_expr);
            //}

            delete q;
            delete l;

            // model.write("gurobi.lp");
            model.optimize();

            std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;

            if (!settings.return_deleted)
            {
                vals.reserve(settings.N);
                for (size_t i = 0; i < N_map; i++)
                {
                    if (x[i].get(GRB_DoubleAttr_X) > 0.99)
                    {
                        vals.push_back(i);
                    }
                }
            }
            else 
            {
                vals.reserve(N_map-settings.N);
                for (size_t i = 0; i < N_map; i++)
                {
                    if (x[i].get(GRB_DoubleAttr_X) < 0.01)
                    {
                        vals.push_back(i);
                    }
                }
            }
        }
        catch (GRBException e)
        {
            std::cout << "Error code = " << e.getErrorCode() << std::endl;
            std::cout << e.getMessage() << std::endl;
        }
        catch (...)
        {
            std::cout << "Exception during optimization" << std::endl;
        }

        return vals;
    }
}
