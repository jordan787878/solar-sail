#pragma once

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include "OdeSolver.h"

class PlannerVirtual{
	public:
        OdeSolver* ode_solver_pointer;
        std::string planner_name;
        int size_x;
        int size_u;

        PlannerVirtual(std::string name){planner_name = name;}
		virtual std::vector<Eigen::VectorXd> plan(const Eigen::VectorXd& x_init, const std::vector<Eigen::VectorXd> x_finals) = 0;
        virtual std::vector<Eigen::VectorXd> construct_trajectory(const std::vector<Eigen::VectorXd>& solution, bool is_process_noise=false) = 0;

    protected:
        void print_eigen_vector(const Eigen::VectorXd);
        double getRandomDouble(const double, const double, const int digit=3);
        int getRandomInt(const int); 
};