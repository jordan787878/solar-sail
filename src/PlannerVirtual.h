#pragma once

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <chrono>
#include "OdeSolver.h"

class PlannerVirtual{
	public:
        OdeSolver* ode_solver_pointer;
        std::string planner_name;
        int size_x;
        int size_u;
        bool is_success;
        double cost_threshold = std::numeric_limits<double>::infinity();
        double plan_time_max  = std::numeric_limits<double>::infinity();

        PlannerVirtual(std::string name){planner_name = name;}
		virtual std::vector<Eigen::VectorXd> plan(const Eigen::VectorXd& x_init, const std::vector<Eigen::VectorXd> x_goals) = 0;
        virtual std::vector<Eigen::VectorXd> construct_trajectory(const std::vector<Eigen::VectorXd>& solution, 
                                                                  const std::vector<Eigen::VectorXd> x_goals={},
                                                                  bool is_process_noise=false) = 0;
        virtual double get_cost() = 0;
        void set_cost_threshold(double);
        void set_plan_time_max(double);
        bool exceed_plan_time_max(std::chrono::time_point<std::chrono::high_resolution_clock>);

    protected:
        void print_eigen_vector(const Eigen::VectorXd);
        int getRandomInt(const int); 
        int getRandomIntBounded(const int, const int);
};