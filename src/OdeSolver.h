#pragma once

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <vector>
#include "OdeVirtual.h"

class OdeSolver{
	public:
		OdeVirtual* ode_pointer;
		double time_integration;

		OdeSolver(double time_integration_step){time_integration = time_integration_step;}

		void link_ode_pointer(OdeVirtual*);


		std::vector<Eigen::VectorXd> solver_runge_kutta(
										const Eigen::VectorXd x0, 
										const Eigen::VectorXd u_zero_order_hold,
                                        const double t_integration,
										double& t_end,
                                        const std::vector<Eigen::VectorXd>& x_goals = {});
		
        // void validate(const Eigen::VectorXd x0, const std::string ufilename, const std::string filename);
		// void write_traj_csv(const std::vector<Eigen::VectorXd>& data, const std::string filename);
		// void read_u_control(const std::string csv_file);
		// void log_vector(const Eigen::VectorXd v);
		// void log_vector(const std::vector<double> v);
};