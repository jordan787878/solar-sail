#pragma once

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <vector>
#include <map>

class OdeVirtual{
	public:
		std::string ode_name;
    	Eigen::VectorXd x_min;
    	Eigen::VectorXd x_max;
    	Eigen::VectorXd u_min;
    	Eigen::VectorXd u_max;

		OdeVirtual(std::string name){ode_name = name;}
		virtual Eigen::VectorXd get_dxdt(const double &t, const Eigen::VectorXd &x, const Eigen::VectorXd &u) = 0;
		virtual bool is_out_of_domain(const Eigen::VectorXd& s) = 0;
		virtual bool is_goals(const Eigen::VectorXd& s, const std::vector<Eigen::VectorXd> goals) = 0;
};
	// 	void set_W_intenstiy(const double W);
	// 	double get_W_intensity();
	// private:
	// 	Eigen::VectorXd generate_process_noise(const int dimension);
// };