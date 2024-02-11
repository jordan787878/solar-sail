#pragma once

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <tuple>

class OdeVirtual{
	public:
		std::string ode_name;
    	Eigen::VectorXd x_min;
    	Eigen::VectorXd x_max;
    	Eigen::VectorXd u_min;
    	Eigen::VectorXd u_max;
		Eigen::VectorXd process_mean;
		Eigen::MatrixXd process_covariance;

		OdeVirtual(std::string name){ode_name = name;}
		virtual Eigen::VectorXd get_dxdt(const double &t, 
										 const Eigen::VectorXd &x, 
										 const Eigen::VectorXd &u, 
										 bool is_process_noise=false,
										 const int mode=0) = 0;
		virtual Eigen::VectorXd state_post_process(const Eigen::VectorXd& s) = 0;
		virtual bool is_out_of_domain(const Eigen::VectorXd& s) = 0;
		virtual bool is_in_unsafe(const Eigen::VectorXd& s) = 0;
		virtual bool is_goals(const Eigen::VectorXd& s, const std::vector<Eigen::VectorXd> goals) = 0;
		virtual std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> get_linear_dynamics_matrices(
               										const Eigen::VectorXd& x, 
													const Eigen::VectorXd& u, 
													double delta_time) = 0;

	protected:
		Eigen::VectorXd generateRandomVector(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance);
};
	// 	void set_W_intenstiy(const double W);
	// 	double get_W_intensity();
	// private:
	// 	Eigen::VectorXd generate_process_noise(const int dimension);
// };