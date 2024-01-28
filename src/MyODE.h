#pragma once

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <vector>

class MyODE{
	public:
		double g0;
		double C1;
		double C2;
		double C3;
		double unit_acc;
		double unit_length;
		double W_intensity;

		std::vector< std::vector<double> > u_data;

		void init_params();
		void set_W_intenstiy(const double W);
		double get_W_intensity();
		Eigen::VectorXd get_dxdt(double t, Eigen::VectorXd x, const std::vector<double> u);
		std::vector<double> get_a_SRP(double u1, double u2);
		bool isTerminal(const Eigen::VectorXd& s);
		std::vector<Eigen::VectorXd> rungeKutta(const double t0, 
												const Eigen::VectorXd x0, 
												const std::vector<double> u, 
                                                const double h, 
												const double t_end);
		void validate(const Eigen::VectorXd x0, const std::string ufilename, const std::string filename);

		// helper funcitons
		void write_traj_csv(const std::vector<Eigen::VectorXd>& data, const std::string filename);
		void read_u_control(const std::string csv_file);
		void log_vector(const Eigen::VectorXd v);
		void log_vector(const std::vector<double> v);
		std::vector<double> sphere_to_cartesian(double r, double theta, double phi);

	private:
		Eigen::VectorXd generate_process_noise(const int dimension);
};
