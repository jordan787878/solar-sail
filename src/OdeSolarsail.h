#pragma once

#include "OdeVirtual.h"
#include <Eigen/Dense>

class OdeSolarsail : public OdeVirtual {
public:
    // Parameters
    double C1;
    double C2;
    double C3;
    double g0;
    double unit_time;
    double unit_length;
    double unit_vel;
    double unit_acc;
    // Target
    double r_ast;

    OdeSolarsail(std::string name) : OdeVirtual(name){}

    Eigen::VectorXd get_dxdt(const double &t, const Eigen::VectorXd &x, const Eigen::VectorXd &u, bool is_process_noise = false) override;

    bool is_out_of_domain(const Eigen::VectorXd& s) override;

    bool is_goals(const Eigen::VectorXd& s, const std::vector<Eigen::VectorXd> goals) override;

    void set_params(std::vector<double> params);

    void set_domain(const Eigen::VectorXd x_min_values, const Eigen::VectorXd x_max_values,
                    const Eigen::VectorXd u_min_values, const Eigen::VectorXd u_max_values);

    void set_r_ast(const double);

    void set_process_noise(const Eigen::VectorXd mean, const Eigen::MatrixXd cov);
};