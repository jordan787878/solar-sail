#pragma once

#include "OdeVirtual.h"
#include <Eigen/Dense>

class OdeSimpleExample : public OdeVirtual {
public:
    OdeSimpleExample(std::string name) : OdeVirtual(name){}

    Eigen::VectorXd get_dxdt(const double &t, 
                             const Eigen::VectorXd &x, 
                             const Eigen::VectorXd &u, 
                             bool is_process_noise=false,
                             const int mode=0) override;

    bool is_out_of_domain(const Eigen::VectorXd& s) override;

    bool is_in_unsafe(const Eigen::VectorXd& s) override;

    bool is_goals(const Eigen::VectorXd& s, const std::vector<Eigen::VectorXd> goals) override;
};