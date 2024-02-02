#pragma once

#include "OdeVirtual.h"
#include <Eigen/Dense>
#include "UnsafeCircle.h"

class OdeSimpleExample : public OdeVirtual {
public:
    // Unsafe
    UnsafeCircle unsafe_circle_regions;

    OdeSimpleExample(std::string name) : OdeVirtual(name){}

    Eigen::VectorXd get_dxdt(const double &t, const Eigen::VectorXd &x, const Eigen::VectorXd &u, bool is_process_noise = false) override;

    bool is_out_of_domain(const Eigen::VectorXd& s) override;

    bool is_in_unsafe(const Eigen::VectorXd& s) override;

    bool is_goals(const Eigen::VectorXd& s, const std::vector<Eigen::VectorXd> goals) override;
};