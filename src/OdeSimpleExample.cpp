#include "OdeSimpleExample.h"

Eigen::VectorXd OdeSimpleExample::get_dxdt(const double &t, const Eigen::VectorXd &x, const Eigen::VectorXd &u, bool is_process_noise){
    const int size = x.size();
    Eigen::VectorXd dxdt(size);
    dxdt[0] = -0.5 * x[0];
    return dxdt;
}


bool OdeSimpleExample::is_out_of_domain(const Eigen::VectorXd& s){
    return false;
}


bool OdeSimpleExample::is_in_unsafe(const Eigen::VectorXd& s){
    return false;
}


bool OdeSimpleExample::is_goals(const Eigen::VectorXd& s, const std::vector<Eigen::VectorXd> goals){
    return false;
}