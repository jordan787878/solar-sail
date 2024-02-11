#pragma once

#include "OdeVirtual.h"
#include <Eigen/Dense>
#include "UnsafeCircle.h"

class OdeInvertPendulum : public OdeVirtual {
public:
    // Q: discrete state
    int q = -1;
    double epsilon;
    double delta;
    std::vector<Eigen::VectorXd> sol_hybrid_control;

    // Unsafe
    UnsafeCircle unsafe_circle_regions;

    OdeInvertPendulum(std::string name) : OdeVirtual(name){}

    Eigen::VectorXd get_dxdt(const double &t, 
                             const Eigen::VectorXd &x, 
                             const Eigen::VectorXd &u, 
                             bool is_process_noise=false,
                             const int mode=0) override;

    Eigen::VectorXd state_post_process(const Eigen::VectorXd& s) override;

    bool is_out_of_domain(const Eigen::VectorXd& s) override;

    bool is_in_unsafe(const Eigen::VectorXd& s) override;

    bool is_goals(const Eigen::VectorXd& s, const std::vector<Eigen::VectorXd> goals) override;

    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> get_linear_dynamics_matrices(
                                        const Eigen::VectorXd& x, 
                                        const Eigen::VectorXd& u, 
                                        double delta_time) override;

    double get_E(const Eigen::VectorXd& x);

    bool is_delta_neighbor(const Eigen::VectorXd& x);

    // void set_domain(const Eigen::VectorXd x_min_values, const Eigen::VectorXd x_max_values,
    //                 const Eigen::VectorXd u_min_values, const Eigen::VectorXd u_max_values);

    // void set_unsafecircles(const double dimension, const std::vector<Eigen::VectorXd> center, const std::vector<double> radius);

    // std::vector<Eigen::VectorXd> output_unsafecircles();

private:
    double get_u(const Eigen::VectorXd& x);
    int transition_q(const Eigen::VectorXd& x);
    double wrapToPi(double angle);                   
};