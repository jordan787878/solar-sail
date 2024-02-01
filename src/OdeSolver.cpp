#include "OdeSolver.h"

void OdeSolver::link_ode_pointer(OdeVirtual* ptr){
    ode_pointer = ptr;
}


std::vector<Eigen::VectorXd> OdeSolver::solver_runge_kutta(
										const Eigen::VectorXd x0, 
										const Eigen::VectorXd u_zero_order_hold,
                                        const double t_integration,
										double& t_end,
                                        const std::vector<Eigen::VectorXd>& x_goals,
                                        bool is_process_noise){

    std::vector<Eigen::VectorXd> x_traj;
    const int size_x = x0.size();
    double t = 0;
    int k = 0;
    Eigen::VectorXd x(size_x);

    x = x0;
    x_traj.push_back(x);

    while (t < t_end) {
        if(ode_pointer->is_out_of_domain(x)){
            x_traj.clear();
            return x_traj;
        }
        if(ode_pointer->is_goals(x,x_goals)){
            std::cout << "[DEBUG] update time from " << t_end << " to " << t << "\n";
            t_end = t; // update the end time
            return x_traj;
        }

        Eigen::VectorXd w1(size_x);
        Eigen::VectorXd w2(size_x);
        Eigen::VectorXd w3(size_x);
        Eigen::VectorXd w4(size_x);

        w1 = ode_pointer->get_dxdt(t, x, u_zero_order_hold, is_process_noise);
        w2 = ode_pointer->get_dxdt(t, x + 0.5*t_integration*w1, u_zero_order_hold, is_process_noise);
        w3 = ode_pointer->get_dxdt(t, x + 0.5*t_integration*w2, u_zero_order_hold, is_process_noise);
        w4 = ode_pointer->get_dxdt(t, x + 1.0*t_integration*w3, u_zero_order_hold, is_process_noise);

        x = x + t_integration*(w1 + 2*w2 + 2*w3 + w4)/6;
        t = t + t_integration;
        k = k + 1;
        x_traj.push_back(x);
    }

    return x_traj;                                                       
}