#include "OdeSolver.h"

void OdeSolver::link_ode_pointer(OdeVirtual* ptr){
    ode_pointer = ptr;
}


std::vector<Eigen::VectorXd> OdeSolver::solver_runge_kutta(
										const Eigen::VectorXd x0, 
										const Eigen::VectorXd u_zero_order_hold,
                                        const double t_integration,
										double& t_end,
                                        const std::vector<Eigen::VectorXd>& x_goals){

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

        w1 = ode_pointer->get_dxdt(t, x, u_zero_order_hold);
        w2 = ode_pointer->get_dxdt(t, x + 0.5*t_integration*w1, u_zero_order_hold);
        w3 = ode_pointer->get_dxdt(t, x + 0.5*t_integration*w2, u_zero_order_hold);
        w4 = ode_pointer->get_dxdt(t, x + 1.0*t_integration*w3, u_zero_order_hold);

        x = x + t_integration*(w1 + 2*w2 + 2*w3 + w4)/6;
        t = t + t_integration;
        k = k + 1;
        x_traj.push_back(x);
    }

    return x_traj;                                                       
}


// void ODESolver::link_ODE(GeneralODE ode_to_link){
//     ode_function = ode_to_link;
// }

// std::vector<Eigen::VectorXd> OdeSolver::RungeKutta(const Eigen::VectorXd x0, 
// 												   const Eigen::VectorXd x_dot,
//                                                    const double t_integration,
// 												   const double t_end)
// {

//     std::vector<Eigen::VectorXd> X_traj;
//     const int x_size = x0.size();
//     double t = 0;
//     int k = 0;
//     Eigen::VectorXd x(x_size);

//     x = x0;
//     X_traj.push_back(x);

//     // while (t < t_end) {
//     //     // if(isTerminal(x)){
//     //     //     return state_traj;
//     //     // }
//     //     // // init u at time k
//     //     // std::vector<double> uk;
//     //     // // a. get u control from u.csv data
//     //     // if(u_data.size() > 0){
//     //     //     // std::cout << "use data control\n";
//     //     //     if(k < u_data.size()){
//     //     //         uk = u_data[k];
//     //     //     }
//     //     //     // exceeding the u.csv data size
//     //     //     else{
//     //     //         return state_traj;
//     //     //     }
//     //     // }
//     //     // // b. prespecified u signal
//     //     // else{
//     //     //     // std::cout << "use sample control\n";
//     //     //     uk = u;
//     //     // }
//     //     // // std::cout << k << " " << t << "\n"; // log_vector(uk);

//         k = k + 1;

//         Eigen::VectorXd w1(x_size);
//         Eigen::VectorXd w2(x_size);
//         Eigen::VectorXd w3(x_size);
//         Eigen::VectorXd w4(x_size);

//         w1 = ode_function(t, x, uk);
//         w2 = get_dxdt(t, x + 0.5*h*w1, uk);
//         w3 = get_dxdt(t, x + 0.5*h*w2, uk);
//         w4 = get_dxdt(t, x + 1.0*h*w3, uk);

//     //     // x = x + h*(w1 + 2*w2 + 2*w3 + w4)/6;
//     //     // t = t + h;
//     //     // state_traj.push_back(x);
//     // }

//     return X_traj;
// }