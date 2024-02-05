#include "OdeInvertPendulum.h"

// NOTES
// 2. for the III initial condition, the current controller cannot work becuase of ZERO w.


Eigen::VectorXd OdeInvertPendulum::get_dxdt(const double &t, 
                                           const Eigen::VectorXd &x, 
                                           const Eigen::VectorXd &u, 
                                           bool is_process_noise,
                                           const int mode){
    const int size = x.size();
    double x1 = x[0]; double x2 = x[1];
    double u1 = u[0];

    // Automaton (q \in [0,3])
    if(q != -1){
        // Discrete Transition
        q = transition_q(x);
        u1 = get_u(x);
        // std::cout << "u: " << u1 << "\n";
    }

    // std::cout << x1 << "\t" << x2 << "\t" << x3 << "\t" <<
    //              u1 << "\t" << u2 << "\t" << u3 << "\n";
    Eigen::VectorXd dxdt(size);
    dxdt[0] = x2;
    dxdt[1] = sin(x1) - u1*cos(x1);
    return dxdt;
}


bool OdeInvertPendulum::is_out_of_domain(const Eigen::VectorXd& s){

    // [TEMP] write state and control trajectory of hybrid automaton
    Eigen::VectorXd row(4);
    row << wrapToPi(s[0]), s[1], get_u(s), q;
    sol_hybrid_control.push_back(row);
    //

    if(x_min.size() == 0){
        return false;
    }
    const int size_x = s.size();
    for(int i=0; i<size_x; i++){
        if(s[i] < x_min[i] || s[i] > x_max[i]){
            return true;
        }
    }
    return false;
}


bool OdeInvertPendulum::is_in_unsafe(const Eigen::VectorXd& s){
    double safe_buffer = 0.1;
    if(unsafe_circle_regions.radius.empty()){
        return false;
    }
    const int dimension = unsafe_circle_regions.dimension;
    for(int i=0; i<unsafe_circle_regions.radius.size(); i++){
        Eigen::VectorXd cent = unsafe_circle_regions.center[i].head(dimension); // [km]
        double dist_to_cent = (s.head(dimension) - cent).norm();
        // std::cout << dist_to_cent << " " << unsafe_circle_regions.radius[i] + safe_buffer << "\n";
        if(dist_to_cent <= unsafe_circle_regions.radius[i] + safe_buffer){
            std::cout << "[DEBUG] is in unsafe\n";
            return true;
        }
    }
    return false;
}


bool OdeInvertPendulum::is_goals(const Eigen::VectorXd& s, const std::vector<Eigen::VectorXd> goals){
    if(goals.empty()){
        return false;
    }

    const int size = goals.size();
    for(int i=0; i<size; i++){
        if(s[i] < goals[i][0] || s[i] > goals[i][1]){
            return false;
        }
    }
    return true;
}


double OdeInvertPendulum::get_u(const Eigen::VectorXd& x){
    // std::cout << "[DEBUG] q: " << q << "\n";
    double x1 = x[0]; double x2 = x[1];
    if(q == 0){
        if(std::abs(wrapToPi(x1)) == M_PI && std::abs(x2) < 1e-1){
            std::cout << "DEBUG: use SWING\n";
            return 1;
        }
        return -x2*cos(x1)/(1+std::abs(x2));
    }
    if(q == 1){
        return 0;
    }
    if(q == 2){
        return x2*cos(x1)/(1+std::abs(x2));
    }
    if(q == 3){
        return (2*x2 + wrapToPi(x1) + sin(x1))/(cos(x1));
    }
    // std::cout << "[ERROR] undefined q for computing u\n";
    return 0.0;
}


double OdeInvertPendulum::get_E(const Eigen::VectorXd& x){
    return 0.5*x[1]*x[1] + cos(x[0]) - 1;
}


bool OdeInvertPendulum::is_delta_neighbor(const Eigen::VectorXd& x){
    if( sqrt(wrapToPi(x[0])*wrapToPi(x[0]) + x[1]*x[1]) <= delta){
        return true;
    }
    return false;
}


double OdeInvertPendulum::wrapToPi(double angle) {
    angle = fmod(angle + M_PI, 2 * M_PI); // Shift angle range to [0, 2*pi) and take modulo
    if (angle < 0)
        angle += 2 * M_PI; // Ensure positive angle
    return angle - M_PI; // Shift back to [-pi, pi)
}


int OdeInvertPendulum::transition_q(const Eigen::VectorXd& x){
    double E = get_E(x);
    // std::cout << "[DEBUG] E: " << E << ", q: " << q << "\t" << wrapToPi(x[0]) << " " << x[1] << "\n";
    if(q == 0){
        if(E <= epsilon && E >= -epsilon){
            return 1;
        }
        return 0;
    }
    if(q == 1){
        if(E < - epsilon){
            return 0;
        }
        if(E > epsilon){
            return 2;
        }
        if(is_delta_neighbor(x)){
            return 3;
        }
        return 1;
    }
    if(q == 2){
        if(E <= epsilon && E >= -epsilon){
            return 1;
        }
        return 2;
    }
    if(q == 3){
        return 3;
    }
    std::cout << "[ERROR] No discrete transition define\n";
    return -1;
}


// void OdeInvertPendulum::set_domain(const Eigen::VectorXd x_min_values, const Eigen::VectorXd x_max_values,
//                                  const Eigen::VectorXd u_min_values, const Eigen::VectorXd u_max_values){
//     x_min = x_min_values;
//     x_max = x_max_values;
//     u_min = u_min_values;
//     u_max = u_max_values;
// }


// void OdeMarineVessel::set_unsafecircles(const double dimension, 
//                                      const std::vector<Eigen::VectorXd> center, 
//                                      const std::vector<double> radius){
//     unsafe_circle_regions.dimension = dimension;
//     unsafe_circle_regions.center = center;
//     unsafe_circle_regions.radius = radius;
// }


// std::vector<Eigen::VectorXd> OdeMarineVessel::output_unsafecircles(){
//     std::vector<Eigen::VectorXd> data;
//     if(unsafe_circle_regions.radius.empty()){
//         return data;
//     }

//     for(int i=0; i<unsafe_circle_regions.radius.size(); i++){
//         Eigen::VectorXd row(3);
//         row << unsafe_circle_regions.radius[i], 
//                unsafe_circle_regions.center[i][0], 
//                unsafe_circle_regions.center[i][1], 
//         data.push_back(row);
//     }
//     return data;
// }


// std::vector<Eigen::VectorXd> OdeMarineVessel::output_unsafecircles(){
//     std::vector<Eigen::VectorXd> data;

//     if(unsafe_circle_regions.radius.empty()){
//         return data;
//     }

//     for(int i=0; i<unsafe_circle_regions.radius.size(); i++){
//         Eigen::VectorXd row(unsafe_circle_regions.dimension+1);
//         row << unsafe_circle_regions.radius[i], 
//                unsafe_circle_regions.center[i][0], 
//                unsafe_circle_regions.center[i][1], 
//         data.push_back(row);
//     }
//     return data;
// }