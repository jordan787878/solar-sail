#include "OdeInvertPendulumSimple.h"

// NOTES
// 2. for the III initial condition, the current controller cannot work becuase of ZERO w.


Eigen::VectorXd OdeInvertPendulumSimple::get_dxdt(const double &t, 
                                           const Eigen::VectorXd &x, 
                                           const Eigen::VectorXd &u, 
                                           bool is_process_noise,
                                           const int mode){
    const int size = x.size();
    double x1 = x[0]; double x2 = x[1]; // theta, and theta_dot
    double x3 = x[2]; double x4 = x[3]; // x, and x_dot
    double u1 = u[0];

    // std::cout << x1 << "\t" << x2 << "\t" << x3 << "\t" <<
    //              u1 << "\t" << u2 << "\t" << u3 << "\n";
    Eigen::VectorXd dxdt(size);
    dxdt[0] = x2;
    dxdt[1] = sin(x1) - u1*cos(x1);
    dxdt[2] = x4;
    dxdt[3] = u1;
    return dxdt;
}


bool OdeInvertPendulumSimple::is_out_of_domain(const Eigen::VectorXd& s){
    if(x_min.size() == 0){
        return false;
    }
    const int size_x = s.size();
    // std::cout << "s: " << s[0] << " " << s[1] << " " << s[2] << " " << s[3] << "\n";
    for(int i=0; i<size_x; i++){
        if(s[i] < x_min[i] || s[i] > x_max[i]){
            return true;
        }
    }
    return false;
}


bool OdeInvertPendulumSimple::is_in_unsafe(const Eigen::VectorXd& s){
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


// HEAD>>>
bool OdeInvertPendulumSimple::is_goals(const Eigen::VectorXd& s, const std::vector<Eigen::VectorXd> goals){
    if(goals.empty()){
        return false;
    }

    const int size = goals.size();
    for(int i=0; i<size; i++){
        double x1 = wrapToPi(s[0]);
        double x2 = s[1];
        double x3 = s[2];
        double x4 = s[3];

        // std::cout << std::abs(x1-goals[i][0]) << " " << std::abs(x2-goals[i][1]) << " "
        //           << std::abs(x3-goals[i][2]) << " " << std::abs(x4-goals[i][3]) << "\n";

        if(std::abs(x1-goals[i][0]) > delta_to_goal[0]){
            return false;
        }
        if(std::abs(x2-goals[i][1]) > delta_to_goal[1]){
            return false;
        }
        if(std::abs(x3-goals[i][2]) > delta_to_goal[2]){
            return false;
        }
        if(std::abs(x4-goals[i][3]) > delta_to_goal[3]){
            return false;
        }
    }
    return true;
}


void OdeInvertPendulumSimple::set_domain(const Eigen::VectorXd x_min_values, const Eigen::VectorXd x_max_values,
                                   const Eigen::VectorXd u_min_values, const Eigen::VectorXd u_max_values){
    x_min = x_min_values;
    x_max = x_max_values;
    u_min = u_min_values;
    u_max = u_max_values;
}


bool OdeInvertPendulumSimple::is_delta_neighbor(const Eigen::VectorXd& x, const double delta){
    double norm = sqrt( wrapToPi(x[0])*wrapToPi(x[0]) + x[1]*x[1] );
    // std::cout << "[DEBUG] " << norm << " " << delta << "\n";
    if(norm <= delta){
        return true;
    }
    return false;
}


double OdeInvertPendulumSimple::wrapToPi(double angle){
    angle = fmod(angle + M_PI, 2 * M_PI); // Shift angle range to [0, 2*pi) and take modulo
    if (angle < 0)
        angle += 2 * M_PI; // Ensure positive angle
    return angle - M_PI; // Shift back to [-pi, pi)
}


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