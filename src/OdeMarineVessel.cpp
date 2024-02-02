#include "OdeMarineVessel.h"

Eigen::VectorXd OdeMarineVessel::get_dxdt(const double &t, 
                                           const Eigen::VectorXd &x, 
                                           const Eigen::VectorXd &u, 
                                           bool is_process_noise,
                                           const int mode){
    const int size = x.size();
    double x1 = x[0]; double x2 = x[1]; double x3 = x[2];
    double u1 = u[0]; double u2 = u[1]; double u3 = u[2];

    // std::cout << x1 << "\t" << x2 << "\t" << x3 << "\t" <<
    //              u1 << "\t" << u2 << "\t" << u3 << "\n";
    Eigen::VectorXd dxdt(size);
    dxdt[0] = u1*cos(x3) - u2*sin(x3);
    dxdt[1] = u1*sin(x3) + u2*cos(x3);
    dxdt[2] = u3;
    return dxdt;
}

void OdeMarineVessel::set_domain(const Eigen::VectorXd x_min_values, const Eigen::VectorXd x_max_values,
                                 const Eigen::VectorXd u_min_values, const Eigen::VectorXd u_max_values){
    x_min = x_min_values;
    x_max = x_max_values;
    u_min = u_min_values;
    u_max = u_max_values;
}


bool OdeMarineVessel::is_out_of_domain(const Eigen::VectorXd& s){
    const int size_x = s.size();
    for(int i=0; i<size_x; i++){
        if(s[i] < x_min[i] || s[i] > x_max[i]){
            return true;
        }
    }
    return false;
}


bool OdeMarineVessel::is_in_unsafe(const Eigen::VectorXd& s){
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


bool OdeMarineVessel::is_goals(const Eigen::VectorXd& s, const std::vector<Eigen::VectorXd> goals){
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


void OdeMarineVessel::set_unsafecircles(const double dimension, 
                                     const std::vector<Eigen::VectorXd> center, 
                                     const std::vector<double> radius){
    unsafe_circle_regions.dimension = dimension;
    unsafe_circle_regions.center = center;
    unsafe_circle_regions.radius = radius;
}


std::vector<Eigen::VectorXd> OdeMarineVessel::output_unsafecircles(){
    std::vector<Eigen::VectorXd> data;
    if(unsafe_circle_regions.radius.empty()){
        return data;
    }

    for(int i=0; i<unsafe_circle_regions.radius.size(); i++){
        Eigen::VectorXd row(3);
        row << unsafe_circle_regions.radius[i], 
               unsafe_circle_regions.center[i][0], 
               unsafe_circle_regions.center[i][1], 
        data.push_back(row);
    }
    return data;
}


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