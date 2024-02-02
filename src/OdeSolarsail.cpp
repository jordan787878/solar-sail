#include "OdeSolarsail.h"


Eigen::VectorXd OdeSolarsail::get_dxdt(const double &t, const Eigen::VectorXd &x, const Eigen::VectorXd &u, bool is_process_noise){
    double r1 = x[0]; double r2 = x[1]; double r3 = x[2];
    double v1 = x[3]; double v2 = x[4]; double v3 = x[5];

    double r = pow(r1*r1 + r2*r2 + r3*r3, 0.5);

    std::vector<double> a_SRP;
    double u1 = u[0]; double u2 = u[1];
    a_SRP.push_back( g0*cos(u1)*( C1*pow(cos(u1),2) + C2*cos(u1) + C3 ) );
    a_SRP.push_back( g0*cos(u1)*( -(C1*cos(u1)+C2)*sin(u1)*sin(u2) ) );
    a_SRP.push_back( g0*cos(u1)*( -(C1*cos(u1)+C2)*sin(u1)*cos(u2) ) );
    double ax = a_SRP[0]/unit_acc;
    double ay = a_SRP[1]/unit_acc;
    double az = a_SRP[2]/unit_acc; //std::cout << "a_SRP: " << ax << " " << ay << " " << az <<  "\n";

    const int size = x.size();
    Eigen::VectorXd dxdt(size);
    dxdt[0] = v1;
    dxdt[1] = v2;
    dxdt[2] = v3;
    dxdt[3] =  2*v2 + 3*r1 - r1/pow(r,3) + ax;
    dxdt[4] = -2*v1 - r2/pow(r,3) + ay;
    dxdt[5] = -r3 - r3/pow(r,3) + az;

    if(is_process_noise){
        // std::cout << "use process noise\n";
        Eigen::VectorXd process_noise = generateRandomVector(process_mean, process_covariance);
        for(int i=0; i<process_noise.size(); i++){
            dxdt[i+3] = dxdt[i+3] + process_noise[i]; // std::cout << process_noise[i] << " ";
        } // std::cout << "\n";
    }

    return dxdt;
}


bool OdeSolarsail::is_out_of_domain(const Eigen::VectorXd& s){
    const int size_x = s.size();
    for(int i=0; i<size_x; i++){
        if(s[i] < x_min[i] || s[i] > x_max[i]){
            return true;
        }
    }
    return false;
}


bool OdeSolarsail::is_in_unsafe(const Eigen::VectorXd& s){
    double safe_buffer = 0.2; // [km]
    if(unsafe_circle_regions.radius.empty()){
        return false;
    }
    else{
        for(int i=0; i<unsafe_circle_regions.radius.size(); i++){
            Eigen::VectorXd cent = unsafe_circle_regions.center[i].head(3); // [km]
            double dist_to_cent = (unit_length*(s.head(3)) - cent).norm();
            // std::cout << dist_to_cent << " " << unsafe_circle_regions.radius[i] + safe_buffer << "\n";
            if(dist_to_cent <= unsafe_circle_regions.radius[i] + safe_buffer){
                std::cout << "[DEBUG] is in unsafe\n";
                return true;
            }
        }
        return false;
    }
}


bool OdeSolarsail::is_goals(const Eigen::VectorXd& s, const std::vector<Eigen::VectorXd> goals){
    for(const auto& goal : goals){
        Eigen::VectorXd delta_pos = s.head(3) - goal.head(3);
        double distance = unit_length * delta_pos.norm();
        if(distance < r_ast * 2){ std::cout << "distance: " << distance << "\n"; }
        if(distance <= r_ast){
            return true;
        }
    }
    return false;
}


void OdeSolarsail::set_params(std::vector<double> params){
    C1 = params[0];
    C2 = params[1];
    C3 = params[2];
    g0 = params[3];
    unit_time = params[4];
    unit_length = params[5];
    unit_vel = unit_length/unit_time;
    unit_acc = unit_vel/unit_time;
}


void OdeSolarsail::set_domain(const Eigen::VectorXd x_min_values, const Eigen::VectorXd x_max_values,
                              const Eigen::VectorXd u_min_values, const Eigen::VectorXd u_max_values){
    x_min = x_min_values;
    x_max = x_max_values;
    u_min = u_min_values;
    u_max = u_max_values;
}


void OdeSolarsail::set_unsafecircles(const double dimension, 
                                     const std::vector<Eigen::VectorXd> center, 
                                     const std::vector<double> radius){
    unsafe_circle_regions.dimension = dimension;
    unsafe_circle_regions.center = center;
    unsafe_circle_regions.radius = radius;
}


void OdeSolarsail::set_r_ast(const double value){
    r_ast = value;
}


void OdeSolarsail::set_process_noise(const Eigen::VectorXd mean, const Eigen::MatrixXd cov){
    process_mean = mean;
    process_covariance = cov;
}


std::vector<Eigen::VectorXd> OdeSolarsail::output_unsafecircles(){
    std::vector<Eigen::VectorXd> data;
    if(!unsafe_circle_regions.radius.empty()){
        for(int i=0; i<unsafe_circle_regions.radius.size(); i++){
            Eigen::VectorXd row(4);
            row << unsafe_circle_regions.radius[i], 
                   unsafe_circle_regions.center[i][0], 
                   unsafe_circle_regions.center[i][1], 
                   unsafe_circle_regions.center[i][2], 
            data.push_back(row);
        }
        return data;
    }
    return data;
}