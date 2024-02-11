#pragma once

#include <iostream>
#include <vector>
#include "helperfunctions.h"

namespace CONFIG_SOLARSAIL{

    std::vector<double> get_parameters(const int env){
        std::vector<double> params;
        if(env == 1){
            params = {2.0, 0.0, 0.0, 4.2118e-11, 6.51688e+06, 108.4094};
        }
        if(env == 2){
            params = {2.0, 0.0, 0.0, 4.2118e-11, 6.51688e+06, 108.4094};
        }
        if(env == 3){
            params = {2.0, 0.0, 0.0, 3.446e-11, 6.51688e+06, 108.4094};
        }
        if(env == 4){
            params = {2.0, 0.0, 0.0, 3.1244e-11, 6.0048e+06, 52.4430};
        }
        return params;
    }

    std::vector<double> get_state_init(const int env){
        std::vector<double> state_init;
        if(env == 1){
            state_init = {0.127680370, 0, 0.084952359, 0, 1.445775202, 0};
        }
        if(env == 2){
            state_init = {-0.1062, 0, 0.1106, 0, 0.8425, 0};
        }
        if(env == 3){
            state_init = {-0.0359, 0.0, 0.1202, 0.0, 1.9244, 0.0};
        }
        if(env == 4){
            state_init = {0.0019, 0.0488, 0.0, 0.0, 0.0, 4.7163};
        }
        return state_init;
    }

    Eigen::VectorXd get_process_mean(){
        Eigen::VectorXd mean(3);
        mean << 0.0, 0.0, 0.0;
        return mean;
    }

    Eigen::MatrixXd get_process_cov(){
        Eigen::MatrixXd cov(3,3);
        cov << 100.0, 0.0, 0.0,
                0.0, 100.0, 0.0,
                0.0, 0.0, 100.0;
        return cov;
    }

    std::vector<Eigen::VectorXd> get_unsafe_centers_km(){
        std::vector<Eigen::VectorXd> centers;
        Eigen::VectorXd center(3);
        center << 1.5, 0.0, 0.0; centers.push_back(center);
        center << -1.5, 0.0, 0.0; centers.push_back(center);
        center << 0, -1.5, 0.0; centers.push_back(center);
        return centers;
    }

    std::vector<double> get_unsafe_raidus_km(){
        std::vector<double> radius;
        double rad1 = 0.9; radius.push_back(rad1);
        rad1 = 0.9; radius.push_back(rad1);
        rad1 = 0.9; radius.push_back(rad1);
        return radius;
    }

    std::vector<double> get_random_unsafe_raidus_km(const int n, const double r_min, const double r_max){
        std::vector<double> radius;
        double rad1;
        for(int i=0; i<n; i++){
            rad1 = HELPER::getRandomDouble(r_min, r_max);
            radius.push_back(rad1);
        }
        return radius;
    }

    std::vector<Eigen::VectorXd> get_random_unsafe_centers_km(const int n, const double pos_min, const double pos_max, const std::vector<double>& rads,
                                                              const Eigen::VectorXd x_start, const std::vector<Eigen::VectorXd>& x_goals,
                                                              const double unit_length){
        std::vector<Eigen::VectorXd> centers;
        Eigen::VectorXd center(3);
        for(int i=0; i<n; i++){
            const double rad = rads[i];
            while(true){
                bool is_center_appropriate = false;
                center << HELPER::getRandomDouble(pos_min, pos_max), 
                          HELPER::getRandomDouble(pos_min, pos_max), 
                          HELPER::getRandomDouble(pos_min, pos_max);
                for(const auto& goal : x_goals){
                    // [To Implement] if this occurs, don't run the below if check and contine while loop
                    double distance_to_goal = (center.head(3) - unit_length*(goal.head(3))).norm();
                    double distance_to_start = (center.head(3) - unit_length*(x_start.head(3))).norm();
                    std::cout << distance_to_goal << "\t" << distance_to_start << "\t" << rad << "\n";
                    if(distance_to_goal > rad && distance_to_start > rad){
                        is_center_appropriate = true;
                        break;
                    }
                }
                if(is_center_appropriate){
                    break;
                }
            }
            centers.push_back(center);
        }
        return centers;
    }

}