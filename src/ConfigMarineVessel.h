#pragma once

#include <iostream>
#include <vector>
#include "helperfunctions.h"

namespace CONFIG_MARINE_VESSEL{

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

    std::vector<Eigen::VectorXd> get_unsafe_centers(){
        std::vector<Eigen::VectorXd> centers;
        Eigen::VectorXd center(2);
        center << 2.5, 1.5; centers.push_back(center);
        center << 5, 5; centers.push_back(center);
        return centers;
    }

    std::vector<double> get_unsafe_raidus(){
        std::vector<double> radius;
        double rad1 = 1.0; radius.push_back(rad1);
        rad1 = 1.0; radius.push_back(rad1);
        return radius;
    }

    std::vector<Eigen::VectorXd> get_random_unsafe_centers(const int n, const double pos_min, const double pos_max){
        std::vector<Eigen::VectorXd> centers;
        Eigen::VectorXd center(2);
        for(int i=0; i<n; i++){
            while(true){
                center << HELPER::getRandomDouble(pos_min, pos_max), 
                          HELPER::getRandomDouble(pos_min, pos_max);

                if(center[1] <= 6){
                    break;
                }
            }
            centers.push_back(center);
        }
        return centers;
    }

    std::vector<double> get_random_unsafe_raidus(const int n, const double r_min, const double r_max){
        std::vector<double> radius;
        double rad1;
        for(int i=0; i<n; i++){
            rad1 = HELPER::getRandomDouble(r_min, r_max);
            radius.push_back(rad1);
        }
        return radius;
    }

}