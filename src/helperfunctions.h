#pragma once

#include<iostream>
#include<random>
#include "Keepout.h"
#include "MyODE.h"
#include "Eigen/Dense"
#include <fstream>
#include <vector>
#include <string>

namespace HELPER{

int getRandomInt(int min, int max) {
    // Create a random number generator engine
    std::random_device rd;  // Random seed from hardware entropy
    std::mt19937 gen(rd());  // Mersenne Twister engine

    // Define a distribution for integer values between min and max (inclusive)
    std::uniform_int_distribution<int> dis(min, max);

    // Generate and return a random integer
    return dis(gen);
}

std::vector<KEEPOUT::Keepout> genearte_keepouts(const int N){
    std::vector<KEEPOUT::Keepout> keepouts;
    for(int i=0; i<N; i++){
        KEEPOUT::Keepout keepout;
        Eigen::VectorXd c(3);

        if(i == 0){
            c[0] = 2000.0; c[1] = 0.0; c[2] = 0.0;
            keepout.center = c;
            keepout.radius = 1800;
        }
        if(i == 1){
            c[0] = -2000.0; c[1] = 0.0; c[2] = 0.0;
            keepout.center = c;
            keepout.radius = 1800;
        }

        keepouts.push_back(keepout);
    }
    return keepouts;
}

std::vector<KEEPOUT::Keepout> genearte_random_keepouts(const int N){
    std::vector<KEEPOUT::Keepout> keepouts;
    int r_ast = 250;
    int r_min = 500;
    int r_max = 5000;
    int p_min = -10000;
    int p_max =  10000;
    for(int i=0; i<N; i++){
        KEEPOUT::Keepout keepout;
        Eigen::VectorXd c(3);
        while(true){
            int radius = getRandomInt(r_min, r_max);
            Eigen::VectorXd center(3);
            center << getRandomInt(p_min, p_max), getRandomInt(p_min, p_max), getRandomInt(p_min, p_max);
            if(center.norm() > r_ast + radius){
                keepout.center = center;
                keepout.radius = radius;
                break;
            }
        }
        keepouts.push_back(keepout);

    }
    std::cout << "finish generating " << N << " random keepouts\n";
    return keepouts;
}

Eigen::VectorXd generate_random_initial_states(const Eigen::VectorXd x_start){
    MyODE ode;
    ode.init_params();

    const double tstep_Traj = 0.0001;

    std::vector<double> u_control;
	const double u1 = 0;
	const double u2 = 0;
	u_control.push_back(u1);
	u_control.push_back(u2);

    Eigen::VectorXd x_new(6); 

    while(true){
        double t_end = 0.001 * getRandomInt(1,500);
        std::vector<Eigen::VectorXd> x_traj;
	    x_traj = ode.rungeKutta(0, x_start, u_control, tstep_Traj, t_end);
        x_new = x_traj.back();
        if(!ode.isTerminal(x_new)){
            break;
        }
    }


    ode.log_vector(x_start);
    ode.log_vector(x_new);
    return x_new;
}

void writeValueToCSV(const std::string& filename, const std::vector<std::string>& valueNames, const std::vector<int>& values) {
    std::ofstream outputFile(filename);

    // Write header with value names
    for (const auto& name : valueNames) {
        outputFile << name << ",";
    }
    outputFile << "\n";

    // Write data
    for (size_t i = 0; i < values.size(); ++i) {
        outputFile << values[i];
        if (i < values.size() - 1) {
            outputFile << ",";
        }
    }
    outputFile << "\n";

    outputFile.close();
}


}