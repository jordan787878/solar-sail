#include<iostream>
#include <cmath>
#include "OdeVirtual.h"
#include "OdeSimpleExample.h"
#include "OdeSolver.h"
#include "helperfunctions.h"


void test(){

    // Define ode
    OdeSimpleExample ode_simple_example("exp_decay");
    OdeVirtual* ode_pointer = &ode_simple_example;

    // Define ode solver
    OdeSolver ode_solver(1e-4);
    ode_solver.link_ode_pointer(ode_pointer);

    // Solving
    Eigen::VectorXd x_start(1);
    for(int i=0; i<1; i++){x_start[i] = 1;}
    double time_nominal = 3.0;
    std::vector<Eigen::VectorXd> nom_traj = ode_solver.solver_runge_kutta(
                                                x_start, 
                                                Eigen::VectorXd::Zero(1), 
                                                ode_solver.time_integration, 
                                                time_nominal, {}, false, false);

    // Writing
    std::string nom_traj_file = "outputs/" + ode_pointer->ode_name + "_traj.csv";
    HELPER::write_traj_to_csv(nom_traj, nom_traj_file);
}


int main(){
    std::cout << "[test simpleode]\n";

    test();

    return 0;
}