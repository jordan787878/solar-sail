#include<iostream>
#include <cmath>
#include "OdeSolarsail.h"
#include "OdeVirtual.h"
#include "OdeSolver.h"
#include "helperfunctions.h"
#include "PlannerVirtual.h"
#include "SetRRT.h"

#include "ConfigSolarsail.h"

void test(){
    // Define ode
    const int env = 4;
    OdeSolarsail ode_solarsail("Solarsail");
    ode_solarsail.set_params(CONFIG_SOLARSAIL::get_parameters(env));

    Eigen::VectorXd x_min(6); x_min << -1, -1, -1, -50, -50, -50;
    Eigen::VectorXd x_max(6); x_max << 1, 1, 1, 50, 50, 50;
    Eigen::VectorXd u_min(3); u_min << -0.5*M_PI, 0.0, 0.001;
    Eigen::VectorXd u_max(3); u_max << 0.5*M_PI, 2*M_PI, 0.05;
    ode_solarsail.set_domain(x_min, x_max, u_min, u_max);
    ode_solarsail.set_r_ast(0.25);
    Eigen::VectorXd process_mean(3); process_mean << 0.0, 0.0, 0.0;
    Eigen::MatrixXd process_cov(3,3); process_cov << 100.0, 0.0, 0.0,
                                                     0.0, 100.0, 0.0,
                                                     0.0, 0.0, 100.0;
    ode_solarsail.set_process_noise(process_mean, process_cov);
    OdeVirtual* ode_pointer = &ode_solarsail;

    // Define ode solver
    OdeSolver ode_solver(1e-4);
    ode_solver.link_ode_pointer(ode_pointer);

    // Define planner
    SetRRT setrrt_planner("SetRRT");
    setrrt_planner.link_ode_solver_pointer(&ode_solver);
    PlannerVirtual* planner_pointer = &setrrt_planner;

    // test ode_solver works in planner
    Eigen::VectorXd x_start(6);
    for(int i=0; i<6; i++){x_start[i] = CONFIG_SOLARSAIL::get_state_init(env)[i];}
    double time_nominal = 0.5;
    std::vector<Eigen::VectorXd> nom_traj = planner_pointer->ode_solver_pointer->solver_runge_kutta(
                                                x_start, 
                                                Eigen::VectorXd::Zero(2), 
                                                planner_pointer->ode_solver_pointer->time_integration, 
                                                time_nominal);
    std::string nom_traj_file = "outputs/" + ode_pointer->ode_name + "_env" + std::to_string(env) + "_traj.csv";
    HELPER::write_traj_to_csv(nom_traj, nom_traj_file);

    std::vector<Eigen::VectorXd> x_goals;
    Eigen::VectorXd x_goal(6);
    x_goal << 0, 0, 0, 0, 0, 0;
    x_goals.push_back(x_goal);
    std::vector<Eigen::VectorXd> sol = planner_pointer->plan(x_start, x_goals);
    HELPER::log_trajectory(sol);
    std::string sol_file = "outputs/" + planner_pointer->planner_name + "_" + ode_pointer->ode_name + "_env" + std::to_string(env) + "_sol.csv";
    HELPER::write_traj_to_csv(sol, sol_file);

    // Construct Controlled Trajectory
    std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol); // HELPER::log_trajectory(traj);
    std::string traj_file = "outputs/" + planner_pointer->planner_name + "_" + ode_pointer->ode_name + "_env" + std::to_string(env) + "_traj.csv";
    HELPER::write_traj_to_csv(traj, traj_file);

    // Construct Controlled Trajectory subject to Process noise
    std::vector<Eigen::VectorXd> traj_noise = planner_pointer->construct_trajectory(sol, true);
    std::string traj_noise_file = "outputs/" + planner_pointer->planner_name + "_" + ode_pointer->ode_name + "_env" + std::to_string(env) + "_trajnoise.csv";
    HELPER::write_traj_to_csv(traj_noise, traj_noise_file);
}


int main(){
    std::cout << "[test SetRRT]\n";

    test();

    return 0;
}