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

    // Define domain
    Eigen::VectorXd x_min(6); x_min << -1, -1, -1, -50, -50, -50;
    Eigen::VectorXd x_max(6); x_max << 1, 1, 1, 50, 50, 50;
    Eigen::VectorXd u_min(3); u_min << -0.5*M_PI, 0.0, 0.001;
    Eigen::VectorXd u_max(3); u_max << 0.5*M_PI, 2*M_PI, 0.1;
    ode_solarsail.set_domain(x_min, x_max, u_min, u_max);

    // Define start
    Eigen::VectorXd x_start(6);
    for(int i=0; i<6; i++){x_start[i] = CONFIG_SOLARSAIL::get_state_init(env)[i];}

    // Define goal
    std::vector<Eigen::VectorXd> x_goals;
    Eigen::VectorXd x_goal(6);  x_goal << 0, 0, 0, 0, 0, 0;
    x_goals.push_back(x_goal);

    // Define unsafe
    std::cout << "setting unsafe...\n";
    const int unsafe_regions = 10;
    std::vector<double> unsafe_circle_radius = CONFIG_SOLARSAIL::get_random_unsafe_raidus_km(unsafe_regions, 0.3, 1.0);
    std::vector<Eigen::VectorXd> unsafe_circle_center = CONFIG_SOLARSAIL::get_random_unsafe_centers_km(
        unsafe_regions, -4.0, 4.0, 
        unsafe_circle_radius, 
        x_start, 
        x_goals,
        ode_solarsail.unit_length);
    ode_solarsail.set_unsafecircles(3, unsafe_circle_center, unsafe_circle_radius);
    std::cout << "complet unsafe setting\n";

    ode_solarsail.set_r_ast(0.25);
    Eigen::VectorXd process_mean(3); process_mean << CONFIG_SOLARSAIL::get_process_mean();
    Eigen::MatrixXd process_cov(3,3); process_cov << CONFIG_SOLARSAIL::get_process_cov();
    ode_solarsail.set_process_noise(process_mean, process_cov);
    OdeVirtual* ode_pointer = &ode_solarsail;

    // Construct Unsafe for Visualization
    std::string unsafe_file = "outputs/" + ode_pointer->ode_name + "_env" + std::to_string(env) + "_unsafe.csv";
    HELPER::write_traj_to_csv(ode_solarsail.output_unsafecircles(), unsafe_file);

    // Define ode solver
    OdeSolver ode_solver(1e-4);
    ode_solver.link_ode_pointer(ode_pointer);

    // Define planner
    SetRRT setrrt_planner("SetRRT");
    setrrt_planner.link_ode_solver_pointer(&ode_solver);
    PlannerVirtual* planner_pointer = &setrrt_planner;

    // test ode_solver works in planner
    Eigen::VectorXd empty_control = Eigen::VectorXd::Zero(3);
    std::vector<Eigen::VectorXd> empoty_goal_states;
    double time_nominal = 0.5;
    const bool use_process_noise = false;
    const bool check_unsafe = false;
    std::vector<Eigen::VectorXd> nom_traj = planner_pointer->ode_solver_pointer->solver_runge_kutta(
                                                x_start, 
                                                empty_control, 
                                                planner_pointer->ode_solver_pointer->time_integration, 
                                                time_nominal, 
                                                empoty_goal_states, 
                                                use_process_noise, 
                                                check_unsafe);
    std::string nom_traj_file = "outputs/" + ode_pointer->ode_name + "_env" + std::to_string(env) + "_traj.csv";
    HELPER::write_traj_to_csv(nom_traj, nom_traj_file);

    // Plan
    std::vector<Eigen::VectorXd> sol = planner_pointer->plan(x_start, x_goals);
    HELPER::log_trajectory(sol);
    std::string sol_file = "outputs/" + planner_pointer->planner_name + "_" + ode_pointer->ode_name + "_env" + std::to_string(env) + "_sol.csv";
    HELPER::write_traj_to_csv(sol, sol_file);

    // Construct Controlled Trajectory
    std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol, x_goals); // HELPER::log_trajectory(traj);
    std::string traj_file = "outputs/" + planner_pointer->planner_name + "_" + ode_pointer->ode_name + "_env" + std::to_string(env) + "_traj.csv";
    HELPER::write_traj_to_csv(traj, traj_file);

    // Construct Controlled Trajectory subject to Process noise
    std::vector<Eigen::VectorXd> traj_noise = planner_pointer->construct_trajectory(sol, x_goals, true);
    std::string traj_noise_file = "outputs/" + planner_pointer->planner_name + "_" + ode_pointer->ode_name + "_env" + std::to_string(env) + "_trajnoise.csv";
    HELPER::write_traj_to_csv(traj_noise, traj_noise_file);
}


int main(){
    std::cout << "[test SetRRT]\n";

    test();

    return 0;
}