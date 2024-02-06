#include<iostream>
#include <cmath>
#include "OdeVirtual.h"
#include "OdeInvertPendulumSimple.h"
#include "OdeSolver.h"
#include "PlannerVirtual.h"
#include "SetRRT.h"
#include "helperfunctions.h"
// #include "ConfigMarineVessel.h"


void test(){

    // Define ode
    OdeInvertPendulumSimple ode_invert_pendulum_simple("InvertPendulumSimple");
    const int size_x = 4;
    const int size_u = 2;

    // Define start state
    Eigen::VectorXd x_start(size_x); 
    x_start << M_PI, 0.0, 0.0, 0.0; // (I)

    // Define domain
    Eigen::VectorXd x_min(size_x); x_min << -std::numeric_limits<double>::infinity(), -10.0, -2.0, -10.0;
    Eigen::VectorXd x_max(size_x); x_max <<  std::numeric_limits<double>::infinity(),  10.0,  2.0,  10.0;
    Eigen::VectorXd u_min(size_u); u_min << -1.0, 0.1; 
    Eigen::VectorXd u_max(size_u); u_max << 1.0, 10.0;
    ode_invert_pendulum_simple.set_domain(x_min, x_max, u_min, u_max);
    // ode_marine_vessel.set_unsafecircles(2, CONFIG_MARINE_VESSEL::get_random_unsafe_centers(15, 0, 10), 
    //                                        CONFIG_MARINE_VESSEL::get_random_unsafe_raidus(15, 0.25, 0.5));
    // // Eigen::VectorXd process_mean(3); process_mean << CONFIG_SOLARSAIL::get_process_mean();
    // // Eigen::MatrixXd process_cov(3,3); process_cov << CONFIG_SOLARSAIL::get_process_cov();
    // // ode_solarsail.set_process_noise(process_mean, process_cov);
    OdeVirtual* ode_pointer = &ode_invert_pendulum_simple;

    // // Construct Unsafe for Visualization
    // std::string unsafe_file = "outputs/" + ode_pointer->ode_name + "_unsafe.csv";
    // HELPER::write_traj_to_csv(ode_marine_vessel.output_unsafecircles(), unsafe_file);

    // Define ode solver
    const double time_integration = 1e-2;
    OdeSolver ode_solver(time_integration);
    ode_solver.link_ode_pointer(ode_pointer);

    // Solving ode
    // Eigen::VectorXd nominal_control(size_u-1); nominal_control << 0.2;
    // double time_nominal = 3.0;
    // std::vector<Eigen::VectorXd> empoty_goal_states;
    // const bool use_process_noise = false;
    // const bool check_unsafe = false;
    // std::vector<Eigen::VectorXd> nom_traj = ode_solver.solver_runge_kutta(
    //                                     x_start, 
    //                                     nominal_control, 
    //                                     ode_solver.time_integration, 
    //                                     time_nominal, empoty_goal_states, use_process_noise, check_unsafe);

    // // Writing ode solution
    // std::string nom_traj_file = "outputs/" + ode_pointer->ode_name + "_traj.csv";
    // HELPER::write_traj_to_csv(nom_traj, nom_traj_file);

    // NOTE: HEAD>>>
    // Define planner
    SetRRT setrrt_planner("SetRRT");
    setrrt_planner.link_ode_solver_pointer(&ode_solver);
    PlannerVirtual* planner_pointer = &setrrt_planner;

    // Define Goals
    std::vector<Eigen::VectorXd> x_goals; // (theta, w, x, v)
    Eigen::VectorXd x1_goal(4); x1_goal << 0.0, 0.0, 0.0, 0.0; x_goals.push_back(x1_goal);
    Eigen::VectorXd delta_to_goal(4); delta_to_goal << 0.05, 0.05, 0.1, 0.1;
    ode_invert_pendulum_simple.delta_to_goal = delta_to_goal;
    std::string goals_file = "outputs/" + ode_pointer->ode_name + "_goals.csv";
    HELPER::write_traj_to_csv(x_goals, goals_file);
    
    // Plan
    std::vector<Eigen::VectorXd> sol = planner_pointer->plan(x_start, x_goals);
    HELPER::log_trajectory(sol);
    std::string sol_file = "outputs/" + planner_pointer->planner_name + "_" 
                            + ode_pointer->ode_name + "_sol.csv";
    HELPER::write_traj_to_csv(sol, sol_file);

    // Construct Controlled Trajectory
    std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol); // HELPER::log_trajectory(traj);
    std::string traj_file = "outputs/" + planner_pointer->planner_name + "_" 
                            + ode_pointer->ode_name + "_traj.csv";
    HELPER::write_traj_to_csv(traj, traj_file);
}


int main(){
    std::cout << "[test invertpendulumsimple]\n";

    std::cout << "[in docker develop env]\n";

    test();

    return 0;
}