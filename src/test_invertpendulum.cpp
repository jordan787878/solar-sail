#include<iostream>
#include <cmath>
#include "OdeVirtual.h"
#include "OdeInvertPendulum.h"
#include "OdeSolver.h"
#include "PlannerVirtual.h"
#include "SetRRT.h"
#include "helperfunctions.h"
// #include "ConfigMarineVessel.h"


void test(){

    // Define ode
    OdeInvertPendulum ode_invert_pendulum("InvertPendulum");
    const int size_x = 2;
    const int size_u = 2;

    // Define start state
    Eigen::VectorXd x_start(size_x); 
    // x_start << M_PI, 1.0;
    // x_start << M_PI/2, 0.0; // (I)
    // x_start << M_PI/12, 0.6;   // (II)
    x_start << M_PI, 0.0;      // (III)

    // Define discrete state of Automaton
    ode_invert_pendulum.epsilon = 0.005;
    ode_invert_pendulum.delta = 0.1;
    double E_start = ode_invert_pendulum.get_E(x_start);
    std::cout << "E_start: " << E_start << "\n";
    if(E_start < -ode_invert_pendulum.epsilon){
        ode_invert_pendulum.q = 0;
    }
    if(E_start <= ode_invert_pendulum.epsilon &&
       E_start >= -ode_invert_pendulum.epsilon){
        ode_invert_pendulum.q = 1;
    }
    if(E_start > ode_invert_pendulum.epsilon){
        ode_invert_pendulum.q = 2;
    }
    if(ode_invert_pendulum.is_delta_neighbor(x_start)){
        ode_invert_pendulum.q = 3;
    }
    std::cout << "q_start: " << ode_invert_pendulum.q << "\n";

    // Define domain
    // Eigen::VectorXd x_min(size_x); x_min << 0.0, 0.0, -M_PI;
    // Eigen::VectorXd x_max(size_x); x_max << 10.0, 6.5, M_PI;
    // Eigen::VectorXd u_min(size_u); u_min << 0.0, -0.00, -0.1, 10.0; 
    // Eigen::VectorXd u_max(size_u); u_max << 0.18, 0.00,  0.1, 100.0;
    // ode_marine_vessel.set_domain(x_min, x_max, u_min, u_max);
    // ode_marine_vessel.set_unsafecircles(2, CONFIG_MARINE_VESSEL::get_random_unsafe_centers(15, 0, 10), 
    //                                        CONFIG_MARINE_VESSEL::get_random_unsafe_raidus(15, 0.25, 0.5));
    // // Eigen::VectorXd process_mean(3); process_mean << CONFIG_SOLARSAIL::get_process_mean();
    // // Eigen::MatrixXd process_cov(3,3); process_cov << CONFIG_SOLARSAIL::get_process_cov();
    // // ode_solarsail.set_process_noise(process_mean, process_cov);
    OdeVirtual* ode_pointer = &ode_invert_pendulum;

    // // Construct Unsafe for Visualization
    // std::string unsafe_file = "outputs/" + ode_pointer->ode_name + "_unsafe.csv";
    // HELPER::write_traj_to_csv(ode_marine_vessel.output_unsafecircles(), unsafe_file);

    // Define ode solver
    const double time_integration = 1e-2;
    OdeSolver ode_solver(time_integration);
    ode_solver.link_ode_pointer(ode_pointer);

    // Solving ode
    Eigen::VectorXd nominal_control(size_u-1); nominal_control << 0.0;
    double time_nominal = 30.0;
    std::vector<Eigen::VectorXd> empoty_goal_states;
    const bool use_process_noise = false;
    const bool check_unsafe = false;
    std::vector<Eigen::VectorXd> nom_traj = ode_solver.solver_runge_kutta(
                                                x_start, 
                                                nominal_control, 
                                                ode_solver.time_integration, 
                                                time_nominal, empoty_goal_states, use_process_noise, check_unsafe);

    // Writing ode solution
    std::string nom_traj_file = "outputs/" + ode_pointer->ode_name + "_traj.csv";
    HELPER::write_traj_to_csv(ode_invert_pendulum.sol_hybrid_control, nom_traj_file);

    // // Define planner
    // SetRRT setrrt_planner("SetRRT");
    // setrrt_planner.link_ode_solver_pointer(&ode_solver);
    // PlannerVirtual* planner_pointer = &setrrt_planner;

    // // Define Goals
    // std::vector<Eigen::VectorXd> x_goals;
    // Eigen::VectorXd x1_goal(2); x1_goal << 8.0, 10.0; x_goals.push_back(x1_goal);
    // Eigen::VectorXd x2_goal(2); x2_goal << 0.0,  2.0; x_goals.push_back(x2_goal);
    // Eigen::VectorXd x3_goal(2); x3_goal << -M_PI/2, -M_PI/3; x_goals.push_back(x3_goal);
    // std::string goals_file = "outputs/" + ode_pointer->ode_name + "_goals.csv";
    // HELPER::write_traj_to_csv(x_goals, goals_file);
    
    // // Plan
    // std::vector<Eigen::VectorXd> sol = planner_pointer->plan(x_start, x_goals);
    // HELPER::log_trajectory(sol);
    // std::string sol_file = "outputs/" + planner_pointer->planner_name + "_" 
    //                         + ode_pointer->ode_name + "_sol.csv";
    // HELPER::write_traj_to_csv(sol, sol_file);

    // // Construct Controlled Trajectory
    // std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol); // HELPER::log_trajectory(traj);
    // std::string traj_file = "outputs/" + planner_pointer->planner_name + "_" 
    //                         + ode_pointer->ode_name + "_traj.csv";
    // HELPER::write_traj_to_csv(traj, traj_file);
}


int main(){
    std::cout << "[test invertpendulum]\n";

    std::cout << "[in docker develop env]\n";

    test();

    return 0;
}