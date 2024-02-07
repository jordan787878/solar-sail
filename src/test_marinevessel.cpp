#include<iostream>
#include <cmath>
#include <tuple>
#include "OdeVirtual.h"
#include "OdeMarineVessel.h"
#include "OdeSolver.h"
#include "PlannerVirtual.h"
#include "SetRRT.h"
#include "helperfunctions.h"
#include "ConfigMarineVessel.h"


std::tuple<OdeVirtual* , PlannerVirtual*, Eigen::VectorXd, std::vector<Eigen::VectorXd> > define_problem(){
    // Define ode
    OdeMarineVessel* ode_pointer = new OdeMarineVessel("MarineVessel");
    const int size_x = 3;
    const int size_u = 4;

    // Define start
    Eigen::VectorXd x_start(size_x); for(int i=0; i<size_x; i++){x_start[i] = 0.0;}

    // Define domain
    Eigen::VectorXd x_min(size_x); x_min << 0.0, 0.0, -M_PI;
    Eigen::VectorXd x_max(size_x); x_max << 10.0, 6.5, M_PI;
    Eigen::VectorXd u_min(size_u); u_min << 0.0, -0.00, -0.1, 1.0; 
    Eigen::VectorXd u_max(size_u); u_max << 0.18, 0.00,  0.1, 100.0;
    ode_pointer->set_domain(x_min, x_max, u_min, u_max);
    ode_pointer->set_unsafecircles(2, CONFIG_MARINE_VESSEL::get_random_unsafe_centers(15, 0.0, 10.0), 
                                      CONFIG_MARINE_VESSEL::get_random_unsafe_raidus(15, 0.25, 0.5));
    // Eigen::VectorXd process_mean(3); process_mean << CONFIG_SOLARSAIL::get_process_mean();
    // Eigen::MatrixXd process_cov(3,3); process_cov << CONFIG_SOLARSAIL::get_process_cov();
    // ode_solarsail.set_process_noise(process_mean, process_cov);

    // Construct Unsafe for Visualization
    std::string unsafe_file = "outputs/" + ode_pointer->ode_name + "_unsafe.csv";
    HELPER::write_traj_to_csv(ode_pointer->output_unsafecircles(), unsafe_file);

    // Define ode solver
    const double time_integration = 1e-1;
    OdeSolver* ode_solver_ptr = new OdeSolver(time_integration);
    ode_solver_ptr->link_ode_pointer(ode_pointer);

    // // Solving ode
    // Eigen::VectorXd nominal_control(size_u-1); nominal_control << 0.9, 0.0, 0.05;
    // double time_nominal = 10.0;
    // std::vector<Eigen::VectorXd> empoty_goal_states;
    // const bool use_process_noise = false;
    // const bool check_unsafe = false;
    // std::vector<Eigen::VectorXd> nom_traj = ode_solver.solver_runge_kutta(
    //                                             x_start, 
    //                                             nominal_control, 
    //                                             ode_solver.time_integration, 
    //                                             time_nominal, empoty_goal_states, use_process_noise, check_unsafe);
    // // Writing result
    // std::string nom_traj_file = "outputs/" + ode_pointer->ode_name + "_traj.csv";
    // HELPER::write_traj_to_csv(nom_traj, nom_traj_file);

    // Define planner
    SetRRT* planner_pointer = new SetRRT("SetRRT");
    planner_pointer->link_ode_solver_pointer(ode_solver_ptr);
    planner_pointer->control_resolution = 100.0;

    // Define Goals
    std::vector<Eigen::VectorXd> x_goals;
    Eigen::VectorXd x1_goal(2); x1_goal << 6.0, 7.0; x_goals.push_back(x1_goal);
    Eigen::VectorXd x2_goal(2); x2_goal << 3.0, 4.0; x_goals.push_back(x2_goal);
    Eigen::VectorXd x3_goal(2); x3_goal << -M_PI/2, -M_PI/3; x_goals.push_back(x3_goal);
    std::string goals_file = "outputs/" + ode_pointer->ode_name + "_goals.csv";
    HELPER::write_traj_to_csv(x_goals, goals_file);
    
    return std::make_tuple(ode_pointer, planner_pointer, x_start, x_goals);
}


void plan(){
    auto [ode_pointer, planner_pointer, x_start, x_goals] = define_problem();    
    std::cout << "ode: " << ode_pointer->ode_name << "\n";
    std::cout << "planner: " << planner_pointer->planner_name << "\n";

    // Plan
    std::vector<Eigen::VectorXd> sol = planner_pointer->plan(x_start, x_goals);

    // Write solution
    double cost = planner_pointer->get_cost();
    std::cout << "[DEBUG] cost: " << cost << "\n";
    HELPER::log_trajectory(sol);
    std::string sol_file = "outputs/" + planner_pointer->planner_name + "_" 
                            + ode_pointer->ode_name + "_sol.csv";
    HELPER::write_traj_to_csv(sol, sol_file);

    // Construct and write trajectory
    std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol); // HELPER::log_trajectory(traj);
    std::string traj_file = "outputs/" + planner_pointer->planner_name + "_" 
                            + ode_pointer->ode_name + "_traj.csv";
    HELPER::write_traj_to_csv(traj, traj_file);
}


void plan_AO(){
    auto [ode_pointer, planner_pointer, x_start, x_goals] = define_problem();    
    std::cout << "ode: " << ode_pointer->ode_name << "\n";
    std::cout << "planner: " << planner_pointer->planner_name << "\n";

    double plan_time_max = 100.0;
    int N_run = 10;
    planner_pointer->set_plan_time_max(plan_time_max);

    for(int i=0; i<N_run; i++){
        std::vector<Eigen::VectorXd> sol = planner_pointer->plan(x_start, x_goals);

        if(planner_pointer->is_success){
            double cost = planner_pointer->get_cost();
            std::cout << "[DEBUG] cost: " << cost << "\n";
            planner_pointer->set_cost_threshold(cost);
            std::cout << "[DEBUG] update cost threshold\n";

            HELPER::log_trajectory(sol);
            std::string sol_file = "outputs/" + planner_pointer->planner_name + "_" 
                            + ode_pointer->ode_name + "_cost:" + HELPER::doubleToString(cost) + "_sol.csv";
            HELPER::write_traj_to_csv(sol, sol_file);

            // Construct and write trajectory
            std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol); // HELPER::log_trajectory(traj);
            std::string traj_file = "outputs/" + planner_pointer->planner_name + "_" 
                                    + ode_pointer->ode_name + "_cost:" + HELPER::doubleToString(cost) + "_traj.csv";
            HELPER::write_traj_to_csv(traj, traj_file);
        }
        else{ // increase plan time max
            plan_time_max = plan_time_max + 100;
            planner_pointer->set_plan_time_max(plan_time_max);
        }
    }
}


int main(){
    std::cout << "[test marinevessel]\n";

    std::cout << "[in docker develop env]\n";

    // plan();

    plan_AO();

    return 0;
}