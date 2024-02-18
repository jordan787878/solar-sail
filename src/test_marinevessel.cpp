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
#include "MDP.h"


std::tuple<OdeVirtual* , PlannerVirtual*, Eigen::VectorXd, std::vector<Eigen::VectorXd> > define_problem(){
    // Define ode
    OdeMarineVessel* ode_pointer = new OdeMarineVessel("MarineVessel");
    const int size_x = 3;
    const int size_u = 4;

    // Define start
    Eigen::VectorXd x_start = Eigen::VectorXd::Zero(size_x);
    x_start[0] = 1.0; x_start[1] = 1.0;

    // Define domain
    Eigen::VectorXd x_min(size_x); x_min << 0.0, 0.0, -M_PI;
    Eigen::VectorXd x_max(size_x); x_max << 10.0, 6.5, M_PI;
    Eigen::VectorXd u_min(size_u); u_min << 0.0, -0.0001, -0.1, 1.0; 
    Eigen::VectorXd u_max(size_u); u_max << 0.18, 0.0001,  0.1, 100.0;
    ode_pointer->set_domain(x_min, x_max, u_min, u_max);

    // Define unsafe
    const int number_unsafe_regions = 0;
    ode_pointer->set_unsafecircles(2, CONFIG_MARINE_VESSEL::get_random_unsafe_centers(number_unsafe_regions, 0.0, 10.0), 
                                      CONFIG_MARINE_VESSEL::get_random_unsafe_raidus(number_unsafe_regions, 0.25, 0.5));
    // Construct Unsafe for Visualization
    std::string unsafe_file = "outputs/" + ode_pointer->ode_name + "_unsafe.csv";
    HELPER::write_traj_to_csv(ode_pointer->output_unsafecircles(), unsafe_file);

    // Define noise (only x,y velocity noise)
    Eigen::VectorXd process_mean(3); process_mean << 0.0, 0.0, 0.0;
    Eigen::MatrixXd process_cov(3,3); process_cov << 0.01, 0.0, 0.0,
                                                     0.0, 0.01, 0.0,
                                                     0.0, 0.0,  0.0;
    ode_pointer->set_process_noise(process_mean, process_cov);

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
    planner_pointer->set_size_state_and_control();

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
    std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol, x_goals); // HELPER::log_trajectory(traj);
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
            std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol, x_goals); // HELPER::log_trajectory(traj);
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


Eigen::VectorXd approx_goal_state_and_control(std::vector<Eigen::VectorXd> x_goals){
    const int size_x = 3;
    const int size_u = 4; // (control inputs and time_duration)
    Eigen::VectorXd goal_state_and_control = Eigen::VectorXd::Zero(size_x+size_u);
    for(int i=0; i<size_x; i++){
        goal_state_and_control[i] = 0.5*(x_goals[i][0] + x_goals[i][1]);
    }
    return goal_state_and_control;
}


void control_motionplanner_and_lqr(){
    const int size_x = 3;
    const int size_u = 4; // (control inputs, time_duration)

    auto [ode_pointer, planner_pointer, x_start, x_goals] = define_problem();

    std::vector<Eigen::VectorXd> sol = planner_pointer->plan(x_start, x_goals);
    HELPER::log_trajectory(sol);
    std::string sol_file = "outputs/" + planner_pointer->planner_name + "_" 
                           + ode_pointer->ode_name + "_sol.csv";
    HELPER::write_traj_to_csv(sol, sol_file);
    // Construct reference Trajectory
    std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol, x_goals);
    std::string traj_file = "outputs/" + planner_pointer->planner_name + "_" 
                            + ode_pointer->ode_name + "_traj.csv";
    HELPER::write_traj_to_csv(traj, traj_file);

    // Compute linear dynamics
    double time_control_update = 1.0;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(size_x, size_x);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(size_u-1, size_u-1);
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(size_x, size_x);
    for(int i=0; i<size_x; i++){
        Q(i,i) = 1.0;
    }
    for(int i=0; i<size_u-1; i++){
        R(i,i) = 1.0;
    }

    // Compute nominal control trajecotory (time_control_update, time_integration)
    int number_data_per_control_update = int(time_control_update/planner_pointer->ode_solver_pointer->time_integration);
    std::vector<Eigen::VectorXd> traj_nominal;
    for(int i=0; i<traj.size(); i+=number_data_per_control_update){
        traj_nominal.push_back(traj[i]);
    }

    // Init
    int time_elong = 1;
    bool is_process_noise = true;
    bool is_check_unsafe = false;
    Eigen::VectorXd x = x_start;
    std::vector<Eigen::VectorXd> traj_runtime;

    for(int i=0; i<traj_nominal.size()+time_elong; i++){
        // std::cout << "[DEBUG] state: "; HELPER::log_vector(x); // real state

        // Select x_u ref
        Eigen::VectorXd x_u;
        if(i < traj_nominal.size()){
            x_u = traj_nominal[i];
        }
        else{
            // approximate a single point of (state and control)
            x_u = approx_goal_state_and_control(x_goals);
        }
        Eigen::VectorXd x_ref = x_u.segment(0,size_x); //std::cout << "x ref: "; HELPER::log_vector(x_ref);
        Eigen::VectorXd u_ref = x_u.segment(size_x,size_u-1); //std::cout << "[DEBUG] u ref: "; HELPER::log_vector(u_ref);

        // Compute u_online
        Eigen::VectorXd u_online = u_ref;
        // auto [F, G] = ode_pointer->get_linear_dynamics_matrices(x_ref, u_ref, time_control_update);
        // // HELPER::log_matrix(F); HELPER::log_matrix(G);
        // bool lqr_solved = HELPER::solveRiccatiIterationD(F, G, Q, R, P);
        // if(lqr_solved){
        //     // HELPER::log_matrix(P);
        //     Eigen::MatrixXd K = (R+G.transpose()*P*G).inverse()*G.transpose()*P*F;
        //     // std::cout << "K: "; HELPER::log_matrix(K);
        //     Eigen::VectorXd state_error = x - x_ref;                
        //     // std::cout << "state error: "; HELPER::log_vector(state_error);

        //     Eigen::VectorXd delta_u_lqr = -K*(state_error);
        //     // [TEMP]
        //     delta_u_lqr = 0.0 * delta_u_lqr;
        //     // std::cout << "delta u lqr: "; HELPER::log_vector(delta_u_lqr);
            
        //     // NOTE: this u_online can grow out of bound
        //     // (1) bound u_online to feasible control domain
        //     // of (2) bound the state error (Referenence Governer)
        //     u_online = u_online + delta_u_lqr; 
        //     u_online = u_online.cwiseMax(-Eigen::VectorXd::Ones(u_online.size()))
        //                         .cwiseMin(Eigen::VectorXd::Ones(u_online.size()));
        // }
        // std::cout << "u online (MP+LQR): "; HELPER::log_vector(u_online);

        //Execute with noise
        std::vector<Eigen::VectorXd> traj_segment;
        traj_segment = planner_pointer->ode_solver_pointer->solver_runge_kutta(
                                        x, 
                                        u_online, 
                                        planner_pointer->ode_solver_pointer->time_integration, 
                                        time_control_update, 
                                        x_goals, 
                                        is_process_noise,
                                        is_check_unsafe);
        // Out-of-domain and Unsafe break
        // NOTE 2024.02.15: need to redfine the solver_runge_kutta return value
        if(traj_segment.empty()){
            std::cout << "[DEBUG] out-of-domain or unsafe break\n";
            break;
        }
        // update
        x = traj_segment.back();
        // write
        if(!traj_runtime.empty()){
            traj_runtime.pop_back();
        }
        traj_runtime.insert(traj_runtime.end(), traj_segment.begin(), traj_segment.end());
        // Reach goal break
        if(ode_pointer->is_goals(x, x_goals)){
            std::cout << "[DEBUG] reach goal\n";
            std::cout << "goal state (run): "; HELPER::log_vector(x);
            break;
        }
    }
    if(!ode_pointer->is_goals(x, x_goals)){
        std::cout << "[DEBUG] cannot reach goal within nominal + " << time_elong * time_control_update << " time \n";
    }

    std::string traj_runtime_file = "outputs/" + planner_pointer->planner_name + "_" 
                            + ode_pointer->ode_name + "_noise_traj(mp+lqr).csv";
    HELPER::write_traj_to_csv(traj_runtime, traj_runtime_file); // HELPER::log_trajectory(traj);
}


void control_mdp(){
    const int size_x = 3;
    const int size_u = 4; // (control inputs, time_duration)

    // Define the problem
    auto [ode_pointer, planner_pointer, x_start, x_goals] = define_problem();
    std::vector<Eigen::VectorXd> sol_optimal;

    // Plan AO
    double plan_time_max = 10.0;
    int N_run = 6;
    planner_pointer->set_plan_time_max(plan_time_max);
    for(int i=0; i<N_run; i++){
        std::vector<Eigen::VectorXd> sol = planner_pointer->plan(x_start, x_goals);

        if(planner_pointer->is_success){
            double cost = planner_pointer->get_cost();
            std::cout << "[DEBUG] cost: " << cost << "\n";
            planner_pointer->set_cost_threshold(cost);
            std::cout << "[DEBUG] update cost threshold\n";
            HELPER::log_trajectory(sol);
            sol_optimal.clear();
            sol_optimal = sol;
        }
        else{ // increase plan time max
            planner_pointer->set_plan_time_max(plan_time_max);
        }
    }

    // Write solution
    HELPER::log_trajectory(sol_optimal);
    std::string sol_file = "outputs/" + planner_pointer->planner_name + "_" 
                           + ode_pointer->ode_name + "_sol.csv";
    HELPER::write_traj_to_csv(sol_optimal, sol_file);

    // Construct reference Trajectory
    std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol_optimal, x_goals);
    std::string traj_file = "outputs/" + planner_pointer->planner_name + "_" 
                            + ode_pointer->ode_name + "_traj.csv";
    HELPER::write_traj_to_csv(traj, traj_file);

    // [Obsolete] Compute nominal control trajecotory (time_control_update, time_integration)
    double time_control_update = 1.0;
    std::vector<Eigen::VectorXd> traj_nominal = traj;
    // int number_data_per_control_update = int(time_control_update/planner_pointer->ode_solver_pointer->time_integration);
    // std::vector<Eigen::VectorXd> traj_nominal;
    // for(int i=0; i<traj.size(); i+=number_data_per_control_update){
    //     traj_nominal.push_back(traj[i]);
    // }
    // HELPER::log_trajectory(traj_nominal);

    // Define MDP
    const int number_per_state  = 60;
    const int number_per_action = 6;
    MDP* mdp_pointer = new MDP(planner_pointer, number_per_state, number_per_action);
    mdp_pointer->x_goals = x_goals;

    // Construct discrete state
    mdp_pointer->construct_discrete_state(traj_nominal);

    // Construct transition
    mdp_pointer->construct_transition();
    if(!mdp_pointer->is_set_transitions_success){
        std::cout << "[ERROR] cannot find transition to goal\n";
        return;
    }

    // write discrete state
    std::vector<Eigen::VectorXd> discrete_traj = mdp_pointer->debug_discrete_state(); //HELPER::log_trajectory(discrete_traj);
    std::string discrete_traj_file = "outputs/" + planner_pointer->planner_name + "_" 
                            + ode_pointer->ode_name + "_traj(discrete).csv";
    HELPER::write_traj_to_csv(discrete_traj, discrete_traj_file);
    // write transision
    std::vector<Eigen::VectorXd> data = mdp_pointer->write_transition();
    std::string transition_file = "outputs/" + planner_pointer->planner_name + "_" 
                            + ode_pointer->ode_name + "_transition.csv";
    HELPER::write_traj_to_csv(data, transition_file);

    // Value iteration
    mdp_pointer->value_iteration();

    // Control Synthesis
    mdp_pointer->synthesize_control();

    // // Print out discrete state
    // for(int i=0; i<mdp_pointer->nodes_Q.size(); i++){
    //     std::cout << mdp_pointer->nodes_Q[i] << " ";
    // }
    // std::cout << "\n";
    // Test feedback control
    // unsigned int node_Q = mdp_pointer->nodes_Q[0];
    // Eigen::VectorXd u = mdp_pointer->discrete_state_to_control[node_Q];
    // std::cout << "[DEBUG] node_Q: " << node_Q << "\n";
    // HELPER::log_vector(u);

    // Run-time control
    int time_elong = 10;
    bool is_process_noise = false;
    bool is_check_unsafe = true;
    Eigen::VectorXd x = x_start;
    std::vector<Eigen::VectorXd> traj_runtime;

    for(int i=0; i<traj_nominal.size()+time_elong; i++){
        // std::cout << "[DEBUG] state: "; HELPER::log_vector(x); // real state

        // Compute u_online
        auto [solved, u_online] = mdp_pointer->get_feedback_control(x);
        if(!solved){
            std::cout << "[ERROR] state exceeds MDP construction\n";
            break;
        }
        // std::cout << "control: "; HELPER::log_vector(u_online);

        //Execute with noise
        std::vector<Eigen::VectorXd> traj_segment;
        traj_segment = planner_pointer->ode_solver_pointer->solver_runge_kutta(
                                        x, 
                                        u_online, 
                                        planner_pointer->ode_solver_pointer->time_integration, 
                                        time_control_update, 
                                        x_goals, 
                                        is_process_noise,
                                        is_check_unsafe);
        // Out-of-domain and Unsafe break
        if(traj_segment.empty()){
            std::cout << "[DEBUG] out-of-domain or unsafe break\n";
            break;
        }
        // update
        x = traj_segment.back();
        // write to trajectory data (state, control)
        for(auto& x_k : traj_segment){
            Eigen::VectorXd x_k_holder = x_k;
            x_k.conservativeResize(x.size()+u_online.size());
            x_k << x_k_holder, u_online;
        }
        if(!traj_runtime.empty()){
            traj_runtime.pop_back();
        }
        traj_runtime.insert(traj_runtime.end(), traj_segment.begin(), traj_segment.end());
        // Reach goal break
        if(ode_pointer->is_goals(x, x_goals)){
            std::cout << "[DEBUG] reach goal\n";
            std::cout << "goal state (run): "; HELPER::log_vector(x);
            break;
        }
    }
    if(!ode_pointer->is_goals(x, x_goals)){
        std::cout << "[DEBUG] cannot reach goal within nominal + " << time_elong * time_control_update << " time \n";
    }

    std::string traj_runtime_file = "outputs/" + planner_pointer->planner_name + "_" 
                            + ode_pointer->ode_name + "_noise_traj(mp+lqr).csv";
    HELPER::write_traj_to_csv(traj_runtime, traj_runtime_file); // HELPER::log_trajectory(traj);

}


int main(){
    std::cout << "[test marinevessel]\n";

    std::cout << "[in docker develop env]\n";

    // plan();

    // plan_AO();

    // control_motionplanner_and_lqr();

    control_mdp();

    return 0;
}