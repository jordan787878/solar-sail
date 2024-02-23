#include<iostream>
#include <cmath>
#include <tuple>
#include "OdeSolarsail.h"
#include "OdeVirtual.h"
#include "OdeSolver.h"
#include "helperfunctions.h"
#include "PlannerVirtual.h"
#include "SetRRT.h"

#include "ConfigSolarsail.h"

std::tuple<OdeVirtual* , PlannerVirtual*, Eigen::VectorXd, std::vector<Eigen::VectorXd>, int > define_problem(){
    // Define ode
    const int env = 4;
    OdeSolarsail* ode_pointer = new OdeSolarsail("Solarsail");
    ode_pointer->set_params(CONFIG_SOLARSAIL::get_parameters(env));

    // Define domain
    Eigen::VectorXd x_min(6); x_min << -1, -1, -1, -50, -50, -50;
    Eigen::VectorXd x_max(6); x_max << 1, 1, 1, 50, 50, 50;
    Eigen::VectorXd u_min(3); u_min << -0.5*M_PI, 0.0, 0.001;
    Eigen::VectorXd u_max(3); u_max << 0.5*M_PI, 2*M_PI, 0.1;
    ode_pointer->set_domain(x_min, x_max, u_min, u_max);

    // Define start
    Eigen::VectorXd x_start(6);
    for(int i=0; i<6; i++){x_start[i] = CONFIG_SOLARSAIL::get_state_init(env)[i];}

    // Define goal
    std::vector<Eigen::VectorXd> x_goals;
    Eigen::VectorXd x_goal(6);  x_goal << 0, 0, 0, 0, 0, 0;
    x_goals.push_back(x_goal);

    // Define unsafe
    std::cout << "setting unsafe...\n";
    const int unsafe_regions = 3;
    std::vector<double> unsafe_circle_radius = CONFIG_SOLARSAIL::get_random_unsafe_raidus_km(unsafe_regions, 0.3, 1.0);
    std::vector<Eigen::VectorXd> unsafe_circle_center = CONFIG_SOLARSAIL::get_random_unsafe_centers_km(
        unsafe_regions, -4.0, 4.0, 
        unsafe_circle_radius, 
        x_start, 
        x_goals,
        ode_pointer->unit_length);
    ode_pointer->set_unsafecircles(3, unsafe_circle_center, unsafe_circle_radius);
    std::cout << "complet unsafe setting\n";
    // Construct Unsafe for Visualization
    std::string unsafe_file = "outputs/" + ode_pointer->ode_name + "_env" + std::to_string(env) + "_unsafe.csv";
    HELPER::write_traj_to_csv(ode_pointer->output_unsafecircles(), unsafe_file);

    // Define asteroid radius
    ode_pointer->set_r_ast(0.25);

    // Define process noise
    Eigen::VectorXd process_mean(3); process_mean << CONFIG_SOLARSAIL::get_process_mean();
    Eigen::MatrixXd process_cov(3,3); process_cov << CONFIG_SOLARSAIL::get_process_cov();
    ode_pointer->set_process_noise(process_mean, process_cov);

    // Define ode solver
    double time_integration = 1e-4;
    OdeSolver* ode_solver_ptr = new OdeSolver(time_integration);
    ode_solver_ptr->link_ode_pointer(ode_pointer);

    // Define planner
    SetRRT* planner_pointer = new SetRRT("SetRRT");
    planner_pointer->link_ode_solver_pointer(ode_solver_ptr);
    planner_pointer->control_resolution = 50.0;
    planner_pointer->set_size_state_and_control();

    // // test ode_solver works in planner
    // Eigen::VectorXd empty_control = Eigen::VectorXd::Zero(3);
    // std::vector<Eigen::VectorXd> empoty_goal_states;
    // double time_nominal = 0.5;
    // const bool use_process_noise = false;
    // const bool check_unsafe = false;
    // std::vector<Eigen::VectorXd> nom_traj = planner_pointer->ode_solver_pointer->solver_runge_kutta(
    //                                             x_start, 
    //                                             empty_control, 
    //                                             planner_pointer->ode_solver_pointer->time_integration, 
    //                                             time_nominal, 
    //                                             empoty_goal_states, 
    //                                             use_process_noise, 
    //                                             check_unsafe);
    // std::string nom_traj_file = "outputs/" + ode_pointer->ode_name + "_env" + std::to_string(env) + "_traj.csv";
    // HELPER::write_traj_to_csv(nom_traj, nom_traj_file);

    return std::make_tuple(ode_pointer, planner_pointer, x_start, x_goals, env);
}


void test_construct_trajectory(){

    auto [ode_pointer, planner_pointer, x_start, x_goals, env] = define_problem();

    // Read solution
    std::string sol_file = "outputs/SetRRT_Solarsail_env4_sol.csv";
    std::vector<Eigen::VectorXd> sol = HELPER::read_csv_data(sol_file);
    HELPER::log_trajectory(sol);

    std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol, x_goals);
    std::string traj_file = "outputs/" + planner_pointer->planner_name + "_" 
                            + ode_pointer->ode_name + "_env" + std::to_string(env) + "_traj(debug).csv";
    HELPER::write_traj_to_csv(traj, traj_file);
}


void plan(){
    auto [ode_pointer, planner_pointer, x_start, x_goals, env] = define_problem();

    std::vector<Eigen::VectorXd> sol = planner_pointer->plan(x_start, x_goals);

    HELPER::log_trajectory(sol);
    std::string sol_file = "outputs/" + planner_pointer->planner_name + "_" + ode_pointer->ode_name + "_env" + std::to_string(env) + "_sol.csv";
    HELPER::write_traj_to_csv(sol, sol_file);

    // Construct Controlled Trajectory
    std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol, x_goals); // HELPER::log_trajectory(traj);
    std::string traj_file = "outputs/" + planner_pointer->planner_name + "_" + ode_pointer->ode_name + "_env" + std::to_string(env) + "_traj.csv";
    HELPER::write_traj_to_csv(traj, traj_file);

    // Construct Controlled Trajectory subject to Process noise
    // std::vector<Eigen::VectorXd> traj_noise = planner_pointer->construct_trajectory(sol, x_goals, true);
    // std::string traj_noise_file = "outputs/" + planner_pointer->planner_name + "_" + ode_pointer->ode_name + "_env" + std::to_string(env) + "_trajnoise.csv";
    // HELPER::write_traj_to_csv(traj_noise, traj_noise_file);
}


void plan_AO(){
    auto [ode_pointer, planner_pointer, x_start, x_goals, env] = define_problem();
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
                            + ode_pointer->ode_name + "_env" + std::to_string(env)
                            + "_cost:" + HELPER::doubleToString(cost) + "_sol.csv";
            HELPER::write_traj_to_csv(sol, sol_file);

            // Construct and write trajectory
            std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol, x_goals); // HELPER::log_trajectory(traj);
            std::string traj_file = "outputs/" + planner_pointer->planner_name + "_" 
                                    + ode_pointer->ode_name + "_env" + std::to_string(env)
                                    + "_cost:" + HELPER::doubleToString(cost) + "_traj.csv";
            HELPER::write_traj_to_csv(traj, traj_file);
        }
        else{ // increase plan time max
            plan_time_max = plan_time_max + 100;
            planner_pointer->set_plan_time_max(plan_time_max);
        }
    }
}


void control_motionplanner_and_lqr(){
    const int size_x = 6;
    const int size_u = 3; // (control inputs, time_duration)

    auto [ode_pointer, planner_pointer, x_start, x_goals, env] = define_problem();

    std::vector<Eigen::VectorXd> sol = planner_pointer->plan(x_start, x_goals);
    HELPER::log_trajectory(sol);
    std::string sol_file = "outputs/" + planner_pointer->planner_name + "_" 
                           + ode_pointer->ode_name + "_env" + std::to_string(env) + "_sol.csv";
    HELPER::write_traj_to_csv(sol, sol_file);
    // Construct reference Trajectory
    std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol, x_goals); // HELPER::log_trajectory(traj);
    std::string traj_file = "outputs/" + planner_pointer->planner_name + "_" 
                            + ode_pointer->ode_name + "_env" + std::to_string(env) + "_traj.csv";
    HELPER::write_traj_to_csv(traj, traj_file);
    // Read reference trajectory
    // std::string traj_file = "outputs/SetRRT_Solarsail_env4_traj.csv";
    // std::vector<Eigen::VectorXd> traj = HELPER::read_csv_data(traj_file);

    // // Compute linear dynamics
    double time_control_update = 0.001;
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
    int time_elong = 100;
    bool is_process_noise = false;
    bool is_check_unsafe = false;
    Eigen::VectorXd x = x_start;
    std::vector<Eigen::VectorXd> traj_runtime;

    for(int i=0; i<traj_nominal.size()+time_elong; i++){
        //std::cout << "[DEBUG] state: "; HELPER::log_vector(x); // real state

        // Select x_u ref
        Eigen::VectorXd x_u;
        if(i < traj_nominal.size()){
            x_u = traj_nominal[i];
        }
        else{
            // because the goal state is (0,0,0,0)
            x_u = 0.0 * traj_nominal[0];
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


int main(){
    std::cout << "[test SetRRT]\n";

    // test_construct_trajectory();

    // plan();

    // plan_AO();

    control_motionplanner_and_lqr();

    return 0;
}