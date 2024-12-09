#include<iostream>
#include<algorithm>
#include <cmath>
#include <tuple>
#include <unsupported/Eigen/MatrixFunctions>
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
    Eigen::VectorXd u_max(3); u_max <<  0.5*M_PI, 2*M_PI, 0.1;
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
    // // Generate and write unsafe 
    const int unsafe_regions = 3;
    std::vector<double> unsafe_circle_radius = CONFIG_SOLARSAIL::get_random_unsafe_raidus_km(unsafe_regions, 0.1, 0.5);
    std::vector<Eigen::VectorXd> unsafe_circle_center = CONFIG_SOLARSAIL::get_random_unsafe_centers_km(
        unsafe_regions, -4.0, 4.0, 
        unsafe_circle_radius, 
        x_start, 
        x_goals,
        ode_pointer->unit_length);
    ode_pointer->set_unsafecircles(unsafe_regions, unsafe_circle_center, unsafe_circle_radius);
    std::cout << "complete unsafe setting\n";
    std::string unsafe_file = "outputs/env" + std::to_string(env) + "_unsafe.csv";
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


std::tuple<bool, Eigen::MatrixXd> computeLQRGain_dlqr_infhorizon(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, 
const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, int max_iters = 10000, double tolerance = 1e-5) {
    /* Time-invarinat LQR Solve for the discrete-time algebraic Riccati Equation with Infinite Horizon.
        DARE: P = A'PA - A'PB*(R+B'PB)^-1 *B'PA + Q (assuming N=0)
        x(k+1) = A x(k) + B u(k)
        u(k) = -K x(k), where K = (R+B'PB)^-1 B'PA
        */
    // Initialize P (a starting guess)
    Eigen::MatrixXd P = Q;

    // Iterate to solve the matrix equation
    for (int iter = 0; iter < max_iters; iter++) {
        Eigen::MatrixXd P_new = A.transpose()*P*A - A.transpose()*P*B*(R+B.transpose()*P*B).inverse()*B.transpose()*P*A + Q;
        // Check for convergence using the Frobenius norm of the difference
        double diff = (P_new - P).norm() / P_new.norm();
        if (diff < tolerance) {
            Eigen::MatrixXd K = (R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
            return std::make_tuple(true, K);
        }
        // Update P
        P = P_new;
    }
    Eigen::MatrixXd K = (R+B.transpose()*P*B).inverse()*(B.transpose()*P*A);
    K.setZero();
    return std::make_tuple(false, K);
}


void control_motionplanner_and_lqr(std::string RUNTIME_CONTROL){
    const int size_x = 6;
    const int size_u = 3; // (control inputs, time_duration)

    auto [ode_pointer, planner_pointer, x_start, x_goals, env] = define_problem();

    // Plan & write reference trajectory
    // std::vector<Eigen::VectorXd> sol = planner_pointer->plan(x_start, x_goals);
    // HELPER::log_trajectory(sol);
    // std::string sol_file = "outputs/" + planner_pointer->planner_name + "_" 
    //                         + ode_pointer->ode_name + "_env" + std::to_string(env) + "_sol.csv";
    // HELPER::write_traj_to_csv(sol, sol_file);
    // // Write reference Trajectory
    // std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol, x_goals); // HELPER::log_trajectory(traj);
    // std::string traj_file = "outputs/" + planner_pointer->planner_name + "_" 
    //                         + ode_pointer->ode_name + "_env" + std::to_string(env) + "_traj.csv";
    // HELPER::write_traj_to_csv(traj, traj_file);
    // Read reference trajectory
    std::string traj_file = "outputs/env"+std::to_string(env)+"_trajplan.csv";
    std::vector<Eigen::VectorXd> traj = HELPER::read_csv_data(traj_file);
    std::cout << "final state (nominal): ";
    HELPER::log_vector(traj.back());

    // Define controller update frequency
    double time_control_update = 0.001;

    // Define state and control weights
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(size_x, size_x);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(size_u-1, size_u-1);
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(size_x, size_x);
    for(int i=0; i<size_x; i++){
        Q(i,i) = 1.0;
    }
    for(int i=0; i<size_u-1; i++){
        R(i,i) = 1.0;
    }
    // K matrix history for Time-varying LQR
    std::vector<Eigen::MatrixXd> K_history;

    // Compute nominal control trajecotory (time_control_update, time_integration)
    int number_data_per_control_update = int(time_control_update/planner_pointer->ode_solver_pointer->time_integration);
    std::vector<Eigen::VectorXd> traj_nominal;
    for(int i=0; i<traj.size(); i+=number_data_per_control_update){
        traj_nominal.push_back(traj[i]);
    }

    // Init
    int nominal_sim_step = traj_nominal.size();
    int time_elong = 0;
    bool is_process_noise = true;
    bool is_check_unsafe = false;
    Eigen::VectorXd x = x_start;
    std::vector<Eigen::VectorXd> traj_ref;
    std::vector<Eigen::VectorXd> traj_runtime;

    int sim_step = 0;
    for(int i=0; i<nominal_sim_step; i++){

        // Select reference (adaptive)
        // double min_dist = 9999.0;
        // int min_index = i;
        // int moving_window = 10;
        // if(i >= traj_nominal.size()){
        //     for(int k = traj_nominal.size()-moving_window; k<traj_nominal.size(); k++){
        //         Eigen::VectorXd x_u_k = traj_nominal[k];
        //         Eigen::VectorXd x_r_k = x_u_k.segment(0,size_x);        
        //         Eigen::VectorXd u_r_k = x_u_k.segment(size_x,size_u-1); 
        //         double dist_k = (x-x_r_k).norm();
        //         if(dist_k < min_dist){
        //             min_dist = dist_k;
        //             min_index = k;
        //         }
        //     }
        // }
        // else{
        //     for(int k = std::max(0, i-moving_window); k < std::min(i+moving_window, static_cast<int>(traj_nominal.size())); k++){
        //         Eigen::VectorXd x_u_k = traj_nominal[k];
        //         Eigen::VectorXd x_r_k = x_u_k.segment(0,size_x);        
        //         Eigen::VectorXd u_r_k = x_u_k.segment(size_x,size_u-1); 
        //         double dist_k = (x-x_r_k).norm();
        //         if(dist_k < min_dist){
        //             min_dist = dist_k;
        //             min_index = k;
        //         }
        //     }
        // }
        int min_index;
        if(i >= traj_nominal.size()){
            min_index = traj_nominal.size()-1;
        }
        else{
            min_index = i;
        }
        std::cout << sim_step << ": min index and dist " << min_index << "\n";
        Eigen::VectorXd x_ref = traj_nominal[min_index].segment(0, size_x);
        Eigen::VectorXd u_ref = traj_nominal[min_index].segment(size_x, size_u-1);
        Eigen::VectorXd u_online = u_ref;
        Eigen::VectorXd state_error = x - x_ref;
        traj_ref.push_back(x_ref);
        std::cout << "[Debug] norm(Error): " << state_error.norm() << "\n";
        
        /* (I) Time-invariant LQR (discrete time, infinite horizon) */
        // auto [A, B] = ode_pointer->get_linear_dynamics_matrices(x_ref, u_ref, time_control_update);
        // Eigen::MatrixXd F = (A * time_control_update).exp();
        // Eigen::MatrixXd G = B * time_control_update;
        // // HELPER::log_matrix(F); HELPER::log_matrix(G);
        // if (F.array().isNaN().any()){
        //     std::cout << "[Error] nan in F matrix, u offline:\n"; 
        //     HELPER::log_vector(u_ref);
        // }
        // else{
        //     auto[lqr_solved, K] = computeLQRGain_dlqr_infhorizon(F, G, Q, R);
        //     if(lqr_solved){             
        //         Eigen::VectorXd delta_u = -K*(state_error);
        //         u_online = u_online + delta_u; 
        //         std::cout << "[LQR Solved] u offline & online:\n"; 
        //         HELPER::log_vector(u_ref);
        //         HELPER::log_vector(u_online);
        //     }
        //     else{
        //         std::cout << "u offline:\n"; 
        //         HELPER::log_vector(u_ref);
        //     }
        // }

        /* (II) Time-varying LQR. It fails because K(t) explodes to -Inf after few time steps */
        // if(i == 0){
        //     Eigen::MatrixXd K = 1000.0 * Eigen::MatrixXd::Identity(6, 6);
        //     // // Integrate K backward in time
        //     for(int j=traj_nominal.size()-1; j>=0; j--){
        //         Eigen::VectorXd x_u_tau = traj_nominal[j];
        //         Eigen::VectorXd x_ref_tau = x_u_tau.segment(0,size_x);        
        //         Eigen::VectorXd u_ref_tau = x_u_tau.segment(size_x,size_u-1); 
        //         auto [A_tau, B_tau] = ode_pointer->get_linear_dynamics_matrices(x_ref_tau, u_ref_tau, time_control_update);
        //         Eigen::MatrixXd K_dot = -K*A_tau + K*B_tau*R.inverse()*B_tau.transpose()*K - Q - A_tau.transpose()*K;
        //         // std::cout << A_tau << "\n";
        //         // std::cout << B_tau << "\n";
        //         // std::cout << K_dot << "\n\n";
        //         Eigen::MatrixXd K_new = K - K_dot * time_control_update;
        //         K_history.insert(K_history.begin(), K_new);
        //         K = K_new;
        //     }
        //     std::cout << K_history.size() << "\t" << traj_nominal.size() << "\n";
        // }
        // if(i < K_history.size()){
        //     auto[A, B] = ode_pointer->get_linear_dynamics_matrices(x_ref, u_ref, time_control_update);
        //     Eigen::MatrixXd K = K_history[i];
        //     Eigen::VectorXd state_error = x - x_ref;              
        //     Eigen::VectorXd delta_u = -R.inverse()*B.transpose()*K*(state_error);
        //     u_online = u_online + delta_u; 
        //     std::cout << "[TV LQR Solved] u offline & online:\n"; 
        //     HELPER::log_vector(u_ref);
        //     HELPER::log_vector(u_online);
        // }
        // else{
        //     std::cout << i << " " << K_history.size() << "\n";
        // }

        /* (III) Optimal Neighboring */
        std::vector<Eigen::MatrixXd> Phi_history;
        Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(12, 12);
        int t_final_index = std::min(min_index+9999, static_cast<int>(traj_nominal.size()) );
        for(int j=min_index; j<t_final_index; j++){
            // std::cout << j << "\n";
            Eigen::VectorXd x_u_j = traj_nominal[j];
            Eigen::VectorXd x_tau = x_u_j.segment(0,size_x);        
            Eigen::VectorXd u_tau = x_u_j.segment(size_x,size_u-1); 
            auto [A_tau, B_tau] = ode_pointer->get_linear_dynamics_matrices(x_tau, u_tau, time_control_update);
            // Create the block matrix
            Eigen::MatrixXd A_tilde(12, 12);
            A_tilde.block(0, 0, 6, 6) = A_tau;           // Top-left block: A
            A_tilde.block(0, 6, 6, 6) = -0.5*B_tau*B_tau.transpose();        // Top-right block: A * A^T
            A_tilde.block(6, 0, 6, 6) = Eigen::MatrixXd::Zero(6, 6); // Bottom-left block: 0
            A_tilde.block(6, 6, 6, 6) = -A_tau.transpose();  // Bottom-right block: -A^T
            // Eigen::MatrixXd Phi_dt = A_tilde*Phi;
            Eigen::MatrixXd Phi_new = (A_tilde*time_control_update).exp() * Phi;
            Phi = Phi_new;
        }
        Eigen::MatrixXd phi_11 = Phi.block(0, 0, 6, 6);
        Eigen::MatrixXd phi_12 = Phi.block(0, 6, 6, 6);
        Eigen::VectorXd lambda_0 = phi_12.inverse()*(-phi_11 * state_error);
        if (lambda_0.array().isNaN().any()){
            std::cout << "u offline:\n"; 
            HELPER::log_vector(u_ref);
        }
        else{
            auto [A, B] = ode_pointer->get_linear_dynamics_matrices(x_ref, u_ref, time_control_update);
            Eigen::VectorXd delta_u = -0.5*B.transpose()*lambda_0;
            u_online = u_online + delta_u; 
            std::cout << "[Optimal Neighbor Solved] u offline & online:\n"; 
            HELPER::log_vector(u_ref);
            HELPER::log_vector(u_online);
        }

        /* IV. Replan */
        if(state_error.norm() > 0.3){
            std::vector<Eigen::VectorXd> sol = planner_pointer->plan(x, x_goals);
            double cost = planner_pointer->get_cost();
            planner_pointer->set_cost_threshold(cost);
            sol = planner_pointer->plan(x, x_goals);
            // update nominal trajectory
            traj_nominal.clear();
            std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol, x_goals);
            for(int i=0; i<traj.size(); i+=number_data_per_control_update){
                traj_nominal.push_back(traj[i]);
            }
            i = 0;
            state_error = state_error * 0.0;
            u_online[0] = sol[0][6];
            u_online[1] = sol[0][7];
            std::cout << "[Replan] u: \n";
            HELPER::log_vector(u_online);
        }


        std::cout << "norm state error: " << state_error.norm() << "\n";
        std::cout << "===\n";
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

        // update
        sim_step = sim_step + 1;
        x = traj_segment.back();

        if(ode_pointer->is_out_of_domain(x)){
            std::cout << "[Fail] out-of-bound\n";
            break;
        }
        if(is_check_unsafe && ode_pointer->is_in_unsafe(x)){
            std::cout << "[Fail] un-safe\n";
            break;
        }
        if(state_error.norm() > 2.0){
            std::cout << "[Debug] norm(Error) > 2.0 \n";
            break;
        }
        // write
        if(!traj_runtime.empty()){
            traj_runtime.pop_back();
        }
        // add data (u_ref, u_online, norm(error))
        for(auto& vec: traj_segment){
            vec.conservativeResize(vec.size() + 5);
            vec(vec.size() - 5) = u_ref[0];
            vec(vec.size() - 4) = u_ref[1];
            vec(vec.size() - 3) = u_online[0];
            vec(vec.size() - 2) = u_online[1];
            vec(vec.size() - 1) = state_error.norm();
        }
        traj_runtime.insert(traj_runtime.end(), traj_segment.begin(), traj_segment.end());
        // Reach goal break
        if(ode_pointer->is_goals(x, x_goals)){
            std::cout << "[Success] reach goal\n";
            std::cout << "final state (run): "; HELPER::log_vector(x);
            break;
        }
    }
    if(!ode_pointer->is_goals(x, x_goals)){
        std::cout << "[Fail] cannot reach goal within nominal + " << time_elong * time_control_update << " time \n";
        std::cout << "final state (run): ";
        HELPER::log_vector(x);
    }

    std::string traj_runtime_file = "outputs/env"+std::to_string(env)+"_trajrun_"+RUNTIME_CONTROL+".csv";
    HELPER::write_traj_to_csv(traj_runtime, traj_runtime_file);
    std::string traj_ref_file = "outputs/env"+std::to_string(env)+"_trajref_"+RUNTIME_CONTROL+".csv";
    HELPER::write_traj_to_csv(traj_ref, traj_ref_file);
}


int main(){
    std::cout << "[test SetRRT]\n";

    // test_construct_trajectory();

    // plan();

    // plan_AO();

    // std::string runtime_control = "nofeedback";
    // std::string runtime_control = "lqr";
    // std::string runtime_control = "optneighbor";
    std::string runtime_control = "optneighbor+replan";

    control_motionplanner_and_lqr(runtime_control);

    return 0;
}