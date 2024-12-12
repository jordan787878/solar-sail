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
    // std::cout << "setting unsafe...\n";
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
    // std::cout << "complete unsafe setting\n";
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


void control_motionplanner_and_lqr(std::string RUNTIME_CONTROL, double& success_flag, double& time_of_flight, double& radius_final){
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
    // std::cout << "final state (nominal): ";
    // HELPER::log_vector(traj.back());

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
    // Final state weights
    Eigen::MatrixXd Qf = 0.0 * Eigen::MatrixXd::Identity(6, 6);

    // Compute nominal control trajecotory (time_control_update, time_integration)
    int number_data_per_control_update = int(time_control_update/planner_pointer->ode_solver_pointer->time_integration);
    std::vector<Eigen::VectorXd> traj_nominal;
    for(int i=0; i<traj.size(); i+=number_data_per_control_update){
        traj_nominal.push_back(traj[i]);
    }
    traj_nominal.push_back(traj.back()); // append the landing state

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
        // std::cout << sim_step << ", sol index: " << min_index << "\n";
        Eigen::VectorXd x_ref = traj_nominal[min_index].segment(0, size_x);
        Eigen::VectorXd u_ref = traj_nominal[min_index].segment(size_x, size_u-1);
        Eigen::VectorXd u_online = u_ref;
        Eigen::VectorXd state_error = x - x_ref;
        traj_ref.push_back(x_ref);
        // std::cout << "[Debug] norm(Error): " << state_error.norm() << "\n";
        
        /* (I) Time-invariant LQR (discrete time, infinite horizon) */
        // auto [A, B] = ode_pointer->get_linear_dynamics_matrices(x_ref, u_ref, time_control_update);
        // Eigen::MatrixXd A_tilde(8, 8);
        // A_tilde.block(0, 0, 6, 6) = A;         
        // A_tilde.block(0, 6, 6, 2) = B;        
        // A_tilde.block(6, 0, 2, 6) = Eigen::MatrixXd::Zero(2, 6); 
        // A_tilde.block(6, 6, 2, 2) = Eigen::MatrixXd::Zero(2, 2);
        // Eigen::MatrixXd A_tilde_exp = (A_tilde*time_control_update).exp();
        // Eigen::MatrixXd F = A_tilde_exp.block(0, 0, 6, 6);
        // Eigen::MatrixXd G = A_tilde_exp.block(0, 6, 6, 2);
        // Eigen::MatrixXd I_check = A_tilde_exp.block(6, 6, 2, 2); // HELPER::log_matrix(I_check);
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

        /* (II) discrete-time Time-varying LQR */
        Eigen::MatrixXd Pt = Qf;
        Eigen::MatrixXd F;
        Eigen::MatrixXd G;
        Eigen::MatrixXd eye_six = Eigen::MatrixXd::Identity(6,6);
        for(int j=traj_nominal.size()-1; j>(i); j--){
            Eigen::VectorXd x_u_j = traj_nominal[j];
            Eigen::VectorXd x_tau = x_u_j.segment(0,size_x);    
            Eigen::VectorXd u_tau = x_u_j.segment(size_x,size_u-1); 
            auto [A_tau, B_tau] = ode_pointer->get_linear_dynamics_matrices(x_tau, u_tau, time_control_update); 
            Eigen::MatrixXd A_tilde(8, 8);
            A_tilde.block(0, 0, 6, 6) = A_tau;         
            A_tilde.block(0, 6, 6, 2) = B_tau;        
            A_tilde.block(6, 0, 2, 6) = Eigen::MatrixXd::Zero(2, 6); 
            A_tilde.block(6, 6, 2, 2) = Eigen::MatrixXd::Zero(2, 2);
            Eigen::MatrixXd A_tilde_exp = (A_tilde*time_control_update).exp();
            F = A_tilde_exp.block(0, 0, 6, 6);
            G = A_tilde_exp.block(0, 6, 6, 2);
            // (basic form)
            Eigen::MatrixXd P_new = Q + F.transpose()*Pt*F - F.transpose()*Pt*G*(R+G.transpose()*Pt*G).inverse()*G.transpose()*Pt*F;
            // (symmetric form)
            // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(Pt);
            // Eigen::MatrixXd Pt_sqrt = solver.eigenvectors() * solver.eigenvalues().cwiseSqrt().asDiagonal() * solver.eigenvectors().transpose();
            // Eigen::MatrixXd P_new = Q + F.transpose()*Pt_sqrt*(eye_six + Pt_sqrt*G*R.inverse()*G.transpose()*Pt_sqrt).inverse()*Pt_sqrt*F;
            Pt = P_new;
        }
        // HELPER::log_matrix(Pt);
        auto [A, B] = ode_pointer->get_linear_dynamics_matrices(x_ref, u_ref, time_control_update); 
        Eigen::MatrixXd A_tilde(8, 8);
        A_tilde.block(0, 0, 6, 6) = A;         
        A_tilde.block(0, 6, 6, 2) = B;        
        A_tilde.block(6, 0, 2, 6) = Eigen::MatrixXd::Zero(2, 6); 
        A_tilde.block(6, 6, 2, 2) = Eigen::MatrixXd::Zero(2, 2);
        Eigen::MatrixXd A_tilde_exp = (A_tilde*time_control_update).exp();
        F = A_tilde_exp.block(0, 0, 6, 6);
        G = A_tilde_exp.block(0, 6, 6, 2);
        Eigen::MatrixXd Kt = (R+G.transpose()*Pt*G).inverse()*G.transpose()*Pt*F;
        // HELPER::log_matrix(Kt);
        Eigen::VectorXd delta_u = -Kt*(state_error);
        u_online = u_online + delta_u; 
        // std::cout << "[Time-varying LQR Solved] u offline & online:\n"; 
        // HELPER::log_vector(u_ref);
        // HELPER::log_vector(u_online);

        /* NOTE: continuous-time formulation fails because P explodes while integrating backward */
        // Eigen::MatrixXd P = Qf;
        // for(int j=traj_nominal.size()-1; j>(i); j--){
        //     Eigen::VectorXd x_u_j = traj_nominal[j];
        //     Eigen::VectorXd x_tau = x_u_j.segment(0,size_x);    
        //     Eigen::VectorXd u_tau = x_u_j.segment(size_x,size_u-1); 
        //     auto [A, B] = ode_pointer->get_linear_dynamics_matrices(x_tau, u_tau, time_control_update); 
        //     Eigen::MatrixXd dPdt = -P*A - A.transpose()*P - Q + P*B*R.inverse()*B.transpose()*P;
        //     Eigen::MatrixXd P_new = P - dPdt * time_control_update;
        //     P = P_new;
        // }
        // HELPER::log_matrix(P);

        /* (III) Optimal Neighboring */
        // std::vector<Eigen::MatrixXd> Phi_history;
        // Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(12, 12);
        // int t_final_index = std::min(min_index+9999, static_cast<int>(traj_nominal.size()) );
        // for(int j=min_index; j<t_final_index; j++){
        //     // std::cout << j << "\n";
        //     Eigen::VectorXd x_u_j = traj_nominal[j];
        //     Eigen::VectorXd x_tau = x_u_j.segment(0,size_x);        
        //     Eigen::VectorXd u_tau = x_u_j.segment(size_x,size_u-1); 
        //     auto [A_tau, B_tau] = ode_pointer->get_linear_dynamics_matrices(x_tau, u_tau, time_control_update);
        //     // Create the block matrix
        //     Eigen::MatrixXd A_tilde(12, 12);
        //     A_tilde.block(0, 0, 6, 6) = A_tau;           // Top-left block: A
        //     A_tilde.block(0, 6, 6, 6) = -0.5*B_tau*B_tau.transpose();        // Top-right block: A * A^T
        //     A_tilde.block(6, 0, 6, 6) = Eigen::MatrixXd::Zero(6, 6); // Bottom-left block: 0
        //     A_tilde.block(6, 6, 6, 6) = -A_tau.transpose();  // Bottom-right block: -A^T
        //     // Eigen::MatrixXd Phi_dt = A_tilde*Phi;
        //     Eigen::MatrixXd Phi_new = (A_tilde*time_control_update).exp() * Phi;
        //     Phi = Phi_new;
        // }
        // Eigen::MatrixXd phi_11 = Phi.block(0, 0, 6, 6);
        // Eigen::MatrixXd phi_12 = Phi.block(0, 6, 6, 6);
        // Eigen::VectorXd lambda_0 = phi_12.inverse()*(-phi_11 * state_error);
        // if (lambda_0.array().isNaN().any()){
        //     std::cout << "u offline:\n"; 
        //     HELPER::log_vector(u_ref);
        // }
        // else{
        //     auto [A, B] = ode_pointer->get_linear_dynamics_matrices(x_ref, u_ref, time_control_update);
        //     Eigen::VectorXd delta_u = -0.5*B.transpose()*lambda_0;
        //     u_online = u_online + delta_u; 
        //     std::cout << "[Optimal Neighbor Solved] u offline & online:\n"; 
        //     HELPER::log_vector(u_ref);
        //     HELPER::log_vector(u_online);
        // }

        /* IV. Replan */
        // if(state_error.norm() > 0.1){
        //     double cost;
        //     // Initial plan
        //     planner_pointer->set_plan_time_max(60);
        //     std::vector<Eigen::VectorXd> sol = planner_pointer->plan(x, x_goals);
        //     if(planner_pointer->is_success){
        //         // std::cout << "[Replan] first u: " << sol[0][6] << " " << sol[0][7] << "\n";
        //         // Optimize plan
        //         cost = planner_pointer->get_cost();
        //         planner_pointer->set_cost_threshold(cost);
        //         std::vector<Eigen::VectorXd> sol_AO = planner_pointer->plan(x, x_goals);
        //         if(planner_pointer->is_success){
        //             // std::cout << "[DEBUG] AO Success\n";
        //             sol.clear();
        //             sol = sol_AO;
        //         }
        //         // Update nominal trajectory
        //         traj_nominal.clear();
        //         std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol, x_goals);
        //         for(int i=0; i<traj.size(); i+=number_data_per_control_update){
        //             traj_nominal.push_back(traj[i]);
        //         }
        //         traj_nominal.push_back(traj.back()); // append the landing state
        //         i = 0;
        //         state_error = state_error * 0.0;
        //         u_online[0] = sol[0][6];
        //         u_online[1] = sol[0][7];
        //         // std::cout << "[Replan] u: \n";
        //         // HELPER::log_vector(u_online);
        //     }
        // }

        // True simulation
        // std::cout << "norm state error: " << state_error.norm() << "\n";
        // std::cout << "===\n";
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
            time_of_flight = sim_step;
            radius_final = pow(x[0]*x[0] + x[1]*x[1] + x[2]*x[2],2);
            break;
        }
        if(is_check_unsafe && ode_pointer->is_in_unsafe(x)){
            std::cout << "[Fail] un-safe\n";
            time_of_flight = sim_step;
            radius_final = pow(x[0]*x[0] + x[1]*x[1] + x[2]*x[2],2);
            break;
        }
        // if(state_error.norm() > 1.0){
        //     std::cout << "[Break] norm(Error) > 1.0 \n";
        //     time_of_flight = sim_step;
        //     radius_final = pow(x[0]*x[0] + x[1]*x[1] + x[2]*x[2],2);
        //     break;
        // }
        if(sim_step > 1000){
            std::cout << "[Break] tf > 1000 steps \n";
            time_of_flight = sim_step;
            radius_final = pow(x[0]*x[0] + x[1]*x[1] + x[2]*x[2],2);
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
            // std::cout << "[Success] reach goal\n";
            // std::cout << "final state (run): "; HELPER::log_vector(x);
            time_of_flight = sim_step;
            radius_final = pow(x[0]*x[0] + x[1]*x[1] + x[2]*x[2],2);
            success_flag = 1.0;
            break;
        }
    }
    if(!ode_pointer->is_goals(x, x_goals)){
        std::cout << "[Fail] cannot reach goal within nominal + " << time_elong * time_control_update << " time \n";
        // std::cout << "final state (run): ";
        // HELPER::log_vector(x);
    }
    std::string traj_runtime_file = "outputs/env"+std::to_string(env)+"_trajrun_"+RUNTIME_CONTROL+".csv";
    HELPER::write_traj_to_csv(traj_runtime, traj_runtime_file);
    std::string traj_ref_file = "outputs/env"+std::to_string(env)+"_trajref_"+RUNTIME_CONTROL+".csv";
    HELPER::write_traj_to_csv(traj_ref, traj_ref_file);
}


void test_navigation(){
    // std::string runtime_control = "nofeedback";
    // std::string runtime_control = "lqr";
    // std::string runtime_control = "tvlqr";
    // std::string runtime_control = "optneighbor";
    // std::string runtime_control = "optneighbor+replan";
    // std::string runtime_control = "tvlqr+replan";
    // std::string runtime_control = "tvlqr_largenoise";
    // std::string runtime_control = "tvlqr+replan_largenoise";
    std::string runtime_control = "test";

    int N_trials = 1;
    std::vector<Eigen::VectorXd> results;

    for(int i=0; i<N_trials; i++){
        double SF = 0.0; double TOF = 0.0; double RF = 0.0;
        control_motionplanner_and_lqr(runtime_control, SF, TOF, RF);
        std::cout << i << ", Success? " << SF << ", (norm) time of flight: " << TOF << ", (norm) final radius: " << RF << "\n";
        Eigen::VectorXd row(3);
        row[0] = SF; row[1] = TOF; row[2] = RF;
        results.push_back(row);
    }
    if(N_trials > 1){
        std::string monte_results_file = "outputs/monte_results.csv";
        HELPER::write_traj_to_csv(results, monte_results_file);
    }
}


int main(){
    std::cout << "[test SetRRT]\n";

    // test_construct_trajectory();

    // plan();

    // plan_AO();

    test_navigation();

    return 0;
}

/*

[test SetRRT]
0, Success? 1, (norm) time of flight: 504, (norm) final radius: 4.90626e-10
1, Success? 1, (norm) time of flight: 498, (norm) final radius: 1.88674e-10
2, Success? 1, (norm) time of flight: 442, (norm) final radius: 4.97776e-10
3, Success? 1, (norm) time of flight: 620, (norm) final radius: 5.12344e-10
4, Success? 1, (norm) time of flight: 474, (norm) final radius: 3.83944e-10
5, Success? 1, (norm) time of flight: 528, (norm) final radius: 3.88731e-10
6, Success? 1, (norm) time of flight: 493, (norm) final radius: 3.79202e-10
7, Success? 1, (norm) time of flight: 477, (norm) final radius: 3.99185e-10
8, Success? 1, (norm) time of flight: 521, (norm) final radius: 2.3713e-10
9, Success? 1, (norm) time of flight: 509, (norm) final radius: 5.02721e-10
10, Success? 1, (norm) time of flight: 626, (norm) final radius: 4.90759e-10
11, Success? 1, (norm) time of flight: 388, (norm) final radius: 3.37781e-10
12, Success? 1, (norm) time of flight: 348, (norm) final radius: 4.44825e-10
13, Success? 1, (norm) time of flight: 656, (norm) final radius: 3.56642e-10
14, Success? 1, (norm) time of flight: 727, (norm) final radius: 5.09e-10
15, Success? 1, (norm) time of flight: 618, (norm) final radius: 2.30853e-10
16, Success? 1, (norm) time of flight: 481, (norm) final radius: 5.14714e-10
17, Success? 1, (norm) time of flight: 492, (norm) final radius: 2.40318e-10
18, Success? 1, (norm) time of flight: 595, (norm) final radius: 4.16507e-10
19, Success? 1, (norm) time of flight: 511, (norm) final radius: 3.24768e-10
root@ecb9e476b118:/develop# ./build_and_compile.sh
-- Configuring done (0.0s)
-- Generating done (0.0s)
-- Build files have been written to: /develop/build
[ 12%] Building CXX object CMakeFiles/test_solarsail.dir/src/test_solarsail.cpp.o
[ 25%] Linking CXX executable bin/test_solarsail
[100%] Built target test_solarsail
root@ecb9e476b118:/develop# ./build/bin/test_solarsail
[test SetRRT]
0, Success? 1, (norm) time of flight: 438, (norm) final radius: 3.55726e-10
1, Success? 1, (norm) time of flight: 586, (norm) final radius: 1.8025e-10
2, Success? 1, (norm) time of flight: 463, (norm) final radius: 3.23695e-10
3, Success? 1, (norm) time of flight: 476, (norm) final radius: 2.51076e-10
4, Success? 1, (norm) time of flight: 525, (norm) final radius: 3.71151e-10
5, Success? 1, (norm) time of flight: 679, (norm) final radius: 5.05925e-10
6, Success? 1, (norm) time of flight: 567, (norm) final radius: 3.59283e-10
7, Success? 1, (norm) time of flight: 375, (norm) final radius: 2.14329e-10
8, Success? 1, (norm) time of flight: 431, (norm) final radius: 4.25018e-10
9, Success? 1, (norm) time of flight: 474, (norm) final radius: 2.15454e-10
10, Success? 1, (norm) time of flight: 537, (norm) final radius: 5.13344e-10
11, Success? 1, (norm) time of flight: 581, (norm) final radius: 2.68117e-10
12, Success? 1, (norm) time of flight: 567, (norm) final radius: 4.96162e-10
13, Success? 1, (norm) time of flight: 539, (norm) final radius: 2.77505e-10
14, Success? 1, (norm) time of flight: 556, (norm) final radius: 4.99964e-10
15, Success? 1, (norm) time of flight: 452, (norm) final radius: 2.82576e-10
16, Success? 1, (norm) time of flight: 552, (norm) final radius: 3.72304e-10
17, Success? 1, (norm) time of flight: 453, (norm) final radius: 3.94004e-10
18, Success? 1, (norm) time of flight: 497, (norm) final radius: 3.44163e-10
19, Success? 1, (norm) time of flight: 527, (norm) final radius: 3.37008e-10
20, Success? 1, (norm) time of flight: 427, (norm) final radius: 4.02424e-10
21, Success? 1, (norm) time of flight: 596, (norm) final radius: 4.09177e-10
22, Success? 1, (norm) time of flight: 601, (norm) final radius: 3.7434e-10
23, Success? 1, (norm) time of flight: 430, (norm) final radius: 4.34113e-10
24, Success? 1, (norm) time of flight: 501, (norm) final radius: 4.37134e-10
25, Success? 1, (norm) time of flight: 563, (norm) final radius: 3.0095e-10
26, Success? 1, (norm) time of flight: 645, (norm) final radius: 3.16759e-10
27, Success? 1, (norm) time of flight: 460, (norm) final radius: 2.98098e-10
[Break] norm(Error) > 1.0 
[Fail] cannot reach goal within nominal + 0 time 
28, Success? 0, (norm) time of flight: 568, (norm) final radius: 7.01581e-09
29, Success? 1, (norm) time of flight: 472, (norm) final radius: 3.68087e-10
30, Success? 1, (norm) time of flight: 378, (norm) final radius: 3.89689e-10
31, Success? 1, (norm) time of flight: 514, (norm) final radius: 4.04777e-10
32, Success? 1, (norm) time of flight: 585, (norm) final radius: 3.61525e-10
33, Success? 1, (norm) time of flight: 533, (norm) final radius: 4.019e-10
34, Success? 1, (norm) time of flight: 408, (norm) final radius: 2.97086e-10
35, Success? 1, (norm) time of flight: 418, (norm) final radius: 2.59749e-10
36, Success? 1, (norm) time of flight: 499, (norm) final radius: 4.94474e-10
37, Success? 1, (norm) time of flight: 589, (norm) final radius: 4.76933e-10
38, Success? 1, (norm) time of flight: 507, (norm) final radius: 2.22383e-10
39, Success? 1, (norm) time of flight: 590, (norm) final radius: 5.05652e-10

*/