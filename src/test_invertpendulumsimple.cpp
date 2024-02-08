#include<iostream>
#include <cmath>
#include <tuple>
#include "OdeVirtual.h"
#include "OdeInvertPendulumSimple.h"
#include "OdeSolver.h"
#include "PlannerVirtual.h"
#include "SetRRT.h"
#include "helperfunctions.h"


// Next step: stabilizing the pendulum over T time.
// 1. simply replanning does not work.


std::tuple<OdeVirtual* , PlannerVirtual*, Eigen::VectorXd, std::vector<Eigen::VectorXd> > define_problem(){
    // Define ode
    OdeInvertPendulumSimple* ode_pointer = new OdeInvertPendulumSimple("InvertPendulumSimple");
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
    ode_pointer->set_domain(x_min, x_max, u_min, u_max);
    // ode_marine_vessel.set_unsafecircles(2, CONFIG_MARINE_VESSEL::get_random_unsafe_centers(15, 0, 10), 
    //                                        CONFIG_MARINE_VESSEL::get_random_unsafe_raidus(15, 0.25, 0.5));
    Eigen::VectorXd process_mean(2);  process_mean << 0.0, 0.0;
    Eigen::MatrixXd process_cov(2,2); process_cov  << 1.0, 0.0,
                                                      0.0, 0.0;
    ode_pointer->set_process_noise(process_mean, process_cov);

    // // Define Unsafe for Visualization
    // std::string unsafe_file = "outputs/" + ode_pointer->ode_name + "_unsafe.csv";
    // HELPER::write_traj_to_csv(ode_marine_vessel.output_unsafecircles(), unsafe_file);

    // Define ode solver
    const double time_integration = 1e-2;
    OdeSolver* ode_solver_ptr = new OdeSolver(time_integration);
    ode_solver_ptr->link_ode_pointer(ode_pointer);

    // // Validate ode
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

    // Define planner
    SetRRT* planner_pointer = new SetRRT("SetRRT");
    planner_pointer->control_resolution = 100.0;
    planner_pointer->link_ode_solver_pointer(ode_solver_ptr);

    // Define Goals
    std::vector<Eigen::VectorXd> x_goals; // (theta, w, x, v)
    Eigen::VectorXd x1_goal(4); x1_goal << 0.0, 0.0, 0.0, 0.0; x_goals.push_back(x1_goal);
    Eigen::VectorXd delta_to_goal(4); delta_to_goal << 0.25, 0.25, 0.1, 0.1;
    ode_pointer->delta_to_goal = delta_to_goal;
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
    std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol, x_goals);
    std::string traj_file = "outputs/" + planner_pointer->planner_name + "_" 
                            + ode_pointer->ode_name + "_traj.csv";
    HELPER::write_traj_to_csv(traj, traj_file); // HELPER::log_trajectory(traj);

    // // Construct Controlled Trajectory subject to Process noise
    // std::vector<Eigen::VectorXd> traj_noise = planner_pointer->construct_trajectory(sol, {}, true);
    // std::string traj_noise_file = "outputs/" + planner_pointer->planner_name + "_" + ode_pointer->ode_name + "_noise_traj.csv";
    // HELPER::write_traj_to_csv(traj_noise, traj_noise_file); // HELPER::log_trajectory(traj_noise);
    // HELPER::log_vector(traj_noise[traj_noise.size()-1]);
}


void plan_ao(){
    auto [ode_pointer, planner_pointer, x_start, x_goals] = define_problem();    
    std::cout << "ode: " << ode_pointer->ode_name << "\n";
    std::cout << "planner: " << planner_pointer->planner_name << "\n";

    double plan_time_max = 500.0;
    int N_run = 20;
    planner_pointer->set_plan_time_max(plan_time_max);
    // planner_pointer->set_cost_threshold(20.00);

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
            std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol, x_goals); 
            // HELPER::log_trajectory(traj);
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


void replan_fixedupdate_realtime(){
    // Assume the planning time is instantaneous
    auto [ode_pointer, planner_pointer, x_start, x_goals] = define_problem();    
    std::cout << "ode: " << ode_pointer->ode_name << "\n";
    std::cout << "planner: " << planner_pointer->planner_name << "\n";
    const int size_x = 4;
    const int size_u = 2;

    const double time_simulation = 50.0;
    double time_control_update = 0.1;
    std::vector<Eigen::VectorXd> traj;
    std::vector<Eigen::VectorXd> state_control_traj;
    Eigen::VectorXd x0 = x_start;

    for(int i=0; i<int(time_simulation/time_control_update); i++){
        // Plan
        std::vector<Eigen::VectorXd> sol = planner_pointer->plan(x0, x_goals);

        // Get first control
        Eigen::VectorXd control(size_u-1);
        for(int i=0; i<size_u-1; i++){
            control[i] = sol[0][size_x+i];
        }

        // State and control pair
        Eigen::VectorXd state_and_control;
        state_and_control.conservativeResize(x0.size() + control.size());
        state_and_control << x0, control;
        state_control_traj.push_back(state_and_control);

        //Execute with noise
        std::vector<Eigen::VectorXd> traj_segment;
        traj_segment = planner_pointer->ode_solver_pointer->solver_runge_kutta(
                                        x0, 
                                        control, 
                                        planner_pointer->ode_solver_pointer->time_integration, 
                                        time_control_update, 
                                        {}, 
                                        false);
        // check if out-of-domain
        if(traj_segment.empty()){
            break;
        }
        // update
        traj.insert(traj.end(), traj_segment.begin(), traj_segment.end());
        x0 = traj_segment.back();
        std::cout << "time update to: " << (i+1)*time_control_update << "\n";
        std::cout << "state update to:"; HELPER::log_vector(x0);
    }
    std::string traj_file = "outputs/" + planner_pointer->planner_name + "_" 
                            + ode_pointer->ode_name + "_realtime_traj.csv";
    HELPER::write_traj_to_csv(traj, traj_file);

    std::string state_control_file = "outputs/" + planner_pointer->planner_name + "_" 
                            + ode_pointer->ode_name + "_statecontrol.csv";
    HELPER::write_traj_to_csv(state_control_traj, state_control_file);
}


void replan_adaptupdate_realtime(){
    // Assume the planning time is instantaneous
    auto [ode_pointer, planner_pointer, x_start, x_goals] = define_problem();    
    std::cout << "ode: " << ode_pointer->ode_name << "\n";
    std::cout << "planner: " << planner_pointer->planner_name << "\n";
    const int size_x = 4;
    const int size_u = 2;

    const double time_simulation = 50.0;
    std::vector<Eigen::VectorXd> traj;
    std::vector<Eigen::VectorXd> state_control_traj;
    Eigen::VectorXd x0 = x_start;
    double time = 0.0;

    while(true){
        // Plan
        std::vector<Eigen::VectorXd> sol = planner_pointer->plan(x0, x_goals);
        std::cout << "sol0:"; HELPER::log_vector(sol[0]);

        // Get first control
        Eigen::VectorXd control(size_u-1);
        for(int i=0; i<size_u-1; i++){
            control[i] = sol[0][size_x+i];
        }

        double time_control_update = sol[0][size_x + size_u -1];
        if(time_control_update < 0.1){
            time_control_update = 0.1;
        }

        // State and control pair
        Eigen::VectorXd state_and_control;
        state_and_control.conservativeResize(x0.size() + control.size());
        state_and_control << x0, control;
        state_control_traj.push_back(state_and_control);

        //Execute with noise
        std::vector<Eigen::VectorXd> traj_segment;
        traj_segment = planner_pointer->ode_solver_pointer->solver_runge_kutta(
                                        x0, 
                                        control, 
                                        planner_pointer->ode_solver_pointer->time_integration, 
                                        time_control_update, 
                                        {}, 
                                        false);
        // check if out-of-domain
        if(traj_segment.empty()){
            break;
        }
        // update
        traj.insert(traj.end(), traj_segment.begin(), traj_segment.end());
        x0 = traj_segment.back();
        time = time + time_control_update;
        std::cout << "time update to: " << time << "\n";
        std::cout << "state update to:"; HELPER::log_vector(x0);
        // check if simulation time is up
        if(time >= time_simulation){
            break;
        }
    }

    std::string traj_file = "outputs/" + planner_pointer->planner_name + "_" 
                            + ode_pointer->ode_name + "_realtime_traj.csv";
    HELPER::write_traj_to_csv(traj, traj_file);

    std::string state_control_file = "outputs/" + planner_pointer->planner_name + "_" 
                            + ode_pointer->ode_name + "_statecontrol.csv";
    HELPER::write_traj_to_csv(state_control_traj, state_control_file);
}


void control_lqr(){
    // NOTE: if x - x_ref is huge, then the lqr controller might fail...
    // the problem is the cart x error, this error grows bigger and then it affects the delta_u
    // We can clearly see this if we set Q(2,2) = 0.0; That is we don't care about the cart x error.
    // Then the delta_u will only do minor modifications.
    // Question: how to select the virtual x_ref(t)?
    const int size_x = 4;
    const int size_u = 2; // (control inputs, time_duration)

    auto [ode_pointer, planner_pointer, x_start, x_goals] = define_problem();    
    std::cout << "ode: " << ode_pointer->ode_name << "\n";
    std::cout << "planner: " << planner_pointer->planner_name << "\n";

    // A. directly read trajectiory
    std::string trajectory_file;
    trajectory_file = "outputs/SetRRT_InvertPendulumSimple_cost:12.10_traj.csv";
    std::vector<Eigen::VectorXd> traj = HELPER::read_csv_data(trajectory_file);
    // // B. Plan
    // std::vector<Eigen::VectorXd> sol = planner_pointer->plan(x_start, x_goals);
    // double cost = planner_pointer->get_cost();
    // std::cout << "[DEBUG] cost: " << cost << "\n";
    // HELPER::log_trajectory(sol);
    // std::string sol_file = "outputs/" + planner_pointer->planner_name + "_" 
    //                         + ode_pointer->ode_name + "_sol.csv";
    // HELPER::write_traj_to_csv(sol, sol_file);
    // // Construct and write trajectory
    // std::vector<Eigen::VectorXd> traj = planner_pointer->construct_trajectory(sol, x_goals);
    // std::string traj_file = "outputs/" + planner_pointer->planner_name + "_" 
    //                         + ode_pointer->ode_name + "_traj.csv";
    // HELPER::write_traj_to_csv(traj, traj_file); // HELPER::log_trajectory(traj);

    
    // Compute linear dynamics
    double time_control_update = 0.1;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(size_x, size_x);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(1, 1);
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(size_x, size_x);
    Q(0,0) = 1.0; Q(1,1) = 1.0; Q(2,2) = 1.0; Q(3,3) = 1.0;
    R(0,0) = 10.0;

    // Compute nominal control trajecotory
    std::vector<Eigen::VectorXd> traj_nominal;
    for(int i=0; i<traj.size(); i+=10){
        traj_nominal.push_back(traj[i]);
    }

    // Init
    Eigen::VectorXd x = x_start;
    std::vector<Eigen::VectorXd> traj_runtime;
    int time_elong = 300;
    bool is_process_noise = true;

    for(int i=0; i<traj_nominal.size()+time_elong; i++){
        std::cout << "state: "; HELPER::log_vector(x); // real state

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
        Eigen::VectorXd u_ref = x_u.segment(size_x,size_u-1); //std::cout << "u ref: "; HELPER::log_vector(u_ref);

        // Compute u_online
        Eigen::VectorXd u_online = u_ref;
        auto [F, G] = ode_pointer->get_linear_dynamics_matrices(x_ref, u_ref, time_control_update);
        // HELPER::log_matrix(F); HELPER::log_matrix(G);
        bool lqr_solved = HELPER::solveRiccatiIterationD(F, G, Q, R, P);
        if(lqr_solved){
            // HELPER::log_matrix(P);
            Eigen::MatrixXd K = (R+G.transpose()*P*G).inverse()*G.transpose()*P*F;
            // std::cout << "K: "; HELPER::log_matrix(K);

            // State error: NOTE: this is the problem now. (When process noise exists)
            // consider that x_ref is pi, and x is -pi. In physics, thesea are the same angle,
            // but in math, these are two different angle...
            Eigen::VectorXd state_error(size_x);
            double angle_diff = HELPER::angleDifference(x[0], x_ref[0]);
            state_error << angle_diff, x[1]-x_ref[1], x[2]-x_ref[2], x[3]-x_ref[3];                    
            std::cout << "state error: "; HELPER::log_vector(state_error);

            Eigen::VectorXd delta_u_lqr = -K*(state_error);
            std::cout << "delta u lqr: "; HELPER::log_vector(delta_u_lqr);
            u_online = u_online + delta_u_lqr; 
            // NOTE: this u_online can grow out of bound
            // (1) bound u_online to feasible control domain
            // of (2) bound the state error (Referenence Governer)
            u_online = u_online.cwiseMax(-Eigen::VectorXd::Ones(u_online.size()))
                                .cwiseMin(Eigen::VectorXd::Ones(u_online.size()));
        }
        std::cout << "u online: "; HELPER::log_vector(u_online);


        //Execute with noise
        std::vector<Eigen::VectorXd> traj_segment;
        traj_segment = planner_pointer->ode_solver_pointer->solver_runge_kutta(
                                        x, 
                                        u_online, 
                                        planner_pointer->ode_solver_pointer->time_integration, 
                                        time_control_update, 
                                        {}, 
                                        is_process_noise);
        // check if out-of-domain
        if(traj_segment.empty()){
            std::cout << "[DEBUG] out-of-domain\n";
            break;
        }
        // update
        x = traj_segment.back();
        // write
        if(!traj_runtime.empty()){
            traj_runtime.pop_back();
        }
        traj_runtime.insert(traj_runtime.end(), traj_segment.begin(), traj_segment.end());
    }
    std::string traj_runtime_file = "outputs/" + planner_pointer->planner_name + "_" 
                            + ode_pointer->ode_name + "_noise_traj(lqr).csv";
    HELPER::write_traj_to_csv(traj_runtime, traj_runtime_file); // HELPER::log_trajectory(traj);

}


int main(){
    std::cout << "[test invertpendulumsimple]\n";

    std::cout << "[in docker develop env]\n";

    // plan();

    // plan_ao();

    // replan_fixedupdate_realtime();

    // replan_adaptupdate_realtime();

    control_lqr();

    return 0;
}