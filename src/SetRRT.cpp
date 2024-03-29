#include "SetRRT.h"

std::vector<Eigen::VectorXd> SetRRT::plan(const Eigen::VectorXd& x_init, 
                                          const std::vector<Eigen::VectorXd> x_goals){
    init_plan(x_init);

    // start timer
    auto startTime = std::chrono::high_resolution_clock::now();

    std::vector<Eigen::VectorXd> solution;

    while(true){
        int node_select = select_node();
        // std::cout << "[DEBUG] select_node: " << node_select << "\n";

        Eigen::VectorXd control_sample = get_control_sample(
                                    ode_solver_pointer->ode_pointer->u_min, 
                                    ode_solver_pointer->ode_pointer->u_max);
        // std::cout << "[DEBUG] control sample:"; print_eigen_vector(control_sample);

        Eigen::VectorXd* state_new_pointer = get_state_new_pointer(node_select, control_sample, x_goals);
        if(state_new_pointer){
            Eigen::VectorXd state_new = (*state_new_pointer);
            // std::cout << "[DEBUG] state new:"; print_eigen_vector(state_new);

            connect(node_select, control_sample, state_new);

            if(ode_solver_pointer->ode_pointer->is_goals(state_new, x_goals)){
                is_success = true;
            }
        }

        // break planning
        // 1. success
        if(is_success){
            std::cout << "[IsGoal:True]\n";
            solution = construct_solution();
            return solution;
        }
        if(exceed_plan_time_max(startTime)){
            break;
        }
    }// end while loop

    return solution;
}


std::vector<Eigen::VectorXd> SetRRT::construct_trajectory(const std::vector<Eigen::VectorXd>& solution, 
                                                          const std::vector<Eigen::VectorXd> x_goals,
                                                          bool is_process_noise){

    std::vector<Eigen::VectorXd> trajectory;
    Eigen::VectorXd row0 = solution[0];
    Eigen::VectorXd x(size_x);
    for(int i=0; i<size_x; i++){
        x[i] = row0[i];
    }

    for(int i=0; i<solution.size(); i++){
        if(i < solution.size()-1){
            Eigen::VectorXd sol_i = solution[i];
            Eigen::VectorXd control = sol_i.segment(size_x, size_u);
            Eigen::VectorXd u = control.head(size_u-1);
            double time_of_control = control[size_u-1];
            std::vector<Eigen::VectorXd> traj_segment;
            traj_segment =  ode_solver_pointer->solver_runge_kutta(
                                        x, 
                                        u, 
                                        ode_solver_pointer->time_integration, 
                                        time_of_control, 
                                        x_goals, 
                                        is_process_noise);
            x = traj_segment.back();

            // write to trajectory data (state, control)
            for(auto& x_k : traj_segment){
                Eigen::VectorXd x_k_holder = x_k;
                x_k.conservativeResize(x.size()+u.size());
                x_k << x_k_holder, u;
            }
            if(!trajectory.empty()){
                trajectory.pop_back();
            }
            trajectory.insert(trajectory.end(), traj_segment.begin(), traj_segment.end());
        }
        else{
            std::cout << "goal state (sol): "; print_eigen_vector(solution[solution.size()-1]);
            std::cout << "goal state (sim): "; print_eigen_vector(x);
            // double traj_sol_difference = (x - solution[i].head(size_x)).norm();
            // std::cout << "difference between trajectory and solution: " << traj_sol_difference << "\n";
        }
    }
    return trajectory;
}


double SetRRT::get_cost(){
    const int node_goal = nodes.size()-1;
    return node_to_cost[node_goal];
}


void SetRRT::link_ode_solver_pointer(OdeSolver* pointer){
    ode_solver_pointer = pointer;
}


void SetRRT::init_plan(const Eigen::VectorXd& x_init){
    std::cout << "plan time max: " << plan_time_max << "\n";
    is_success = false;
    graph.cleanGraph();
    nodes.clear();
    node_to_state.clear();
    node_to_cost.clear();
    nodes.push_back(0);
    node_to_state[0] = x_init;
    node_to_cost[0] = 0.0;
}


Eigen::VectorXd SetRRT::get_control_sample(const Eigen::VectorXd u_min, const Eigen::VectorXd u_max){
    int size_u = u_min.size();
    Eigen::VectorXd u(size_u);
    for(int i=0; i<size_u; i++){
        if(i == size_u-1){
            // control duration
            int N = int(u_max[i]/u_min[i]);
            u[i] = getRandomIntBounded(0, N) * u_min[i];
        }
        else{
            // control signal
            double step = (u_max[i] - u_min[i])/(control_resolution);
            u[i] = getRandomIntBounded(0, control_resolution)*step + u_min[i];
            // TEMP (round to 2 digits)
            double multiplier = std::pow(10.0, 2);
            u[i] = std::round(u[i] * multiplier) / multiplier;
        }
    }
    return u;
}


int SetRRT::select_node(){
    if(nodes.size() < 1){
        std::cout << "[ERROR] no nodes to select\n";
        return -1;
    }
    int node = getRandomInt(nodes.size()); //std::cout << "MySetRRT.select_node() " << node << "\n";
    return node;
}


Eigen::VectorXd* SetRRT::get_state_new_pointer(const int node_select, 
                                               Eigen::VectorXd& control_sample, 
                                               const std::vector<Eigen::VectorXd>& x_finals){
    const Eigen::VectorXd state_select = node_to_state[node_select];
    // std::cout << "[DEBUG] state select\n"; // print_eigen_vector(state_select);
    const int size_control = control_sample.size();
    Eigen::VectorXd u(size_control-1); // exclude the control time duration
    for(int i=0; i<u.size(); i++){
        u[i] = control_sample[i];
    }
    // std::cout << "[DEBUG] u\n"; // print_eigen_vector(u);
    std::vector<Eigen::VectorXd> x_traj = ode_solver_pointer->solver_runge_kutta(
                                            state_select, u, 
                                            ode_solver_pointer->time_integration, control_sample[size_control-1],
                                            x_finals);
    if(x_traj.empty()){
         return nullptr;
    }
    if(node_to_cost[node_select] + control_sample[size_u-1] > cost_threshold){
        // time of control to the new node
        return nullptr;
    }

    Eigen::VectorXd state_new;
    Eigen::VectorXd* state_new_pointer = new Eigen::VectorXd(state_new.size());
    state_new = x_traj.back();
    // std::cout << "[DEBUG] state_new\n"; // print_eigen_vector(state_new);
    (*state_new_pointer) = state_new;
    return state_new_pointer;
}


void SetRRT::connect(const int node_select, const Eigen::VectorXd control_apply, const Eigen::VectorXd state_new){
    const Eigen::VectorXd state_select = node_to_state[node_select];
    int node_new = nodes.size();
    nodes.push_back(node_new);
    node_to_state[node_new] = state_new;
    // std::cout << "[DEBUG] connect: " << node_new << " to " << node_select << "\n";
    // std::cout << "state new:"; print_eigen_vector(state_new);
    graph.connect(node_new, node_select);
    std::pair<int, int> edge(node_select, node_new);
    edge_to_control[edge] = control_apply;

    double cost = node_to_cost[node_select] + control_apply[size_u-1];
    node_to_cost[node_new] = cost;
}


std::vector<Eigen::VectorXd> SetRRT::construct_solution(){
    std::vector<int> solution_nodes;
    int node_current = nodes.size()-1;

    while(true){        
        solution_nodes.insert(solution_nodes.begin(), node_current);
        int node_parent = graph.findParent(node_current);
        if(node_parent < 0){
            break;
        }
        node_current = node_parent;
    }

    std::cout << "[DEBUG] solution nodes: ";

    std::vector<Eigen::VectorXd> solution;
    for(int k=0; k<solution_nodes.size(); k++){
        int n = solution_nodes[k];
        int n_next = solution_nodes[k+1];
        Eigen::VectorXd sol(size_x + size_u + 1);
        Eigen::VectorXd state = node_to_state[n];
        sol.head(size_x) = state;
        Eigen::VectorXd control = Eigen::VectorXd::Zero(size_u);
        if(n != nodes.size()-1){
            std::pair<int, int> edge(n, n_next);
            control = edge_to_control[edge];
        }
        sol.segment(size_x, size_u) = control;
        sol[size_x+size_u] = n;
        solution.push_back(sol);
        std::cout << n << " ";
    }
    std::cout << "\n";

    return solution;
}


void SetRRT::print_nodes_and_states(){
    for (const auto& n : nodes) {
        std::cout << "n:";
        print_eigen_vector(node_to_state[n]);
    }
}

