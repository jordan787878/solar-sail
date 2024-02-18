#include "MDP.h"
#include <cmath>
#include <limits>
#include <algorithm>

MDP::MDP(PlannerVirtual* pointer, const int n_per_state, const int n_per_action){
    planner_pointer = pointer;
    size_x = planner_pointer->size_x;
    size_u = planner_pointer->size_u - 1; // to remove the time duraiton
    number_per_state = n_per_state;
    number_per_action = n_per_action;
    number_Q = pow(number_per_state, size_x);
    number_Sigma = pow(number_per_action, size_u);

    dx = Eigen::VectorXd::Zero(size_x);
    for(int i=0; i<size_x; i++){
        dx[i] = (planner_pointer->ode_solver_pointer->ode_pointer->x_max[i]
                - planner_pointer->ode_solver_pointer->ode_pointer->x_min[i])/ number_per_state;
        std::cout << "[DEBUG] dx " << dx[i] << "\n";
    }

    du = Eigen::VectorXd::Zero(size_u);
    for(int i=0; i<size_u; i++){
        du[i] = (planner_pointer->ode_solver_pointer->ode_pointer->u_max[i]
                - planner_pointer->ode_solver_pointer->ode_pointer->u_min[i])/ number_per_action;
        std::cout << "[DEBUG] du " << du[i] << "\n";
    }

    // sparse_matrix_pointer = new SparseMatrix(number_Q*number_Sigma, number_Q);
}


void MDP::construct_discrete_state(const std::vector<Eigen::VectorXd>& traj){
    // intersection with trajectory
    nodes_Q.clear();
    for(const auto& s : traj){
        Eigen::VectorXd x = s.head(size_x); //log_vector(x);
        std::vector<unsigned int> q = map_x_to_q(x); //log_vector(q);
        unsigned int Nq = map_q_to_Nq(q); //std::cout << Nq << "\t" << number_Q << "\n";
        // Use std::find to check if the element exists in the vector
        auto it = std::find(nodes_Q.begin(), nodes_Q.end(), Nq);
        if(it == nodes_Q.end()){
            // std::cout << "Element " << Nq << " not found in the vector." << std::endl;
            nodes_Q.push_back(Nq);
        }
    }

    // intersection with x goals (sample the goal region)
    for(int i=0; i<50; i++){
        Eigen::VectorXd x(size_x);
        for(int j=0; j<size_x; j++){
            x[j] = getRandomDouble(x_goals[j][0], x_goals[j][1]);
        }
        std::vector<unsigned int> q = map_x_to_q(x); //log_vector(q);
        unsigned int Nq = map_q_to_Nq(q); //std::cout << Nq << "\t" << number_Q << "\n";
        // Use std::find to check if the element exists in the vector
        auto it = std::find(nodes_Q.begin(), nodes_Q.end(), Nq);
        if(it == nodes_Q.end()){
            // std::cout << "Element " << Nq << " not found in the vector." << std::endl;
            nodes_Q.push_back(Nq);
        }
    }

    // store these pivot nodes
    std::vector<unsigned int> nodes_Q_pivot;
    nodes_Q_pivot = nodes_Q;
    add_Q_neighbors(nodes_Q_pivot);
    nodes_Q_pivot = nodes_Q;
    add_Q_neighbors(nodes_Q_pivot);

    // construct the goal node mapping: Nq --> bool
    // init to true
    for(int i=0; i<nodes_Q.size(); i++){
        map_to_is_goal_node[i] = true;
    }
    // Set to false
    for(int i=0; i<nodes_Q.size(); i++){
        Eigen::VectorXd x = map_q_to_x( map_Nq_to_q(nodes_Q[i]));
        std::vector<Eigen::VectorXd> x_s = get_boader_points(x);
        for(const auto& x_ss : x_s){
            if(!planner_pointer->ode_solver_pointer->ode_pointer->is_goals(x_ss, x_goals)){
                map_to_is_goal_node[i] = false;
                break;
            }
        }
    }
    std::cout << "goal nodes: ";
    for(int i=0; i<map_to_is_goal_node.size(); i++){
        if(map_to_is_goal_node[i]){
            std::cout << i << ",";
        }
    }
    std::cout << "\n";

    // add a fail node
    nodes_Q.push_back(number_Q);
}


std::vector<Eigen::VectorXd> MDP::debug_discrete_state(){
    std::vector<Eigen::VectorXd> traj;

    // print out
    std::cout << "size of node: " << nodes_Q.size() << "\n";
    for(const auto& Nq : nodes_Q){
        // std::cout << Nq << "\t";
        Eigen::VectorXd x = map_q_to_x(map_Nq_to_q(Nq));
        traj.push_back(x);
    }
    std::cout << "\n";
    return traj;
}

std::tuple<bool, Eigen::VectorXd> MDP::get_feedback_control(const Eigen::VectorXd& s){
    Eigen::VectorXd u = Eigen::VectorXd::Zero(size_u);
    unsigned int Nq = map_q_to_Nq(map_x_to_q(s));
    // std::cout << "[DEBUG] discrete state: " << Nq << "\n";
    for(int i=0; i<nodes_Q.size(); i++){
        if(Nq == nodes_Q[i]){
            u = discrete_state_to_control[i];
            // std::cout << "[DEBUG] u(MDP): "; log_vector(u);
            return std::make_tuple(true, u);
        }
    }
    std::cout << "[ERROR] no defined feedback control\n";
    return std::make_tuple(false, u);;
}


void MDP::construct_transition(){
    // test_mapping();
    // return;
    
    // Init transition
    const unsigned int number_finite_state = nodes_Q.size();
    is_set_transitions_success = false;

    // Transition_pointer = new SparseMatrix(number_finite_state*number_Sigma, number_finite_state);
    matrix = new std::vector<std::map<unsigned int, double>>(number_finite_state*number_Sigma);
    int number_samples = 1;
    double time_duration = 1.0;
    bool is_process_noise = false;
    bool is_check_unsafe = true;

    // for progress print-out
    unsigned int dN;
    if(nodes_Q.size() >= 100){
        dN = nodes_Q.size()/100;
    }
    else{
        dN = 1;
    }

    for(unsigned int n_q=0; n_q<number_finite_state; n_q++){
        if(n_q%dN == 0){
            std::cout << round( 100.0 * static_cast<double>(n_q) / static_cast<double>(nodes_Q.size()) ) << " %\n";
        }
        for(unsigned int n_sigma=0; n_sigma<number_Sigma; n_sigma++){
            set_transitions(n_q, n_sigma, number_samples, time_duration, is_process_noise, is_check_unsafe);
        }
    }
}


void MDP::set_transitions(unsigned int n_q, unsigned int n_sigma, int number_samples, double time_duration,
                          bool is_process_noise, bool is_check_unsafe){
    // get Nq
    unsigned int Nq = nodes_Q[n_q];
    if(Nq == number_Q){
        // std::cout << "[DEBUG] fail node, no transition needed\n";
        return;
    }
    //std::cout << n_q << "-th node is: " << Nq << "\t";

    // get x
    Eigen::VectorXd x = map_q_to_x(map_Nq_to_q(Nq));
    //std::cout << "x: "; log_vector(x);

    // get x_s
    std::vector<Eigen::VectorXd> x_s = get_boader_points(x);
    x_s.push_back(x);

    // get Nsigma
    unsigned int Nsigma = n_sigma;
    // get u
    Eigen::VectorXd u = map_sigma_to_u( map_Nsigma_to_sigma(Nsigma));
    //std::cout << n_sigma << " -th u is: "; log_vector(u);

    // get index row
    unsigned int index_row = number_Sigma * n_q + n_sigma;
    //std::cout << "i(row): " << index_row << "\n";

    // outerloop: noise sample, inner loop: state sample
    const int number_boader_points = x_s.size();
    const int number_total_samples = number_samples * number_boader_points;

    for(int i=0; i<number_samples; i++){
        for(int j=0; j<number_boader_points; j++){
            std::vector<Eigen::VectorXd> traj_segment;
            traj_segment = planner_pointer->ode_solver_pointer->solver_runge_kutta(
                                            x_s[j], 
                                            u, 
                                            planner_pointer->ode_solver_pointer->time_integration, 
                                            time_duration, 
                                            x_goals, 
                                            is_process_noise,
                                            is_check_unsafe);
            // if feasible new state exists
            if(!traj_segment.empty()){
                // get x_new
                Eigen::VectorXd x_new = traj_segment.back();

                // map to Nq_new
                std::vector<unsigned int> q_new = map_x_to_q(x_new);
                unsigned int Nq_new = map_q_to_Nq(q_new);
                // std::cout << "Nq new: " << Nq_new << "\t";

                // check if Nq new is in the finite state and get the corresponding index.
                unsigned int index_Nq_new;
                auto it = find(nodes_Q.begin(), nodes_Q.end(), Nq_new);
                if(it != nodes_Q.end()){
                    // std::cout << j << "-set transition: " << Nq << " " << Nsigma << " " << Nq_new << "\n";
                    index_Nq_new = it - nodes_Q.begin();
                }
                else{
                    // std::cout << j << "-set transition (fail node): " << Nq << " " << Nsigma << " " << number_Q << "\n";
                    index_Nq_new = nodes_Q.size()-1;
                }
                // std::cout << "i(Nq new): " << index_Nq_new;

                if(!is_set_transitions_success && map_to_is_goal_node[index_Nq_new] && n_q != index_Nq_new){
                    is_set_transitions_success = true;
                }

                // get the value
                double value = (*matrix)[index_row][index_Nq_new];
                // std::cout << "\texisting value: " << value;
                value = value + 1.0/static_cast<double>(number_total_samples);
                // update the value
                (*matrix)[index_row][index_Nq_new] = value;
                // std::cout << "\tafter update: " << (*matrix)[index_row][index_Nq_new] << "\n";
            }
        }
    }
    // std::cout << "\n";
}


void MDP::value_iteration(){
    // [TEMP] MDP for testing
    // nodes_Q.clear();
    // number_Sigma = 2;
    // matrix->clear();
    // nodes_Q = {0, 1, 2, 3, 4};
    // const unsigned int number_finite_state = nodes_Q.size();
    // matrix = new std::vector<std::map<unsigned int, double>>(number_finite_state*number_Sigma);
    // (*matrix)[0][1] = 1.0; (*matrix)[1][2] = 1.0; (*matrix)[2][2] = 1.0; (*matrix)[3][3] = 1.0;
    // (*matrix)[4][3] = 1.0; (*matrix)[5][4] = 1.0; (*matrix)[6][4] = 1.0; (*matrix)[7][3] = 1.0;
    // print_transition();
    // map_to_is_goal_node.clear();
    // for(int i=0; i<nodes_Q.size(); i++){
    //     map_to_is_goal_node[i] = false;
    // }
    // map_to_is_goal_node[3] = true;

    const double converge_epsilon = 0.01;

    // Value iteration (1)
    J = new std::vector<double>(nodes_Q.size(), 0.0);

    for(int i=0; i<nodes_Q.size(); i++){
        if(map_to_is_goal_node[i]){
            (*J)[i] = reward(i, 0);
        }
    }
    J->back() = reward(nodes_Q.size()-1, 0);
    // std::cout << "J(0): "; print_J((*J));

    // iteration k
    for(int k=0; k<200; k++){
        double epsilon = -std::numeric_limits<double>::infinity();

        for(int i=0; i<nodes_Q.size()-1; i++){
            if(!map_to_is_goal_node[i]){
                // Init
                double optimal_value_J_q_sigma = -std::numeric_limits<double>::infinity();
                for(int j=0; j<number_Sigma; j++){
                    unsigned int Nsigma = j;
                    unsigned int index_row = number_Sigma * i + j;
                    double value_J_q_sigma = reward(i, 0);
                    for (const auto& entry : (*matrix)[index_row]) {
                        //std::cout << "(" << index_row << ", " << entry.first << "): " << entry.second << "\t";
                        unsigned int index_col = entry.first;
                        double probability = entry.second;
                        value_J_q_sigma = value_J_q_sigma + gamma * probability* (*J)[index_col];
                        // if((*J)[index_col] > 0){
                        //     std::cout << i << " " << probability << " " << (*J)[index_col] << " " << value_J_q_sigma << "\n";
                        // }
                    }
                    if(value_J_q_sigma > optimal_value_J_q_sigma){
                        optimal_value_J_q_sigma = value_J_q_sigma;
                    }
                }
                // compute epsilon i
                double error_i = std::abs(optimal_value_J_q_sigma - (*J)[i]);
                if(error_i > epsilon){
                    epsilon = error_i;
                }
                // update J[i]
                // std::cout << i << " " << optimal_value_J_q_sigma << "\n";
                (*J)[i] = optimal_value_J_q_sigma;
            }
        }

        // iteration end
        // std::cout << k+1 << " == max error === " << epsilon << "\n";
        // std::cout << "J(" << k+1 << "): "; print_J(*J);
        // std::cout << "\n";

        if(epsilon < converge_epsilon){
            std::cout << "value iteration converage\n";
            break;
        }
    }
    // std::cout << "J: "; print_J(*J);
}


void MDP::synthesize_control(){
    for(int i=0; i<nodes_Q.size()-1; i++){
        if(!map_to_is_goal_node[i]){
            unsigned int optimal_Nsigma = 0;
            double optimal_value_J_q_sigma = -std::numeric_limits<double>::infinity();
            for(int j=0; j<number_Sigma; j++){
                unsigned int index_row = number_Sigma * i + j;
                double value_J_q_sigma = reward(i, 0);
                for (const auto& entry : (*matrix)[index_row]) {
                    unsigned int index_col = entry.first;
                    double probability = entry.second;
                    value_J_q_sigma = value_J_q_sigma + gamma * probability * (*J)[index_col];
                }
                if(value_J_q_sigma > optimal_value_J_q_sigma){
                    optimal_value_J_q_sigma = value_J_q_sigma;
                    optimal_Nsigma = j;
                }
            }
            Eigen::VectorXd u = map_sigma_to_u(map_Nsigma_to_sigma(optimal_Nsigma));
            discrete_state_to_control[i] = u;
        }
        else{
            Eigen::VectorXd u = Eigen::VectorXd::Zero(size_u);
            discrete_state_to_control[i] = u;
        }
    }

    // testing
    int i = 0;
    log_vector(discrete_state_to_control[i]);
}


std::vector<unsigned int> MDP::map_Nq_to_q(const unsigned int& Nq){
    // fail node
    if(Nq == number_Q){
        std::vector<unsigned int> q(size_x);
        for(int i=0; i<size_x; i++){
            q[i] = number_Q;
        }
        return q;
    }
    // normal nodes
    std::vector<unsigned int> q(size_x);
    unsigned int N = Nq;
    for(unsigned int i=0; i<size_x; i++){
        unsigned int power = (size_x-i-1);
        unsigned int base = pow(number_per_state,power);
        if(N >= base){
            q[power] = N/base;
            N = N - base * q[power];
        }
        else{
            q[power] = 0;
        }
    }
    return q;
}


std::vector<unsigned int> MDP::map_Nsigma_to_sigma(const unsigned int& Nsigma){
    std::vector<unsigned int> sigma(size_u);
    unsigned int N = Nsigma;
    for(unsigned int i=0; i<size_u; i++){
        unsigned int power = (size_u-i-1);
        unsigned int base = pow(number_per_action,power);
        if(N >= base){
            sigma[power] = N/base;
            N = N - base * sigma[power];
        }
        else{
            sigma[power] = 0;
        }
    }
    return sigma;
}


unsigned int MDP::map_q_to_Nq(const std::vector<unsigned int>& q){
    unsigned int Nq = 0;
    for(int i=0; i<size_x; i++){
        Nq = Nq + q[i] * pow(number_per_state, i);
    }
    return Nq;
}


Eigen::VectorXd MDP::map_q_to_x(const std::vector<unsigned int>& q){
    // fail node
    if(q[0] == number_Q){
        Eigen::VectorXd x = Eigen::VectorXd::Zero(size_x);
        for(int i=0; i<size_x; i++){
            x[i] = std::numeric_limits<double>::quiet_NaN();
        }
        return x;
    }

    // normal nodes
    Eigen::VectorXd x = Eigen::VectorXd::Zero(size_x);
    for(int i=0; i<size_x; i++){
        x[i] = planner_pointer->ode_solver_pointer->ode_pointer->x_min[i]
               + 0.5  * dx[i]
               + q[i] * dx[i];
    }
    return x;
}


Eigen::VectorXd MDP::map_sigma_to_u(const std::vector<unsigned int>& sigma){
    Eigen::VectorXd u = Eigen::VectorXd::Zero(size_u);
    for(int i=0; i<size_u; i++){
        u[i] = planner_pointer->ode_solver_pointer->ode_pointer->u_min[i]
               + 0.5  * du[i]
               + sigma[i] * du[i];
    }
    return u;
}


std::vector<unsigned int> MDP::map_x_to_q(const Eigen::VectorXd& x){
    std::vector<unsigned int> q(size_x);
    for(int i=0; i<size_x; i++){
        if(dx[i] == 0){
            q[i] == 0;
        }
        else{
            double q_double = (x[i] - planner_pointer->ode_solver_pointer->ode_pointer->x_min[i])/ dx[i];
            q[i] = static_cast<int>(q_double);
        }
    }
    return q;
}


std::vector<unsigned int> MDP::map_u_to_sigma(const Eigen::VectorXd& u){
    std::vector<unsigned int> sigma(size_u);
    for(int i=0; i<size_u; i++){
        if(du[i] == 0){
            sigma[i] == 0;
        }
        else{
            double sigma_double = (u[i] - planner_pointer->ode_solver_pointer->ode_pointer->u_min[i])/ du[i];
            sigma[i] = static_cast<int>(sigma_double);
        }
    }
    return sigma;
}


unsigned int MDP::map_NqNsigma_to_row(const unsigned int& Nq, const unsigned int& Nsigma){
    unsigned int row;
    row = Nq * number_Sigma + Nsigma;
    return row;
}


std::tuple<unsigned int, unsigned int> MDP::map_row_to_NqNsigma(const unsigned int& row){
    if(row >= number_Sigma){
        unsigned int Nq = row / number_Sigma;
        unsigned int Nsigma = row - Nq * number_Sigma;
        return std::make_tuple(Nq, Nsigma);
    }
    else{
        unsigned int Nq = 0;
        unsigned int Nsigma = row;
        return std::make_tuple(Nq, Nsigma);
    }
}


void MDP::log_vector(const Eigen::VectorXd& v){
	std::cout << "vector: ";
    for(int i = 0; i < v.size(); i ++){
        std::cout << v[i] << " ";
    }
	std::cout << "\n";
}


void MDP::log_vector(const std::vector<unsigned int>& v){
	std::cout << "vector: ";
    for(int i = 0; i < v.size(); i ++){
        std::cout << v[i] << " ";
    }
	std::cout << "\n";
}


void MDP::test_mapping(){
    std::cout << "mapping test\n";

    // for(unsigned int i=0; i<number_Q; i++){
    //     std::vector<unsigned int> q0 = map_Nq_to_q(i);
    //     log_vector(q0);

    //     Eigen::VectorXd x0 = map_q_to_x(q0);
    //     log_vector(x0);

    //     q0 = map_x_to_q(x0);
    //     log_vector(q0);

    //     std::cout << map_q_to_Nq(q0);

    //     std::cout << "\n";
    // }

    std::cout << "\n";
    for(unsigned int i=0; i<number_Sigma; i++){
        std::vector<unsigned int> sigma = map_Nsigma_to_sigma(i);
        log_vector(sigma);

        Eigen::VectorXd u = map_sigma_to_u(sigma);
        log_vector(u);

        sigma = map_u_to_sigma(u);
        log_vector(sigma);

        std::cout << "\n";
    }

    // for(unsigned int Nq=0; Nq<number_Q; Nq++){
    //     for(unsigned int Nsigma=0; Nsigma<number_Sigma; Nsigma++){
    //         unsigned int row = map_NqNsigma_to_row(Nq, Nsigma);
    //         auto [Nqq, Nsigmasigma] = map_row_to_NqNsigma(row);
    //         std::cout << Nq << " " << Nsigma << " " << row << "\t" << Nqq << " " << Nsigmasigma << "\n";
    //     }
    // }
}


std::vector<Eigen::VectorXd> MDP::get_neighbor_points(const Eigen::VectorXd& x){
    std::vector<Eigen::VectorXd> x_s;
    // positive neighbor
    for(int i=0; i<size_x; i++){
        Eigen::VectorXd x_neighbor_i = x;
        x_neighbor_i[i] = x_neighbor_i[i] + dx[i];
        if(!planner_pointer->ode_solver_pointer->ode_pointer->is_out_of_domain(x_neighbor_i)){
            x_s.push_back(x_neighbor_i);
        }
    }
    // negative neighbor
    for(int i=0; i<size_x; i++){
        Eigen::VectorXd x_neighbor_i = x;
        x_neighbor_i[i] = x_neighbor_i[i] - dx[i];
        if(!planner_pointer->ode_solver_pointer->ode_pointer->is_out_of_domain(x_neighbor_i)){
            x_s.push_back(x_neighbor_i);
        }
    }
    // all positive neigbor
    Eigen::VectorXd x_neighbor_i = x + dx;
    if(!planner_pointer->ode_solver_pointer->ode_pointer->is_out_of_domain(x_neighbor_i)){
        x_s.push_back(x_neighbor_i);
    }
    // all negative neighbor
    x_neighbor_i = x - dx;
    if(!planner_pointer->ode_solver_pointer->ode_pointer->is_out_of_domain(x_neighbor_i)){
        x_s.push_back(x_neighbor_i);
    }
    return x_s;
}


std::vector<Eigen::VectorXd> MDP::get_boader_points(const Eigen::VectorXd& x){
    std::vector<Eigen::VectorXd> x_s;
    // positive neighbor
    for(int i=0; i<size_x; i++){
        Eigen::VectorXd x_neighbor_i = x;
        x_neighbor_i[i] = x_neighbor_i[i] + 0.5*dx[i];
        if(!planner_pointer->ode_solver_pointer->ode_pointer->is_out_of_domain(x_neighbor_i)){
            x_s.push_back(x_neighbor_i);
        }
    }
    // negative neighbor
    for(int i=0; i<size_x; i++){
        Eigen::VectorXd x_neighbor_i = x;
        x_neighbor_i[i] = x_neighbor_i[i] - 0.5*dx[i];
        if(!planner_pointer->ode_solver_pointer->ode_pointer->is_out_of_domain(x_neighbor_i)){
            x_s.push_back(x_neighbor_i);
        }
    }
    // all positive neigbor
    Eigen::VectorXd x_neighbor_i = x + 0.5*dx;
    if(!planner_pointer->ode_solver_pointer->ode_pointer->is_out_of_domain(x_neighbor_i)){
        x_s.push_back(x_neighbor_i);
    }
    // all negative neighbor
    x_neighbor_i = x - 0.5*dx;
    if(!planner_pointer->ode_solver_pointer->ode_pointer->is_out_of_domain(x_neighbor_i)){
        x_s.push_back(x_neighbor_i);
    }
    return x_s;
}


std::vector<Eigen::VectorXd> MDP::write_transition(){
    std::vector<Eigen::VectorXd> data;
    for (int i = 0; i < matrix->size(); ++i) {
        for (const auto& entry : (*matrix)[i]) {
            // std::cout << "(" << i << ", " << entry.first << "): " << entry.second << "\t";
            Eigen::VectorXd row(3);
            row << i, entry.first, entry.second;
            data.push_back(row);
        }
    }
    // std::cout << "\n";
    return data;
}


double MDP::getRandomDouble(double x_min, double x_max) {
    // Seed the random number generator with a random device
    std::random_device rd;
    std::mt19937 gen(rd());

    // Define the distribution for double values in the range [x_min, x_max]
    std::uniform_real_distribution<double> distribution(x_min, x_max);

    double value = distribution(gen);

    return value;
    // double multiplier = std::pow(10.0, 3);
    // return std::round(value * multiplier) / multiplier;
}


double MDP::reward(const unsigned int& i_Nq, const unsigned int& i_Nsigma){
    if(map_to_is_goal_node[i_Nq]){
        return 1.0;
    }
    if(i_Nq == nodes_Q.size()-1){
        return -1.0;
    }
    return 0.0;
}

void MDP::print_J(const std::vector<double>& J){
    for(const auto& jj : J){
        std::cout << std::fixed << std::setprecision(2) << jj << ", ";
    }
    std::cout << "\n";
}

void MDP::add_Q_neighbors(const std::vector<unsigned int> nodes_Q_pivot){
    // add 1-neigborhood nodes
    for(const auto& Nq : nodes_Q_pivot){
        if(Nq != number_Q){
            Eigen::VectorXd x = map_q_to_x(map_Nq_to_q(Nq));
            std::vector<Eigen::VectorXd> x_s = get_neighbor_points(x);
            for(const auto& x_neighbor : x_s){
                // [NOTE: more Rigorous checkind is required]
                if(!planner_pointer->ode_solver_pointer->ode_pointer->is_out_of_domain(x_neighbor)){
                    unsigned int Nq_neighbor = map_q_to_Nq(map_x_to_q(x_neighbor));
                    auto it = std::find(nodes_Q.begin(), nodes_Q.end(), Nq_neighbor);
                    if(it == nodes_Q.end()){
                        nodes_Q.push_back(Nq_neighbor);
                    }
                }
            }
        }
    }
}


// std::vector<Eigen::VectorXd> MDP::write_sparse_matrix(){
//     // std::vector<Eigen::VectorXd> sparse_matrix_data;
//     // for (int i = 0; i < sparse_matrix_pointer->rows; ++i) {
//     //     for (const auto& entry : sparse_matrix_pointer->matrix[i]) {
//     //         unsigned int row = i;
//     //         auto [Nq, Nsigma] = map_row_to_NqNsigma(row);
//     //         if(Nq != entry.first){
//     //             std::cout << "(" << Nq << ", " << Nsigma << ", " << entry.first << "): " << entry.second << std::endl;
//     //         }

//     //         Eigen::VectorXd data(4);
//     //         data << Nq, Nsigma, entry.first, entry.second;
//     //         sparse_matrix_data.push_back(data);
//     //     }
//     // }
//     // return sparse_matrix_data;
// }