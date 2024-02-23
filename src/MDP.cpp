#include "MDP.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>


MDP::MDP(PlannerVirtual* pointer, const int n_per_state, const int n_per_action){
    planner_pointer = pointer;
    size_x = planner_pointer->size_x;
    size_u = planner_pointer->size_u - 1; // to remove the time duraiton
    number_per_state = n_per_state;
    number_per_action = n_per_action;
    size_total_discrete_state = pow(number_per_state, size_x);
    size_total_discrete_action = pow(number_per_action, size_u);
    dx = Eigen::VectorXd::Zero(size_x);
    for(int i=0; i<size_x; i++){
        dx[i] = (planner_pointer->ode_solver_pointer->ode_pointer->x_max[i]
                - planner_pointer->ode_solver_pointer->ode_pointer->x_min[i])/ number_per_state;
        // std::cout << "[DEBUG] dx " << dx[i] << "\n";
    }
    du = Eigen::VectorXd::Zero(size_u);
    for(int i=0; i<size_u; i++){
        du[i] = (planner_pointer->ode_solver_pointer->ode_pointer->u_max[i]
                - planner_pointer->ode_solver_pointer->ode_pointer->u_min[i])/ number_per_action;
        // std::cout << "[DEBUG] du " << du[i] << "\n";
    }
}


void MDP::construct_discrete_state(const std::vector<Eigen::VectorXd>& traj){
    // intersection with trajectory
    included_discrete_states.clear();
    for(const auto& s : traj){
        Eigen::VectorXd x = s.head(size_x); //log_vector(x);
        std::vector<unsigned int> q = map_x_to_q(x); //log_vector(q);
        unsigned int labelq = map_q_to_labelq(q); //std::cout << labelq << "\t" << size_total_discrete_state << "\n";
        add_labelq(labelq);
    }

    // intersection with x goals (randomly sample the goal region)
    for(int i=0; i<50; i++){
        Eigen::VectorXd x(size_x);
        for(int j=0; j<size_x; j++){
            x[j] = getRandomDouble(x_goals[j][0], x_goals[j][1]);
        }
        std::vector<unsigned int> q = map_x_to_q(x); //log_vector(q);
        unsigned int labelq = map_q_to_labelq(q); //std::cout << labelq << "\t" << size_total_discrete_state << "\n";
        add_labelq(labelq);
    }

    // K-layer neighbor nodes
    std::vector<unsigned int> included_discrete_states_pivot;
    for(int i=0; i<k_layer; i++){
        included_discrete_states_pivot = included_discrete_states;
        add_neighbor_discrete_states(included_discrete_states_pivot);
    }

    // Construct the goal node mapping: Index_Q --> bool
    construct_index_discrete_state_to_bool_isgoal();

    // add a fail node
    included_discrete_states.push_back(size_total_discrete_state);
    // print out
    std::cout << "[DEBUG] size of nodes: " << included_discrete_states.size() << "\n";
    
    // Init J
    init_J();
}


void MDP::read_discrete_state(std::string filename){
    included_discrete_states.clear();
    std::ifstream file(filename);
    // Check if the file is open
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
    }
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            // Convert string to unsigned int and add to the vector
            included_discrete_states.push_back(std::stoul(cell));
        }
    }
    // Close the file
    file.close();
    // Construct the goal node mapping: Index_Q --> bool
    construct_index_discrete_state_to_bool_isgoal();
    // print out
    std::cout << "[DEBUG] size of nodes: " << included_discrete_states.size() << "\n";
    // Init J
    init_J();
}


std::vector<Eigen::VectorXd> MDP::write_discrete_state(){
    std::vector<Eigen::VectorXd> data;
    for(int i=0; i<included_discrete_states.size(); i++){
        Eigen::VectorXd row(size_x+1); // row (x, J)
        Eigen::VectorXd x = map_q_to_x(map_labelq_to_q(included_discrete_states[i]));
        double value = (*J)[i];
        row << x, value;
        data.push_back(row);
    }
    return data;
}


std::vector<Eigen::VectorXd> MDP::write_discrete_state_info(){
    std::vector<Eigen::VectorXd> data; // first row (dx, du). second row (index goal nodes)

    // get state partition information (dx, du)
    Eigen::VectorXd row1(size_x);
    for(int i=0; i<size_x; i++){
        row1[i] = dx[i];
    }
    data.push_back(row1);

    // get goal nodes index
    std::cout << "[DEBUG] goal nodes: ";
    std::vector<int> row2_vector;
    for(const auto& key : set_index_goal_q){
        std::cout << key << ",";
        row2_vector.push_back(key);
    } std::cout << "\n";
    // for(int i=0; i<index_discrete_state_to_bool_isgoal.size(); i++){
    //     if(index_discrete_state_to_bool_isgoal[i]){
    //         std::cout << i << ",";
    //         row2_vector.push_back(i);
    //         // Eigen::VectorXd x = map_q_to_x(map_labelq_to_q(included_discrete_states[i]));
    //         // log_vector(x);
    //     }
    // } std::cout << "\n";

    // cast std::vector to Eigen::VectorXd
    Eigen::VectorXd row2(row2_vector.size());
    for(int i=0; i<row2_vector.size(); i++){
        row2[i] = row2_vector[i];
    }
    data.push_back(row2);
    return data;
}


std::tuple<bool, Eigen::VectorXd> MDP::get_feedback_control(const Eigen::VectorXd& s){
    Eigen::VectorXd u = Eigen::VectorXd::Zero(size_u);
    unsigned int labelq = map_q_to_labelq(map_x_to_q(s));
    // std::cout << "[DEBUG] discrete state: " << labelq << "\n";
    for(int i=0; i<included_discrete_states.size()-1; i++){
        if(labelq == included_discrete_states[i]){
            u = discrete_state_to_control[i];
            // std::cout << "[DEBUG] u(MDP): "; log_vector(u);
            return std::make_tuple(true, u);
        }
    }
    if(labelq == size_total_discrete_state){
        std::cout << "[ERROR] Node label: " << labelq << " reach fail node\n";
    }
    else{
        std::cout << "[ERROR] No defined feedback control\n";
    }
    return std::make_tuple(false, u);;
}


void MDP::construct_transition(){
    std::cout << "[DEBUG] Constructing transition ...\n";
    std::atomic<unsigned int> progress(0);

    // Init transition
    const unsigned int number_finite_state = included_discrete_states.size();
    is_set_transitions_success = false;
    transition_matrix_pointer = new std::vector<std::map<unsigned int, double>>(number_finite_state*size_total_discrete_action);
    double time_duration = 1.0;
    bool is_process_noise = false;
    bool is_check_unsafe = true;

    // [Multi-threads]
    unsigned int num_threads = std::thread::hardware_concurrency();
    unsigned int total_iterations = number_finite_state * size_total_discrete_action;
    unsigned int chunk_size = total_iterations / num_threads;
    std::vector<std::thread> threads;
    for (unsigned int i = 0; i < num_threads; ++i) {
        unsigned int start = i * chunk_size;
        unsigned int end = (i == num_threads - 1) ? total_iterations : (i + 1) * chunk_size;
        threads.emplace_back([=, &progress]() {
            process_range(start, end, size_total_discrete_action, number_samples, time_duration, is_process_noise, is_check_unsafe, std::ref(progress));
        });
    }
    for (auto& thread : threads) {
        thread.join();
    }
    std::cout << "[DEBUG] Complete constructing transition.csv\n";
}
    // [Single thread]
    // progress print-out
    // unsigned int total_iterations = number_finite_state * size_total_discrete_action;
    // unsigned int dN;
    // if(total_iterations >= 100){
    //     dN = total_iterations/100;
    // } else{ dN = 1; }
    // // loop over inclueded finite states
    // for(unsigned int i=0; i<total_iterations; i++){
    //     unsigned int index_q = i / size_total_discrete_action;
    //     unsigned int index_sigma = i % size_total_discrete_action;
    //     set_transitions(index_q, index_sigma, number_samples, time_duration, is_process_noise, is_check_unsafe);
    //     if(i%dN == 0){
    //         std::cout << round( 100.0 * static_cast<double>(index_q) / static_cast<double>(included_discrete_states.size()) ) << " %\n";
    //     }
    // }


void MDP::read_transition(std::string filename){
    // Open the CSV file
    std::ifstream inputFile(filename);

    // Check if the file is open
    if (!inputFile.is_open()) {
        std::cerr << "Error opening the file." << std::endl;
    }

    // Init transition_matrix_pointer
    transition_matrix_pointer = new std::vector<std::map<unsigned int, double>>(included_discrete_states.size() * size_total_discrete_action);

    // Read the file line by line
    std::string line;

    while (std::getline(inputFile, line)) {
        // Create a stringstream from the line
        std::istringstream iss(line);

        // Parse the CSV values using a loop
        std::vector<double> values;
        std::string value;

        while (std::getline(iss, value, ',')) {
            values.push_back(std::stod(value));
        }
        unsigned int row = values[0];
        unsigned int col = values[1];
        (*transition_matrix_pointer)[row][col] = values[2];
    }

    // Close the file
    inputFile.close();
    std::cout << "[DEBUG] Complete reading transition.csv\n";
}


// multi-threads wrapper function
void MDP::process_range(
                unsigned int start, 
                unsigned int end, 
                unsigned int size_total_discrete_action, 
                unsigned int number_samples, 
                unsigned int time_duration, 
                bool is_process_noise, 
                bool is_check_unsafe,
                std::atomic<unsigned int>& progress){
    unsigned int total_iterations = end - start;
    for (unsigned int i = start; i < end; ++i) {
        unsigned int index_q = i / size_total_discrete_action;
        unsigned int index_sigma = i % size_total_discrete_action;
        set_transitions(index_q, index_sigma, number_samples, time_duration, is_process_noise, is_check_unsafe);
        // progress print out
        unsigned int current_progress = static_cast<unsigned int>(100.0 * static_cast<double>(i - start) / static_cast<double>(total_iterations));
        unsigned int prev_progress = progress.load();
        if (current_progress > prev_progress && current_progress % 1 == 0) {
            progress.store(current_progress);
            std::cout << "Progress: " << current_progress << " %\n";
        }
    }
}


void MDP::set_transitions(unsigned int index_q, unsigned int index_sigma, int number_samples, double time_duration,
                          bool is_process_noise, bool is_check_unsafe){
    // get labelq
    unsigned int labelq = included_discrete_states[index_q];
    // no transition for fail node
    if(labelq == size_total_discrete_state){
        return;
    }

    // get x
    Eigen::VectorXd x = map_q_to_x(map_labelq_to_q(labelq)); //std::cout << "x: "; log_vector(x);

    // get x_s
    std::vector<Eigen::VectorXd> x_s;
    x_s = get_boader_points(x);
    x_s.push_back(x);

    // get labelsigma
    unsigned int labelsigma = index_sigma;
    // get u
    Eigen::VectorXd u = map_sigma_to_u(map_labelsigma_to_sigma(labelsigma));
    //std::cout << index_sigma << " -th u is: "; log_vector(u);

    // get index row
    unsigned int index_row = size_total_discrete_action * index_q + index_sigma;
    //std::cout << "i(row): " << index_row << "\n";

    // outerloop: noise sample, inner loop: state sample
    const int number_boader_points = x_s.size();
    const int number_total_samples = number_samples * number_boader_points;

    // combine outer and inner for loops to one for loop
    for(int k=0; k<number_total_samples; k++){
        int i = k / number_boader_points;
        int j = k % number_boader_points;
    // for(int i=0; i<number_samples; i++){
    //     for(int j=0; j<number_boader_points; j++){
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
            // map to labelq_new
            std::vector<unsigned int> q_new = map_x_to_q(x_new);
            unsigned int labelq_new = map_q_to_labelq(q_new);
            // std::cout << "labelq new: " << labelq_new << "\t";
            // check if labelq new is in the finite state and get the corresponding index.
            unsigned int index_labelq_new;
            auto it = find(included_discrete_states.begin(), included_discrete_states.end(), labelq_new);
            if(it != included_discrete_states.end()){
                // std::cout << j << "-set transition: " << labelq << " " << labelsigma << " " << labelq_new << "\n";
                index_labelq_new = it - included_discrete_states.begin();
            }
            else{
                // std::cout << j << "-set transition (fail node): " << labelq << " " << labelsigma << " " << size_total_discrete_state << "\n";
                index_labelq_new = included_discrete_states.size()-1;
            }
            // std::cout << "i(labelq new): " << index_labelq_new;
            if(!is_set_transitions_success && is_indexq_goal(index_labelq_new) && index_q != index_labelq_new){
                is_set_transitions_success = true;
            }
            // get the value
            double value = (*transition_matrix_pointer)[index_row][index_labelq_new];
            // std::cout << "\texisting value: " << value;
            value = value + 1.0/static_cast<double>(number_total_samples);
            // update the value
            (*transition_matrix_pointer)[index_row][index_labelq_new] = value;
                // std::cout << "\tafter update: " << (*transition_matrix_pointer)[index_row][index_labelq_new] << "\n";
            //}
        }
    }
    // std::cout << "\n";
}


void MDP::value_iteration(){
    std::cout << "[DEBUG] value iteration ...\t";
    // [TEMP] MDP for testing
    // included_discrete_states.clear();
    // size_total_discrete_action = 2;
    // transition_matrix_pointer->clear();
    // included_discrete_states = {0, 1, 2, 3, 4};
    // const unsigned int number_finite_state = included_discrete_states.size();
    // transition_matrix_pointer = new std::vector<std::map<unsigned int, double>>(number_finite_state*size_total_discrete_action);
    // (*transition_matrix_pointer)[0][1] = 1.0; (*transition_matrix_pointer)[1][2] = 1.0; (*transition_matrix_pointer)[2][2] = 1.0; (*transition_matrix_pointer)[3][3] = 1.0;
    // (*transition_matrix_pointer)[4][3] = 1.0; (*transition_matrix_pointer)[5][4] = 1.0; (*transition_matrix_pointer)[6][4] = 1.0; (*transition_matrix_pointer)[7][3] = 1.0;
    // print_transition();
    // index_discrete_state_to_bool_isgoal.clear();
    // for(int i=0; i<included_discrete_states.size(); i++){
    //     index_discrete_state_to_bool_isgoal[i] = false;
    // }
    // index_discrete_state_to_bool_isgoal[3] = true;

    const double converge_epsilon = 0.01;

    // iteration k
    for(int k=0; k<200; k++){
        double epsilon = -std::numeric_limits<double>::infinity();
        for(unsigned int i=0; i<included_discrete_states.size()-1; i++){
            if(!is_indexq_goal(i)){
                // Init
                double optimal_value_J_q_sigma = -std::numeric_limits<double>::infinity();
                for(int j=0; j<size_total_discrete_action; j++){
                    unsigned int labelsigma = j;
                    unsigned int index_row = size_total_discrete_action * i + j;
                    double value_J_q_sigma = reward(i, 0);
                    for (const auto& entry : (*transition_matrix_pointer)[index_row]) {
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
        // std::cout << "J(" << k+1 << "): "; print_J(*J); // std::cout << "\n";
        if(epsilon < converge_epsilon){
            std::cout << "value iteration converage\n";
            break;
        }
    }
    // print_J((*J)); 
    std::cout << "J(0): " << (*J)[0] << "\n";
}


void MDP::synthesize_control(){
    std::cout << "[DEBUG] synthesize control ...\t";
    for(unsigned int i=0; i<included_discrete_states.size()-1; i++){
        if(!is_indexq_goal(i)){
            unsigned int optimal_labelsigma = 0;
            double optimal_value_J_q_sigma = -std::numeric_limits<double>::infinity();
            for(int j=0; j<size_total_discrete_action; j++){
                unsigned int index_row = size_total_discrete_action * i + j;
                double value_J_q_sigma = reward(i, 0);
                for (const auto& entry : (*transition_matrix_pointer)[index_row]) {
                    unsigned int index_col = entry.first;
                    double probability = entry.second;
                    value_J_q_sigma = value_J_q_sigma + gamma * probability * (*J)[index_col];
                }
                if(value_J_q_sigma > optimal_value_J_q_sigma){
                    optimal_value_J_q_sigma = value_J_q_sigma;
                    optimal_labelsigma = j;
                }
            }
            Eigen::VectorXd u = map_sigma_to_u(map_labelsigma_to_sigma(optimal_labelsigma));
            discrete_state_to_control[i] = u;
        }
        else{
            Eigen::VectorXd u = Eigen::VectorXd::Zero(size_u);
            discrete_state_to_control[i] = u;
        }
    }
    std::cout << "complete\n";
}


void MDP::test_synthesis_control(const Eigen::VectorXd &s){
    //std::cout << "[DEBUG] Test synthesis control\n";
    bool is_labelq_in_MDP = false;
    Eigen::VectorXd u = Eigen::VectorXd::Zero(size_u);
    unsigned int labelq = map_q_to_labelq(map_x_to_q(s));
    for(int i=0; i<included_discrete_states.size()-1; i++){
        if(labelq == included_discrete_states[i]){
            //std::cout << "Index_Q: " << i << ", Fail Node Index: " << included_discrete_states.size()-1 << ", Fail Node Label: " <<  size_total_discrete_state << " \n";
            u = discrete_state_to_control[i];
            std::vector<unsigned int> sigma = map_u_to_sigma(u);
            unsigned int labelsigma = map_sigam_to_labelsigma(sigma); 
            unsigned int row = map_indexq_and_indexsigma_to_row(i, labelsigma);
            // for (const auto& entry : (*transition_matrix_pointer)[row]){
            //     std::cout << "(" << i << ", " 
            //               << labelsigma << ", " 
            //               << entry.first << ") -> " 
            //               << entry.second << "\n";
            // }
            is_labelq_in_MDP = true;
            break;
        }
    }
    if(!is_labelq_in_MDP){
        if(labelq == size_total_discrete_state){
            std::cout << "[ERROR] labelq: " << labelq << " reaches fail node: " << size_total_discrete_state << "\n";
        }
        else{
            std::cout << "[ERROR] labelq: " << labelq << " is not in MDP\n";
        }
    }
}

// This function might imply that the goal nodes are not well defined.
// [TEMP] narrow the goal region
void MDP::print_goal_node_index(const Eigen::VectorXd& s){
    unsigned int labelq = map_q_to_labelq(map_x_to_q(s));
    for(unsigned int i=0; i<included_discrete_states.size(); i++){
        if(labelq == included_discrete_states[i]){
            if(is_indexq_goal(i)){
                std::cout << "[DEBUG] finite goal state (index): " << i << "\n";
                break;
            }
            // unsigned int index_Q = i;
            // // std::cout << "goal node (index): ";
            // for(int j=0; j<index_discrete_state_to_bool_isgoal.size(); j++){
            //     // std::cout << j << " ";
            //     if(index_Q == j && index_discrete_state_to_bool_isgoal[j]){
            //         std::cout << "[DEBUG] finite goal state (index): " << index_Q << "\n";
            //     }
            // }
        }
    }
}


std::vector<unsigned int> MDP::map_labelq_to_q(const unsigned int& labelq){
    // fail node
    if(labelq == size_total_discrete_state){
        std::vector<unsigned int> q(size_x);
        for(int i=0; i<size_x; i++){
            q[i] = size_total_discrete_state;
        }
        return q;
    }
    // normal nodes
    std::vector<unsigned int> q(size_x);
    unsigned int N = labelq;
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


std::vector<unsigned int> MDP::map_labelsigma_to_sigma(const unsigned int& labelsigma){
    std::vector<unsigned int> sigma(size_u);
    unsigned int N = labelsigma;
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


unsigned int MDP::map_q_to_labelq(const std::vector<unsigned int>& q){
    unsigned int labelq = 0;
    for(int i=0; i<size_x; i++){
        labelq = labelq + q[i] * pow(number_per_state, i);
    }
    return labelq;
}


unsigned int MDP::map_sigam_to_labelsigma(const std::vector<unsigned int>& sigma){
    unsigned int labelsigma = 0;
    for(int i=0; i<size_u; i++){
        labelsigma = labelsigma + sigma[i] * pow(number_per_action, i);
    }
    return labelsigma;
}


Eigen::VectorXd MDP::map_q_to_x(const std::vector<unsigned int>& q){
    // fail node
    if(q[0] == size_total_discrete_state){
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


unsigned int MDP::map_indexq_and_indexsigma_to_row(const unsigned int& labelq, const unsigned int& labelsigma){
    unsigned int row;
    row = labelq * size_total_discrete_action + labelsigma;
    return row;
}


std::tuple<unsigned int, unsigned int> MDP::map_row_to_indexq_and_indexsigma(const unsigned int& row){
    if(row >= size_total_discrete_action){
        unsigned int labelq = row / size_total_discrete_action;
        unsigned int labelsigma = row - labelq * size_total_discrete_action;
        return std::make_tuple(labelq, labelsigma);
    }
    else{
        unsigned int labelq = 0;
        unsigned int labelsigma = row;
        return std::make_tuple(labelq, labelsigma);
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

    // for(unsigned int i=0; i<size_total_discrete_state; i++){
    //     std::vector<unsigned int> q0 = map_labelq_to_q(i);
    //     log_vector(q0);

    //     Eigen::VectorXd x0 = map_q_to_x(q0);
    //     log_vector(x0);

    //     q0 = map_x_to_q(x0);
    //     log_vector(q0);

    //     std::cout << map_q_to_labelq(q0);

    //     std::cout << "\n";
    // }

    std::cout << "\n";
    for(unsigned int i=0; i<size_total_discrete_action; i++){
        std::vector<unsigned int> sigma = map_labelsigma_to_sigma(i);
        log_vector(sigma);

        Eigen::VectorXd u = map_sigma_to_u(sigma);
        log_vector(u);

        sigma = map_u_to_sigma(u);
        log_vector(sigma);

        std::cout << "\n";
    }

    // for(unsigned int labelq=0; labelq<size_total_discrete_state; labelq++){
    //     for(unsigned int labelsigma=0; labelsigma<size_total_discrete_action; labelsigma++){
    //         unsigned int row = map_labelqlabelsigma_to_row(labelq, labelsigma);
    //         auto [labelqq, labelsigmasigma] = map_row_to_labelqlabelsigma(row);
    //         std::cout << labelq << " " << labelsigma << " " << row << "\t" << labelqq << " " << labelsigmasigma << "\n";
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


void MDP::write_transition(std::string filename){
    std::ofstream outputFile(filename);
    if (!outputFile.is_open()) {
        std::cerr << "Error opening the file for writing." << std::endl;
    }
    //// [Single thread]
    // for (int i = 0; i < transition_matrix_pointer->size(); ++i) {
    //     for (const auto& entry : (*transition_matrix_pointer)[i]) {
    //         // std::cout << "(" << i << ", " << entry.first << "): " << entry.second << "\t";
    //         outputFile << i << "," << entry.first << ',' << entry.second << std::endl;
    //     }
    // }
    // [Multi threads]
    std::vector<std::thread> threads;
    const int chunk_size = transition_matrix_pointer->size() / std::thread::hardware_concurrency();
    std::mutex mtx; // Mutex for thread-safe file writing
    for (int i = 0; i < std::thread::hardware_concurrency(); ++i) {
        int start = i * chunk_size;
        int end = (i == std::thread::hardware_concurrency() - 1) ? transition_matrix_pointer->size() : (i + 1) * chunk_size;
        threads.emplace_back([&, start, end]() {
            std::stringstream ss;
            for (int j = start; j < end; ++j) {
                for (const auto& entry : (*transition_matrix_pointer)[j]) {
                    ss << j << "," << entry.first << "," << entry.second << "\n";
                }
            }
            std::lock_guard<std::mutex> lock(mtx); // Lock the mutex before writing to file
            outputFile << ss.str();
        });
    }
    for (auto& thread : threads) {
        thread.join();
    }
    outputFile.close();
    std::cout << "Save file to: " << filename << "\n";
}


std::vector<Eigen::VectorXd> MDP::write_synthesis_control(){
    std::vector<Eigen::VectorXd> data;
    for(int i=0; i<included_discrete_states.size()-1; i++){
        Eigen::VectorXd u = discrete_state_to_control[i];
        Eigen::VectorXd x = map_q_to_x(map_labelq_to_q(included_discrete_states[i]));
        Eigen::VectorXd row(size_x+size_u);
        row << x, u;
        data.push_back(row);
    }
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


double MDP::reward(const unsigned int& i_labelq, const unsigned int& i_labelsigma){
    if(is_indexq_goal(i_labelq)){
        // goal partitions
        return 1.0;
    }
    if(i_labelq == included_discrete_states.size()-1){
        // fail partition
        return 0.0;
    }
    return 0.0;
}


void MDP::print_J(const std::vector<double>& J){
    for(const auto& jj : J){
        std::cout << std::fixed << std::setprecision(2) << jj << ", ";
    }
    std::cout << "\n";
}


void MDP::add_neighbor_discrete_states(const std::vector<unsigned int> included_discrete_states_pivot){
    for(const auto& labelq : included_discrete_states_pivot){
        if(labelq != size_total_discrete_state){
            Eigen::VectorXd x = map_q_to_x(map_labelq_to_q(labelq));
            std::vector<Eigen::VectorXd> x_s = get_neighbor_points(x);
            for(const auto& x_neighbor : x_s){
                if(!planner_pointer->ode_solver_pointer->ode_pointer->is_out_of_domain(x_neighbor)
                && !planner_pointer->ode_solver_pointer->ode_pointer->is_in_unsafe(x_neighbor)){
                    unsigned int labelq_neighbor = map_q_to_labelq(map_x_to_q(x_neighbor));
                    add_labelq(labelq_neighbor);
                }
            }
        }
    }
}


void MDP::construct_index_discrete_state_to_bool_isgoal(){
    for(int i=0; i<included_discrete_states.size()-1; i++){
        Eigen::VectorXd x = map_q_to_x(map_labelq_to_q(included_discrete_states[i]));
        std::vector<Eigen::VectorXd> x_s = get_boader_points(x);
        bool is_i_goal = true;
        for(const auto& x_ss : x_s){
            if(!planner_pointer->ode_solver_pointer->ode_pointer->is_goals(x_ss, x_goals)){
                is_i_goal = false;
                break;
            }
        }
        if(is_i_goal){
            set_index_goal_q.insert(i);
        }
    }
    // // Construct the goal node mapping: Index_Q --> bool
    // // init to true
    // for(int i=0; i<included_discrete_states.size()-1; i++){
    //     index_discrete_state_to_bool_isgoal[i] = true;
    // }
    // // set to false
    // for(int i=0; i<included_discrete_states.size()-1; i++){
    //     Eigen::VectorXd x = map_q_to_x(map_labelq_to_q(included_discrete_states[i]));
    //     std::vector<Eigen::VectorXd> x_s = get_boader_points(x);
    //     for(const auto& x_ss : x_s){
    //         if(!planner_pointer->ode_solver_pointer->ode_pointer->is_goals(x_ss, x_goals)){
    //             index_discrete_state_to_bool_isgoal[i] = false;
    //             break;
    //         }
    //     }
    // }
    // index_discrete_state_to_bool_isgoal[included_discrete_states.size()-1] = false;
}


bool MDP::is_indexq_goal(const unsigned int& index_q){
    if (set_index_goal_q.find(index_q) != set_index_goal_q.end()) {
        return true;
    }
    return false;
}


void MDP::init_J(){
    std::cout << "[DEBUG] Init J\n";
    J = new std::vector<double>(included_discrete_states.size(), 0.0);
    // init reward of goal partitions
    for(const auto& key : set_index_goal_q){
        (*J)[key] = reward(key, 0);
    }
    // for(int i=0; i<included_discrete_states.size(); i++){
    //     if(index_discrete_state_to_bool_isgoal[i]){
    //         (*J)[i] = reward(i, 0);
    //     }
    // }

    // init reward of fail partition
    J->back() = reward(included_discrete_states.size()-1, 0);
    // std::cout << "J(0): "; print_J((*J));
}


void MDP::add_labelq(const unsigned int& labelq){
    auto it = std::find(included_discrete_states.begin(), included_discrete_states.end(), labelq);
    if(it == included_discrete_states.end()){
        // std::cout << "Element " << labelq << " not found in the vector." << std::endl;
        included_discrete_states.push_back(labelq);
    }
}