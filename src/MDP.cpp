#include "MDP.h"
#include <cmath>
#include <limits>

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

            // [TEMP] will be replace after control synthesizing
            Eigen::VectorXd u = s.segment(size_x, size_u); // log_vector(u);
            discrete_state_to_control[Nq] = u;
            // std::cout << "discrete state to control map(" << Nq << ") "; log_vector(discrete_state_to_control[Nq]);
        }
    }
    // store these pivot nodes
    std::vector<unsigned int> nodes_Q_pivot = nodes_Q;
    // add 1-neigborhood nodes
    for(const auto& Nq : nodes_Q_pivot){
        Eigen::VectorXd x = map_q_to_x(map_Nq_to_q(Nq));
        // positive neighbor
        for(int i=0; i<size_x; i++){
            Eigen::VectorXd x_neighbor_i = x;
            x_neighbor_i[i] = x_neighbor_i[i] + dx[i];
            if(!planner_pointer->ode_solver_pointer->ode_pointer->is_out_of_domain(x_neighbor_i)){
                unsigned int Nq_neighbor_i = map_q_to_Nq(map_x_to_q(x_neighbor_i));
                auto it = std::find(nodes_Q.begin(), nodes_Q.end(), Nq_neighbor_i);
                if(it == nodes_Q.end()){
                    // std::cout << "Element " << Nq << " not found in the vector." << std::endl;
                    nodes_Q.push_back(Nq_neighbor_i);

                    // [TEMP] will be replace after control synthesizing
                    Eigen::VectorXd u = discrete_state_to_control[Nq];
                    discrete_state_to_control[Nq_neighbor_i] = u;
                }
            }
        }
        // negative neighbor
        for(int i=0; i<size_x; i++){
            Eigen::VectorXd x_neighbor_i = x;
            x_neighbor_i[i] = x_neighbor_i[i] - dx[i];
            if(!planner_pointer->ode_solver_pointer->ode_pointer->is_out_of_domain(x_neighbor_i)){
                unsigned int Nq_neighbor_i = map_q_to_Nq(map_x_to_q(x_neighbor_i));
                auto it = std::find(nodes_Q.begin(), nodes_Q.end(), Nq_neighbor_i);
                if(it == nodes_Q.end()){
                    // std::cout << "Element " << Nq << " not found in the vector." << std::endl;
                    nodes_Q.push_back(Nq_neighbor_i);

                    // [TEMP] will be replace after control synthesizing
                    Eigen::VectorXd u = discrete_state_to_control[Nq];
                    discrete_state_to_control[Nq_neighbor_i] = u;
                }
            }
        }
    }

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

Eigen::VectorXd MDP::get_feedback_control(const Eigen::VectorXd& s){
    Eigen::VectorXd u = Eigen::VectorXd::Zero(size_u);
    unsigned int Nq = map_q_to_Nq(map_x_to_q(s));
    // std::cout << "[DEBUG] discrete state: " << Nq << "\n";
    for(const auto& Q : nodes_Q){
        if(Nq == Q){
            u = discrete_state_to_control[Nq];
            // std::cout << "[DEBUG] u(MDP): "; log_vector(u);
            return u;
        }
    }
    std::cout << "[ERROR] no defined feedback control\n";
    return u;
}


void MDP::construct_mdp(){
    // test_mapping();
    // //trans_prob.clear();
    // int number_samples = 1;
    // double time_duration = 1.0;
    // bool is_process_noise = false;
    // bool is_check_unsafe = true;
    // unsigned int dN = number_Q/100;
    // for(unsigned int Nq=0; Nq<number_Q; Nq++){
    //     for(unsigned int Nsigma=0; Nsigma<number_Q; Nsigma++){
    //         construct_trans_prob(Nq, Nsigma, number_samples, time_duration, is_process_noise, is_check_unsafe);
    //     }
    //     if(Nq%dN == 0){
    //         std::cout << round( 100.0 * static_cast<double>(Nq) / static_cast<double>(number_Q) ) << " %\n";
    //     }
    // }
}


void MDP::construct_trans_prob(unsigned int Nq, unsigned int Nsigma, int number_samples, double time_duration,
                               bool is_process_noise, bool is_check_unsafe){
    // std::vector<unsigned int> q = map_Nq_to_q(Nq);
    // std::vector<unsigned int> sigma = map_Nsigma_to_sigma(Nsigma);
    // Eigen::VectorXd x = map_q_to_x(q);
    // Eigen::VectorXd u = map_sigma_to_u(sigma);
    // for(int i=0; i<number_samples; i++){
    //     std::vector<Eigen::VectorXd> traj_segment;
    //     traj_segment = planner_pointer->ode_solver_pointer->solver_runge_kutta(
    //                                     x, 
    //                                     u, 
    //                                     planner_pointer->ode_solver_pointer->time_integration, 
    //                                     time_duration, 
    //                                     x_goals, 
    //                                     is_process_noise,
    //                                     is_check_unsafe);
    //     if(!traj_segment.empty()){
    //         Eigen::VectorXd x_new = traj_segment.back();
    //         std::vector<unsigned int> q_new = map_x_to_q(x_new);
    //         unsigned int Nq_new = map_q_to_Nq(q_new);
    //         unsigned int row = map_NqNsigma_to_row(Nq, Nsigma);
    //         // get the value
    //         unsigned int value = sparse_matrix_pointer->getElement(row, Nq_new);
    //         // update the vaue
    //         value = value + 1.0/(static_cast<double>(number_samples));
    //         sparse_matrix_pointer->setElement(row, Nq_new, value);           
    //         //std::tuple< unsigned int, unsigned int, unsigned int > state_action_state(Nq, Nsigma, Nq_new);
    //         //trans_prob[state_action_state] = trans_prob[state_action_state] + 1.0/(static_cast<double>(number_samples));
    //         // if(Nq != Nq_new){
    //         //     log_vector(q);
    //         //     std::cout << " --> ";
    //         //     log_vector(q_new);
    //         //     std::cout << "\n";
    //         // }
    //     }
    // }
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

    for(unsigned int i=0; i<number_Q; i++){
        std::vector<unsigned int> q0 = map_Nq_to_q(i);
        log_vector(q0);

        Eigen::VectorXd x0 = map_q_to_x(q0);
        log_vector(x0);

        q0 = map_x_to_q(x0);
        log_vector(q0);

        std::cout << map_q_to_Nq(q0);

        std::cout << "\n";
    }

    // std::cout << "\n";
    // for(unsigned int i=0; i<number_Sigma; i++){
    //     std::vector<unsigned int> sigma = map_Nsigma_to_sigma(i);
    //     log_vector(sigma);

    //     Eigen::VectorXd u = map_sigma_to_u(sigma);
    //     log_vector(u);

    //     sigma = map_u_to_sigma(u);
    //     log_vector(sigma);

    //     std::cout << "\n";
    // }

    // for(unsigned int Nq=0; Nq<number_Q; Nq++){
    //     for(unsigned int Nsigma=0; Nsigma<number_Sigma; Nsigma++){
    //         unsigned int row = map_NqNsigma_to_row(Nq, Nsigma);
    //         auto [Nqq, Nsigmasigma] = map_row_to_NqNsigma(row);
    //         std::cout << Nq << " " << Nsigma << " " << row << "\t" << Nqq << " " << Nsigmasigma << "\n";
    //     }
    // }
}


 bool MDP::is_same_q(const std::vector<unsigned int>& q1, const std::vector<unsigned int>& q2){
    for(int i=0; i<size_x; i++){
        if(q1[i] != q2[i]){
            return false;
        }
    }
    return true;
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