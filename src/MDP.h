# pragma once

#include <iostream>
#include <vector>
#include "PlannerVirtual.h"
#include <Eigen/Dense>
#include <map>
#include <tuple>
#include <random>
#include <iomanip>  
#include <set>
#include <atomic>


class MDP{
    public:
        PlannerVirtual* planner_pointer;
        std::vector<Eigen::VectorXd> x_goals;
        int size_x;
        int size_u;
        int number_per_state;
        int number_per_action;
        unsigned int size_total_discrete_state;
        unsigned int size_total_discrete_action;
        Eigen::VectorXd dx;
        Eigen::VectorXd du;
        std::vector<unsigned int> included_discrete_states;
        std::map<unsigned int, Eigen::VectorXd> discrete_state_to_control;
        std::vector< std::map<unsigned int, double> >* transition_matrix_pointer;
        bool is_set_transitions_success;
        std::set<unsigned int> set_index_goal_q; // std::map<unsigned int, bool> index_discrete_state_to_bool_isgoal;
        std::vector<double>* J;
        int k_layer = 7;
        double gamma = 1.0;
        int number_samples = 1;

        MDP(PlannerVirtual* pointer, const int n_per_state, const int n_per_action);

        void construct_discrete_state(const std::vector<Eigen::VectorXd>& traj);
        void read_discrete_state(std::string filename);
                
        void construct_transition();
        void process_range(
                unsigned int start, 
                unsigned int end, 
                unsigned int size_total_discrete_action, 
                unsigned int number_samples, 
                unsigned int time_duration, 
                bool is_process_noise, 
                bool is_check_unsafe,
                std::atomic<unsigned int>& progress);
        void read_transition(std::string filename);

        void value_iteration();

        void synthesize_control();

        void test_synthesis_control(const Eigen::VectorXd &s);

        std::tuple<bool, Eigen::VectorXd> get_feedback_control(const Eigen::VectorXd& s);

        std::vector<Eigen::VectorXd> write_discrete_state();
        std::vector<Eigen::VectorXd> write_discrete_state_info();
        void write_transition(std::string filename);
        std::vector<Eigen::VectorXd> write_synthesis_control();
        void print_goal_node_index(const Eigen::VectorXd& s);

        void set_transitions(unsigned int n_q, unsigned int n_sigma, 
                             int number_samples, double time_duration,
                             bool is_process_noise, bool is_check_unsafe);

    protected:
        std::vector<unsigned int> map_labelq_to_q(const unsigned int& labelq);
        std::vector<unsigned int> map_labelsigma_to_sigma(const unsigned int& labelsigma);
        unsigned int map_q_to_labelq(const std::vector<unsigned int>& q);
        unsigned int map_sigam_to_labelsigma(const std::vector<unsigned int>& sigma);
        Eigen::VectorXd map_q_to_x(const std::vector<unsigned int>& q);
        Eigen::VectorXd map_sigma_to_u(const std::vector<unsigned int>& sigma);
        std::vector<unsigned int> map_x_to_q(const Eigen::VectorXd& x);
        std::vector<unsigned int> map_u_to_sigma(const Eigen::VectorXd& u);
        unsigned int map_indexq_and_indexsigma_to_row(const unsigned int& labelq, const unsigned int& labelsigma);
        std::tuple<unsigned int, unsigned int> map_row_to_indexq_and_indexsigma(const unsigned int& row);
        void log_vector(const Eigen::VectorXd& v);
        void log_vector(const std::vector<unsigned int>& v);

    private:
        void add_labelq(const unsigned int& labelq);
        void test_mapping();
        std::vector<Eigen::VectorXd> get_neighbor_points(const Eigen::VectorXd& x);
        std::vector<Eigen::VectorXd> get_boader_points(const Eigen::VectorXd& x);
        double getRandomDouble(double x_min, double x_max);
        double reward(const unsigned int& i_labelq, const unsigned int& i_labelsigma);
        void add_neighbor_discrete_states(const std::vector<unsigned int> included_discrete_states_pivot);
        void construct_index_discrete_state_to_bool_isgoal();
        bool is_indexq_goal(const unsigned int& index_q);
        void print_J(const std::vector<double>& J);
        void init_J();
};