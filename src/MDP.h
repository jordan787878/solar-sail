# pragma once

#include <iostream>
#include <vector>
#include "PlannerVirtual.h"
#include <Eigen/Dense>
#include <map>
#include <tuple>
#include <random>
#include <iomanip>  

class MDP{
    public:
        PlannerVirtual* planner_pointer;
        std::vector<Eigen::VectorXd> x_goals;
        int size_x;
        int size_u;
        int number_per_state;
        int number_per_action;
        unsigned int number_Q;
        unsigned int number_Sigma;
        Eigen::VectorXd dx;
        Eigen::VectorXd du;
        std::vector<unsigned int> nodes_Q;
        std::map<unsigned int, Eigen::VectorXd> discrete_state_to_control;
        std::vector< std::map<unsigned int, double> >* matrix;
        bool is_set_transitions_success;
        std::map<unsigned int, bool> map_to_is_goal_node;
        std::vector<double>* J;
        double gamma = 0.9;

        MDP(PlannerVirtual* pointer, const int n_per_state, const int n_per_action);

        void construct_discrete_state(const std::vector<Eigen::VectorXd>& traj);
        std::vector<Eigen::VectorXd> debug_discrete_state();

        void construct_transition();

        void value_iteration();

        void synthesize_control();

        std::tuple<bool, Eigen::VectorXd> get_feedback_control(const Eigen::VectorXd& s);

        std::vector<Eigen::VectorXd> write_transition();

    protected:
        void set_transitions(unsigned int n_q, unsigned int n_sigma, 
                             int number_samples, double time_duration,
                             bool is_process_noise, bool is_check_unsafe);

        std::vector<unsigned int> map_Nq_to_q(const unsigned int& Nq);
        std::vector<unsigned int> map_Nsigma_to_sigma(const unsigned int& Nsigma);
        unsigned int map_q_to_Nq(const std::vector<unsigned int>& q);
        Eigen::VectorXd map_q_to_x(const std::vector<unsigned int>& q);
        Eigen::VectorXd map_sigma_to_u(const std::vector<unsigned int>& sigma);
        std::vector<unsigned int> map_x_to_q(const Eigen::VectorXd& x);
        std::vector<unsigned int> map_u_to_sigma(const Eigen::VectorXd& u);
        unsigned int map_NqNsigma_to_row(const unsigned int& Nq, const unsigned int& Nsigma);
        std::tuple<unsigned int, unsigned int> map_row_to_NqNsigma(const unsigned int& row);

        void log_vector(const Eigen::VectorXd& v);
        void log_vector(const std::vector<unsigned int>& v);

    private:
        void test_mapping();
        bool is_same_q(const std::vector<unsigned int>& q1, const std::vector<unsigned int>& q2);
        std::vector<Eigen::VectorXd> get_neighbor_points(const Eigen::VectorXd& x);
        std::vector<Eigen::VectorXd> get_boader_points(const Eigen::VectorXd& x);
        double getRandomDouble(double x_min, double x_max);
        double reward(const unsigned int& i_Nq, const unsigned int& i_Nsigma);
        void add_Q_neighbors(const std::vector<unsigned int> nodes_Q_pivot);

        void print_J(const std::vector<double>& J);
};