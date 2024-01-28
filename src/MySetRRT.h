#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include "Eigen/Dense"
#include "MyODE.h"
#include "Graph.h"
#include "Keepout.h"
// #include "KeepoutRegion.h"

struct Element
{
    int node;
    double h;
};

class MySetRRT{
    public:
        MyODE ode;
        Eigen::VectorXd x_init;
        Eigen::VectorXd x_goal;
        std::vector< std::pair<double, double> > U_bounds;
        double tstep_Traj;
        int max_nodes;
        std::vector<int> node_path;
        double timeOfFlight;

        std::vector<int> tree;
        Graph graph;
        std::unordered_map<int, Eigen::VectorXd> node_to_coord;
        std::map<std::pair<int, int>, std::vector<double>> edge_to_control;
        std::vector<KEEPOUT::Keepout> keepouts;
        double max_planningtime;
        double cost_threshold;

        void problem_setup();
        void keepouts_setup(std::vector<KEEPOUT::Keepout> kps);
        void max_planningtime_setup(const int time);
        void optimization_setup(const double threshold);
        void plan(const Eigen::VectorXd xi, const Eigen::VectorXd xf);
        void write_solution_data(const std::string filename);
        void write_keepouts_data(const std::string filename);
        std::vector< Eigen::VectorXd > construct_trajectory(const std::string filename);

    private:
        int select_node();
        std::vector<double> get_control_sample();
        Eigen::VectorXd* get_q_sample_ptr(const Eigen::VectorXd& x0, 
                                          const std::vector<double>& U_sample, 
                                          const int node);
        void connect_node(Eigen::VectorXd* state_sample_ptr, const int node, 
                          int& node_count, const std::vector<double>& U_sample);
        bool isLand(Eigen::VectorXd* state_sample_ptr);
        bool isGoal(Eigen::VectorXd* state_sample_ptr, const Eigen::VectorXd xf);
        bool isInKeepOut(std::vector<Eigen::VectorXd> traj);
        void checkKeepOut(Eigen::VectorXd state);
        void trace_path(const int node_goal);
        double trace_timeOfFlight(const int node_goal);
        Eigen::VectorXd get_x_new(const std::vector<Eigen::VectorXd>);

        int getRandomInteger(int N);
        double getRandomDouble(double x_min, double x_max);
        void print_solution_data(const std::vector< std::vector<double> > data);
        std::vector<Eigen::VectorXd> read_solution_data(const std::string filename);
};