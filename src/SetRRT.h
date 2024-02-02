# pragma once

#include "PlannerVirtual.h"
#include "OdeSolver.h"
#include "Graph.h"
#include<map>

class SetRRT : public PlannerVirtual{

public:
    Graph graph;
    std::vector<int> nodes;
    std::map<int, Eigen::VectorXd> node_to_state;
    std::map<std::pair<int, int>, Eigen::VectorXd> edge_to_control;

    SetRRT(std::string name) : PlannerVirtual(name){}

    std::vector<Eigen::VectorXd> plan(const Eigen::VectorXd& x_init, 
                                      const std::vector<Eigen::VectorXd> x_goals) override;

    std::vector<Eigen::VectorXd> construct_trajectory(const std::vector<Eigen::VectorXd>& solution, 
                                                      const std::vector<Eigen::VectorXd> x_goals={},
                                                      bool is_process_noise=false) override;

    void link_ode_solver_pointer(OdeSolver* pointer);

protected:
    void init_plan(const Eigen::VectorXd&);

    Eigen::VectorXd get_control_sample(const Eigen::VectorXd, const Eigen::VectorXd);

    int select_node();

    Eigen::VectorXd* get_state_new_pointer(const int, Eigen::VectorXd&, const std::vector<Eigen::VectorXd>&);

    void connect(const int, const Eigen::VectorXd, const Eigen::VectorXd);

    void print_nodes_and_states();

    std::vector<Eigen::VectorXd> construct_solution();



};