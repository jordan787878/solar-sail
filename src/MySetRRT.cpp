#include "MySetRRT.h"
#include <random>
#include <chrono>
#include <cmath>

void MySetRRT::problem_setup(){
    ode.init_params();

    // Control bounds (u_min, u_max)
    std::pair<double, double> U1_bounds(-0.5*M_PI, 0.5*M_PI);
    U_bounds.push_back(U1_bounds);
    std::pair<double, double> U2_bounds(0, 2*M_PI);
    U_bounds.push_back(U2_bounds);
    std::pair<double, double> U3_bounds(0.001, 0.05);
    U_bounds.push_back(U3_bounds);

    tstep_Traj = 0.0001;

    max_nodes = 1000000;

    // default max planning time (sec)
    max_planningtime = 20;

    // default cost threshold
    cost_threshold = -1.0;

    timeOfFlight = 0.0;

    planSuccess = false;

    // clean data structure
    tree.clear();
    graph.cleanGraph();
    node_to_coord.clear();
    edge_to_control.clear();
    keepouts.clear();
    node_path.clear();
}

void MySetRRT::keepouts_setup(std::vector<KEEPOUT::Keepout> kps){
    keepouts = kps;
}

void MySetRRT::max_planningtime_setup(const int time){
    max_planningtime = time;
    std::cout << "planning time setup: " << max_planningtime << "\n";
}

void MySetRRT::optimization_setup(const double threshold){
    cost_threshold = threshold;
    std::cout << "optimization threshold setup: " << cost_threshold << "\n";
}

void MySetRRT::plan(const Eigen::VectorXd xi, const Eigen::VectorXd xf){
    node_path.clear();
    x_init = xi;
    x_goal = xf;

    // init planning time
    auto startTime = std::chrono::high_resolution_clock::now();

    // init tree
    tree.push_back(0);
    node_to_coord[0] = x_init;
    int node_count = 1;

    // init checking if keep-out regions do not collide with initial states
    checkKeepOut(x_init);

    while(true){
        // debug
        //std::cout << tree.size() << "\n";

        // select
        const int node_select = select_node();
        const Eigen::VectorXd q_select = node_to_coord[node_select]; //std::cout << "Tree size " << tree.size() << "\n"; std::cout << "SELECT node " << node_select << " vector " ; ode.log_vector(q_select);

        // sample
        std::vector<double> control_sample = get_control_sample(); //std::cout << "SAMPLE\n";

        // extend
        Eigen::VectorXd* q_new_ptr = get_q_sample_ptr(q_select, control_sample, node_select); //std::cout << "EXTEND\n";

        // connect
        connect_node(q_new_ptr, node_select, node_count, control_sample); //std::cout << "CONNECT\n";

        // break loop
        if(isGoal(q_new_ptr, x_goal) && isLand(q_new_ptr)){
            // // debug
            // std::cout << "node count: " << node_count << "\n";
            // // success
            // std::cout << "isGoal: True\n";

            // debug last node
            Eigen::VectorXd x_from_nodecount = node_to_coord[node_count-1];
            std::cout << "goal state from node_count: ";
            ode.log_vector(x_from_nodecount);
            Eigen::VectorXd x_from_ptr = (*q_new_ptr);
            std::cout << "goal state from ptr: ";
            ode.log_vector(x_from_ptr);

            // std::vector<double> x_goal_debug; 
            // x_goal_debug.push_back(x_new[0]*ode.unit_length*1000.0);
            // x_goal_debug.push_back(x_new[1]*ode.unit_length*1000.0);
            // x_goal_debug.push_back(x_new[2]*ode.unit_length*1000.0);
            // std::cout << "land at (X,Y,Z) [meter]: ";
            // ode.log_vector(x_goal_debug);

            trace_path(node_count-1);
            planSuccess = true;
            timeOfFlight = trace_timeOfFlight(node_count-1);
            std::cout << "Time of Flight: " << timeOfFlight << "\n";
            break;
        }
        if(tree.size() >= max_nodes){
            // fail
            std::cout << "exceed max. nodes\n";
            break;
        }
        // Check if the elapsed time exceeds the maximum duration
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime);
        double elapsedSeconds = elapsedTime.count(); //std::cout << "elapsedSeconds: " << elapsedSeconds << "\n";
        if (elapsedSeconds >= max_planningtime) {
            std::cout << "exceeded max. planning time: " << max_planningtime << " sec.\n";
            break;
        }
    }
}

void MySetRRT::write_solution_data(const std::string filename){
    std::vector<Eigen::VectorXd> sol;
    for(int i=0; i<node_path.size(); i++){
        int node = node_path[i];
        if(i < node_path.size()-1){
            Eigen::VectorXd x = node_to_coord[node];
            std::pair<int, int> edge(node, node_path[i+1]);
            std::vector<double> u_sample = edge_to_control[edge];
            std::vector<double> u; u.push_back(u_sample[0]);  u.push_back(u_sample[1]);
            double t_end = u_sample[2];

            Eigen::VectorXd sol_row(9);
            sol_row << x[0], x[1], x[2], x[3], x[4], x[5], u[0], u[1], t_end;
            sol.push_back(sol_row);
        }
        else{
            Eigen::VectorXd x = node_to_coord[node];
            Eigen::VectorXd sol_row(9);
            sol_row << x[0], x[1], x[2], x[3], x[4], x[5], 0, 0, 0;
            sol.push_back(sol_row);
        }
    }
    ode.write_traj_csv(sol, filename);
}

void MySetRRT::write_keepouts_data(const std::string filename){
    std::vector<Eigen::VectorXd> data;
    for(int i=0; i<keepouts.size(); i++){
        Eigen::VectorXd row(4);
        row << keepouts[i].center[0], keepouts[i].center[1], keepouts[i].center[2], keepouts[i].radius;
        data.push_back(row);
    }
    ode.write_traj_csv(data, filename);
}

std::vector< Eigen::VectorXd > MySetRRT::construct_trajectory(const std::string filename){

    const std::vector<Eigen::VectorXd> sol_data = read_solution_data(filename);

    std::vector<Eigen::VectorXd> trajectory;
    Eigen::VectorXd row0 = sol_data[0];
    Eigen::VectorXd x(6);
    x << row0[0], row0[1], row0[2], row0[3], row0[4], row0[5];
    for(int i=0; i<sol_data.size(); i++){
        if(i < sol_data.size()-1){
            // propagate
            // ode.log_vector(x);
            std::vector<double> u; u.push_back(sol_data[i][6]);  u.push_back(sol_data[i][7]);
            // ode.log_vector(u);
            double t_end = sol_data[i][8];

            std::vector<Eigen::VectorXd> traj_segment;
            traj_segment = ode.rungeKutta(0.0, x, u, tstep_Traj, t_end);
            trajectory.insert(trajectory.end(), traj_segment.begin(), traj_segment.end());
            x = trajectory.back();
        }
        else{
            std::cout << "goal state (sim): "; ode.log_vector(x);
        }
    }
    std::cout << "\n";

    return trajectory;
}



int MySetRRT::select_node(){
    int node = getRandomInteger(tree.size()); //std::cout << "MySetRRT.select_node() " << node << "\n";
    return node;
}

std::vector<double> MySetRRT::get_control_sample(){
    std::vector<double> U_sample;
    for(int i=0; i<U_bounds.size(); i++){
        U_sample.push_back(getRandomDouble(U_bounds[i].first, U_bounds[i].second));
    }

    // std::cout << "control sample: "; ode.log_vector(U_sample);

    return U_sample;
}

Eigen::VectorXd* MySetRRT::get_q_sample_ptr(const Eigen::VectorXd& x0, 
                                            const std::vector<double>& U_sample, 
                                            const int node){       
    // node: n_select     
    const double u1 = U_sample[0];
    const double u2 = U_sample[1];
    const double t_end = U_sample[2];

    // optimization
    if(cost_threshold > 0 && (trace_timeOfFlight(node) + t_end) >= cost_threshold){
        return nullptr;
    }


    std::vector<double> u_control; u_control.push_back(u1); u_control.push_back(u2); //std::cout << "get u_control\n";
    std::vector<Eigen::VectorXd> x_traj; //std::cout << "init x_traj\n"; std::cout << "x0 for ode"; ode.log_vector(x0);
    x_traj = ode.rungeKutta(0.0, x0, u_control, tstep_Traj, t_end); //std::cout << "simulate x_traj\n";

    // Check the validity of the edge (the x_traj)
    // for(auto& q: x_traj){
    //     if(isCollision(q)){
    //         return nullptr;
    //     }
    // }
    
    // Get the node ptr
    Eigen::VectorXd x_new = get_x_new(x_traj);
    Eigen::VectorXd* state_sample_ptr = new Eigen::VectorXd(6);
    (*state_sample_ptr) = x_new;

    // check the validity of the new node
    // land but not goal
    // if(isLand(state_sample_ptr)){
    //     if(!isGoal(state_sample_ptr, x_goal)){
    //         std::vector<double> x_goal_debug; 
    //         x_goal_debug.push_back(x_new[0]*ode.unit_length*1000.0);
    //         x_goal_debug.push_back(x_new[1]*ode.unit_length*1000.0);
    //         x_goal_debug.push_back(x_new[2]*ode.unit_length*1000.0);
    //         std::cout << "land but not goal. (X,Y,Z) [meter]: ";
    //         // ode.log_vector(x_goal_debug);
    //         return nullptr;
    //     }
    // }
    // position bounds
    if(abs(x_new[0]) > 1 || abs(x_new[1]) > 1 || abs(x_new[2]) > 1){
        // std::cout << "exceed pos bound\n";
        return nullptr;
    }
    // velocity bounds
    if(abs(x_new[3]) > 50 || abs(x_new[4]) > 50 || abs(x_new[5]) > 50){
        // std::cout << "exceed vel bound\n";
        return nullptr;
    }
    // keepout zone for the entire trajectory
    if(isInKeepOut(x_traj)){
        return nullptr;
    }

    // // Store the 20-minimal element list
    // double d_new = distance_Eu(x_new, x_goal);
    // // Sort the vector based on the h value using a lambda function
    // std::sort(elements.begin(), elements.end(), [](const Element& a, const Element& b) {
    //     return a.h < b.h;
    // });
    // Element e = elements.back();
    // double d_thres = e.h;
    // // std::cout << "size: " << elements.size() << " ";
    // // std::cout << "d_thres: " << d_thres << "\n";
    // if(d_new < d_thres){
    //     if(elements.size() >= max_elements){
    //         elements.pop_back();
    //     }
    //     elements.push_back({node, d_new});
    //     std::cout << "d_new: " << d_new << "\n";
    // }

    // std::cout << "MySetRRT.get_q_sample_ptr() "; ode.log_vector((*state_sample_ptr));

    return state_sample_ptr;
}


void MySetRRT::connect_node(Eigen::VectorXd* state_sample_ptr, const int node, 
                            int& node_count, const std::vector<double>& U_sample){
    if(state_sample_ptr){
        Eigen::VectorXd x0 = node_to_coord[node];
        Eigen::VectorXd x_new = (*state_sample_ptr); 
        // std::cout << "[DEBUG] Valid state sample: "; // log_vector(x_new);
        tree.push_back(node_count);
        node_to_coord[node_count] = x_new; // std::cout << "push node: " << node_count << " to tree\n";
        graph.connect(node_count, node);

        std::pair<int, int> edge(node, node_count);
        edge_to_control[edge] = U_sample;
        // std::cout << node << "->" << node_count << ": " // << U_sample[0] << " " << U_sample[1] << " " << U_sample[2] << "\n";
        // std::cout << "    "; // log_vector(x0);
        // std::cout << " -> "; // log_vector(x_new); // std::cout << "\n";
        node_count = node_count + 1;
    }
}


bool MySetRRT::isLand(Eigen::VectorXd* state_sample_ptr){
    // std::cout << "check isLand\n";
    double r_ast_meter = 250.0;
    double km2m = 1000.0;
    if(state_sample_ptr){
        const Eigen::VectorXd s0 = (*state_sample_ptr);
        double x =  abs(s0[0]*ode.unit_length*km2m);
        double y =  abs(s0[1]*ode.unit_length*km2m);
        double z =  abs(s0[2]*ode.unit_length*km2m);
        double r = pow(x*x + y*y + z*z, 0.5);

        // debug print
        if(r < 1000){
            std::cout << tree.size() << " r: " << r << "\n";
        }
        if(r <= r_ast_meter){
            // std::cout << "isLand True\n";
            return true;
        }
        // std::cout << "not is Land\n";
        return false;
    }
    else{
        return false;
    }
}

bool MySetRRT::isGoal(Eigen::VectorXd* state_sample_ptr, const Eigen::VectorXd sf){
    if(state_sample_ptr){
        // const Eigen::VectorXd s0 = (*state_sample_ptr);
        // Eigen::VectorXd delta_s = s0 - sf;
        // double km2m = 1000.0;
        // double x =  abs(delta_s[0]*ode.unit_length*km2m);
        // double y =  abs(delta_s[1]*ode.unit_length*km2m);
        // double z =  abs(delta_s[2]*ode.unit_length*km2m);
        // double r = pow(x*x + y*y + z*z, 0.5);

        // if(r < 1000.0){
        //     return true;
        // }
        // return false;
        // std::cout << "isGoal True\n";
        return true;
    }
    else{
        return false;
    }
}

bool MySetRRT::isInKeepOut(std::vector<Eigen::VectorXd> traj){
    if(keepouts.size() < 1){
        return false;
    }

    double tol_meter = 100; // [meter] to keep the trajectory away from keep-out regions
    for(const auto& state: traj){
        double km2m = 1000.0;
        double x =  state[0]*ode.unit_length*km2m;
        double y =  state[1]*ode.unit_length*km2m;
        double z =  state[2]*ode.unit_length*km2m;
        Eigen::VectorXd pos(3);
        pos[0] = x; pos[1] = y; pos[2] = z;

        for(int i=0; i<keepouts.size(); i++){
            // std::cout << "keep-out regions: " << i << "\n";
            KEEPOUT::Keepout keepout = keepouts[i];
            Eigen::VectorXd center = keepout.center;
            double radius = keepout.radius;
            Eigen::VectorXd diff = pos - center;
            double distance = diff.norm();
            // std::cout << "distance: " << distance << "\n";
            if(distance <= radius + tol_meter){
                // std::cout << "x_new in keepout: (x,y,z) ";
                //ode.log_vector(pos);
                return true;
            }
        }
    }
    return false;
}

void MySetRRT::checkKeepOut(Eigen::VectorXd state){
    if(keepouts.size() < 1){
        return;
    }

    double tol_meter = 100; // [meter] to keep the trajectory away from keep-out regions
    double km2m = 1000.0;
    double x =  state[0]*ode.unit_length*km2m;
    double y =  state[1]*ode.unit_length*km2m;
    double z =  state[2]*ode.unit_length*km2m;
    Eigen::VectorXd pos(3);
    pos[0] = x; pos[1] = y; pos[2] = z;
    for(int i=0; i<keepouts.size(); i++){
        // std::cout << "keep-out regions: " << i << "\n";
        KEEPOUT::Keepout keepout = keepouts[i];
        Eigen::VectorXd center = keepout.center;
        double radius = keepout.radius;
        Eigen::VectorXd diff = pos - center;
        double distance = diff.norm();
        if(distance <= radius + tol_meter){
            std::cout << "remove keep-out region: " << i << " , because collision with initial state\n";
            keepouts.erase(keepouts.begin() + i);
        }
    }
    std::cout << "valid keep-out regions size: " << keepouts.size() << "\n";
}

void MySetRRT::trace_path(const int node_goal){

    // std::cout << "tree size: " << tree.size() << "\n";
    int n = node_goal;
    while(true){
        // std::cout << n << " ";
        
        node_path.insert(node_path.begin(), n);

        int node_parent = graph.findParent(n);
        if(node_parent < 0){
            break;
        }
        n = node_parent;
    }
}

double MySetRRT::trace_timeOfFlight(const int node_goal){
    // std::cout << "trace time of flight()\n";
    // init
    double result = 0;
    int n = node_goal;
    if(n == 0){
        return result;
    }

    while(true){
        // std::cout << n << " ";

        int node_parent = graph.findParent(n);
        if(node_parent < 0){
            break;
        }
        // summing control durations
        std::pair<int, int> edge(node_parent, n);
        double u_duration = edge_to_control[edge][2];
        result = result + u_duration;

        // update
        n = node_parent;
    }
    // std::cout << "Time of Flight: " << result << "\n";
    return result;
}

Eigen::VectorXd MySetRRT::get_x_new(const std::vector<Eigen::VectorXd> traj){
    // for(int i=0; i< traj.size() - 1; i++){
    //     const Eigen::VectorXd xi = traj[i];
    //     const Eigen::VectorXd xf = traj[i+1];

    //     const Eigen::VectorXd x1 = (xi + xf)/2;
    //     if(ode.isTerminal(x1)){
    //         return x1;
    //     }
    //     const Eigen::VectorXd x2 = (xi + x1)/2;
    //     if(ode.isTerminal(x2)){
    //         return x2;
    //     }
    //     const Eigen::VectorXd x3 = (x1 + xf)/2;
    //     if(ode.isTerminal(x3)){
    //         return x3;
    //     }
    //     // const Eigen::VectorXd x4 = (xi + x2)/2;
    //     // if(ode.isTerminal(x4)){
    //     //     return x4;
    //     // }
    //     // const Eigen::VectorXd x5 = (x2 + x1)/2;
    //     // if(ode.isTerminal(x5)){
    //     //     return x5;
    //     // }
    //     // const Eigen::VectorXd x6 = (x1 + x3)/2;
    //     // if(ode.isTerminal(x6)){
    //     //     return x6;
    //     // }
    //     // const Eigen::VectorXd x7 = (x3 + xf)/2;
    //     // if(ode.isTerminal(x7)){
    //     //     return x7;
    //     // }
    // }
    return traj.back(); // test code
}

int MySetRRT::getRandomInteger(int N){
    // Seed the random number generator with a random device
    std::random_device rd;
    std::mt19937 gen(rd());

    // Define the distribution for integers in the range [0, N-1]
    std::uniform_int_distribution<int> distribution(0, N - 1);

    // Generate a random integer
    return distribution(gen);
}

double MySetRRT::getRandomDouble(double x_min, double x_max) {
    // Seed the random number generator with a random device
    std::random_device rd;
    std::mt19937 gen(rd());

    // Define the distribution for double values in the range [x_min, x_max]
    std::uniform_real_distribution<double> distribution(x_min, x_max);

    double value = distribution(gen);

    double multiplier = std::pow(10.0, 3);
    return std::round(value * multiplier) / multiplier;

    // int N = result/x_min;
    // std::cout << N << "\n";

    // Generate a random double
    // return distribution(gen);
}

void MySetRRT::print_solution_data(const std::vector< std::vector<double> > data){
    // Print the data (optional)
    for (const auto& row : data) {
        for (double value : row) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }
}

std::vector<Eigen::VectorXd> MySetRRT::read_solution_data(const std::string filename){
    // Open the CSV file
    std::ifstream inputFile(filename);

    // Check if the file is open
    if (!inputFile.is_open()) {
        std::cerr << "Error opening the file." << std::endl;
    }

    // Read the file line by line
    std::string line;
    std::vector<Eigen::VectorXd> data;

    while (std::getline(inputFile, line)) {
        // Create a stringstream from the line
        std::istringstream iss(line);

        // Parse the CSV values using a loop
        std::vector<double> values;
        std::string value;

        while (std::getline(iss, value, ',')) {
            values.push_back(std::stod(value));
        }

        // Convert the vector of values to an Eigen::VectorXd
        Eigen::VectorXd vectorData(values.size());
        for (size_t i = 0; i < values.size(); ++i) {
            vectorData(i) = values[i];
        }

        // Store the Eigen::VectorXd in the vector
        data.push_back(vectorData);
    }

    // Close the file
    inputFile.close();

    // Example: Print the data
    // for (const auto& vectorData : data) {
    //     std::cout << "Data: " << vectorData.transpose() << std::endl;
    // }

    return data;
}