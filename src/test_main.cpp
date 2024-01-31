// src/main.cpp
#include <iostream>
#include <chrono>
#include "myownlibrary.h"
#include "MyODE.h"
#include "Graph.h"
#include "MySetRRT.h"
#include "MyResult.h"
#include "helperfunctions.h"
// #include "Keepout.h"

void test_ode_class(){
    MyODE ode;
    ode.init_params();

    Eigen::VectorXd x0(6);
    // x0 << -0.106189341, 0.0, 0.110648109, 0.0, 0.842527695, 0.0;  // orbit 2
    x0 << 0.0019, 0.0488, 0.0, 0.0, 0.0, 4.7163; // orbit 4
    std::string filename1 = "/Users/chko1829/src/SolarSailLanding/file_dump/MySetRRT/orbit4_valid_zeroU.csv";
    std::string filename2 = "/Users/chko1829/src/SolarSailLanding/file_dump/MySetRRT/orbit4_valid_dataU.csv";
    std::string ufilename = "/Users/chko1829/src/SolarSailLanding/file_dump/from_matlab/orbit4_U.csv";

    ode.validate(x0, "", filename1);
    ode.validate(x0, ufilename, filename2);
}

void test_graph_class(){
    Graph graph;

    // Connect nodes
    graph.connect(1, 2);
    graph.connect(2, 11);
    graph.connect(3, 7);
    graph.connect(7, 9);

    // Display results
    std::cout << "Parent of 2: " << graph.findParent(2) << std::endl; // Expected output: 11
    std::cout << "Parent of 3: " << graph.findParent(3) << std::endl; // Expected output: 7
    std::cout << "Parent of 7: " << graph.findParent(7) << std::endl; // Expected output: 9
}

void test_mysetrrt_construct_traj(){
    std::string solutionfile = "/Users/chko1829/src/SolarSailLanding/file_dump/uncertainty/orbit2_omplRRT_solution_nom.csv";
    std::string filename = "/Users/chko1829/src/SolarSailLanding/file_dump/uncertainty/orbit2_omplRRT_traj_noise.csv";

    MySetRRT rrtplanner;
    rrtplanner.problem_setup();
    rrtplanner.ode.set_W_intenstiy(100.0);
    std::vector<Eigen::VectorXd> trajectory = rrtplanner.construct_trajectory(solutionfile);
    rrtplanner.ode.write_traj_csv(trajectory, filename);
}

void test_mysetrrt_class(){
    MySetRRT rrtplanner;
    rrtplanner.problem_setup();
    rrtplanner.max_planningtime_setup(10000);

    // start state
    Eigen::VectorXd x_start(6);
    // x_start << 0.127680370, 0.0, 0.084952359, 0.0, 1.445775202, 0.0;  // orbit 1
    // x_start << -0.106189341, 0.0, 0.110648109, 0.0, 0.842527695, 0.0; // orbit 2
    x_start << 0.0019, 0.0488, 0.0, 0.0, 0.0, 4.7163; // orbit 4

    // goal state
    double km2m = 1000.0;
    std::vector<double> goal_accurate = rrtplanner.ode.sphere_to_cartesian(250.0, 0.0, 0.0);
    std::cout << "x goal pos (X,Y,Z) [meter]: ";
    rrtplanner.ode.log_vector(goal_accurate);
    Eigen::VectorXd x_goal(6);
    x_goal << goal_accurate[0]/km2m/rrtplanner.ode.unit_length, 
              goal_accurate[1]/km2m/rrtplanner.ode.unit_length,
              goal_accurate[2]/km2m/rrtplanner.ode.unit_length,
              0.0, 0.0, 0.0;

    rrtplanner.plan(x_start, x_goal);

    if(rrtplanner.node_path.size() > 0){
        // std::string solutionfile = "/Users/chko1829/src/SolarSailLanding/file_dump/MySetRRT/orbit4_mysetRRT_solution.csv";
        std::string solutionfile = "outputs/orbit4_mysetRRT_solution.csv";
        // write solution and use it to simulate trajectory
        rrtplanner.write_solution_data(solutionfile);
        std::vector<Eigen::VectorXd> trajectory = rrtplanner.construct_trajectory(solutionfile);
        
        // std::string filename = "/Users/chko1829/src/SolarSailLanding/file_dump/MySetRRT/orbit4_mysetRRT_traj.csv";
        std::string filename = "outputs/orbit4_mysetRRT_traj.csv";
        rrtplanner.ode.write_traj_csv(trajectory, filename);
    }
}


void test_mysetrrt_class_optimal_standard(){
    std::cout << "[test optimal planner]\n";

    MySetRRT rrtplanner;
    rrtplanner.problem_setup();
    rrtplanner.max_planningtime_setup(1200);
    rrtplanner.optimization_setup(0.4);

    // start state
    Eigen::VectorXd x_start(6);
    // x_start << 0.127680370, 0.0, 0.084952359, 0.0, 1.445775202, 0.0;  // orbit 1
    // x_start << -0.106189341, 0.0, 0.110648109, 0.0, 0.842527695, 0.0; // orbit 2
    x_start << 0.0019, 0.0488, 0.0, 0.0, 0.0, 4.7163; // orbit 4

    // goal state
    double km2m = 1000.0;
    std::vector<double> goal_accurate = rrtplanner.ode.sphere_to_cartesian(250.0, 0.0, 0.0);
    std::cout << "x goal pos (X,Y,Z) [meter]: ";
    rrtplanner.ode.log_vector(goal_accurate);
    Eigen::VectorXd x_goal(6);
    x_goal << goal_accurate[0]/km2m/rrtplanner.ode.unit_length, 
              goal_accurate[1]/km2m/rrtplanner.ode.unit_length,
              goal_accurate[2]/km2m/rrtplanner.ode.unit_length,
              0.0, 0.0, 0.0;

    rrtplanner.plan(x_start, x_goal);

    if(rrtplanner.planSuccess){
        // std::string solutionfile = "/Users/chko1829/src/SolarSailLanding/file_dump/MySetRRT/orbit4_mysetRRT_solution.csv";
        std::string solutionfile = "outputs/orbit4_mysetRRT_solution.csv";
        // write solution and use it to simulate trajectory
        rrtplanner.write_solution_data(solutionfile);
        std::vector<Eigen::VectorXd> trajectory = rrtplanner.construct_trajectory(solutionfile);
        
        // std::string filename = "/Users/chko1829/src/SolarSailLanding/file_dump/MySetRRT/orbit4_mysetRRT_traj.csv";
        std::string filename = "outputs/orbit4_mysetRRT_traj.csv";
        rrtplanner.ode.write_traj_csv(trajectory, filename);
    }
}

void planner_AO(MySetRRT &rrtplanner, Eigen::VectorXd x_start, Eigen::VectorXd x_goal, std::string solutionfile, std::string trajfile){

    // Asymptoptically Optimization
    bool breakplan = false;
    double time_threshold = 2.0;
    int plan_time = 200;
    int max_plan_time = 2*3600;
    auto startTime = std::chrono::high_resolution_clock::now();

    // init adaptive planning time, best time of flight
    int adapt_plan_time = plan_time;
    double best_timeOfFlight = std::numeric_limits<double>::infinity();

    while(true){
        // check Total Time
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime);
        double elapsedSeconds = elapsedTime.count(); std::cout << "Total Plan Time: " << elapsedSeconds << "\n";
        if(elapsedSeconds >= max_plan_time){
            breakplan = true;
            std::cout << "Exceeds Total Plan Time\n";
            return;
        }

        // planning
        rrtplanner.problem_setup();
        rrtplanner.max_planningtime_setup(adapt_plan_time);
        rrtplanner.optimization_setup(time_threshold);
        rrtplanner.plan(x_start, x_goal);

        // solved
        if(rrtplanner.planSuccess){
            rrtplanner.write_solution_data(solutionfile);
            std::vector<Eigen::VectorXd> trajectory = rrtplanner.construct_trajectory(solutionfile);
            rrtplanner.ode.write_traj_csv(trajectory, trajfile);
            
            best_timeOfFlight = rrtplanner.timeOfFlight;
            std::cout << "Solution improves. Best Solution: " << best_timeOfFlight << "\n";
            // incrementally update the time threshold
            time_threshold = best_timeOfFlight;

            // TODO: the next rrtplanner can "inherit" the tree of the success planner
        } 
        // un-solved
        else{
            std::cout << "no solution\n";
            // adapt the planning time
            adapt_plan_time = adapt_plan_time + 100;
        }
    }
}


void test_mysetrrt_class_optimal_asymptoptic(){
    std::cout << "[test optimal planner asymptoptic]\n";

    MySetRRT rrtplanner;

    // start state
    Eigen::VectorXd x_start(6);
    // x_start << 0.127680370, 0.0, 0.084952359, 0.0, 1.445775202, 0.0;  // orbit 1
    // x_start << -0.106189341, 0.0, 0.110648109, 0.0, 0.842527695, 0.0; // orbit 2
    x_start << 0.0019, 0.0488, 0.0, 0.0, 0.0, 4.7163; // orbit 4

    // goal state
    Eigen::VectorXd x_goal(6);
    x_goal << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // specify solution and trajectory files
    std::string solutionfile = "outputs/orbit4_mysetRRT_solution.csv";
    std::string trajfile = "outputs/orbit4_mysetRRT_traj.csv";
    planner_AO(rrtplanner, x_start, x_goal, solutionfile, trajfile);
}


void BenchMark_mysetrrt_class(){
    // Number of Trials
    int N_trials = 100;
    std::vector<PlannerResult> result_data;

    for(int i=0; i<N_trials; i++){

        std::cout << "run: " << i+1 << "\n";

        // Init result
        PlannerResult result;

        MySetRRT rrtplanner;
        rrtplanner.problem_setup();
        rrtplanner.max_planningtime_setup(60);

        // start state
        Eigen::VectorXd x_start(6);
        // x_start << -0.106189341, 0.0, 0.110648109, 0.0, 0.842527695, 0.0; // orbit 2
        x_start << 0.0019, 0.0488, 0.0, 0.0, 0.0, 4.7163; // orbit 4

        // goal state
        Eigen::VectorXd x_goal(6);
        x_goal << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        // Start the clock
        auto start = std::chrono::high_resolution_clock::now();
        
        // Solving
        rrtplanner.plan(x_start, x_goal);
        
        // Stop the clock
        auto stop = std::chrono::high_resolution_clock::now();
        // Calculate the duration in seconds as a double
        std::chrono::duration<double> duration = stop - start;
        double computationSecs = duration.count();

        // Post-process
        if(rrtplanner.node_path.size() > 0){
            // write solution and use it to simulate trajectory
            std::string solutionfile = "/Users/chko1829/src/SolarSailLanding/file_dump/MySetRRT/orbit4_mysetRRT_solution.csv";
            rrtplanner.write_solution_data(solutionfile);
            std::vector<Eigen::VectorXd> trajectory = rrtplanner.construct_trajectory(solutionfile);
        
            std::string filename = "/Users/chko1829/src/SolarSailLanding/file_dump/MySetRRT/orbit4_mysetRRT_traj.csv";
            rrtplanner.ode.write_traj_csv(trajectory, filename);

            result.computationTime = computationSecs;
            result.isSuccess = 1;
            result.trajLength = cal_trajLength(trajectory);
            result.timeOfFlight = rrtplanner.timeOfFlight;
            result.treeSize = rrtplanner.tree.size();
        }
        else{
            std::cout << "Fail\n";
        }

        // Store result data
        result_data.push_back(result);
    }

    // Print-out result data
    for(int i=0; i<int(result_data.size()); i++){
        std::cout << "result " << i << ", ";
        std::cout << "Comp Time: " << result_data[i].computationTime << ", ";
        std::cout << "Success: " << result_data[i].isSuccess << ", ";
        std::cout << "Trajectory Length: " << result_data[i].trajLength << ", ";
        std::cout << "Time Of Flight: " << result_data[i].timeOfFlight << ", ";
        std::cout << "Tree Size: " << result_data[i].treeSize << "\n";
    }
    write_result_csv(result_data, "/Users/chko1829/src/SolarSailLanding/file_dump/MySetRRT/orbit4_mysetRRT_result.csv");
}



void test_mysetrrt_class_withkeepout(){
    MySetRRT rrtplanner;
    rrtplanner.problem_setup();

    // Keepout Regions
    // std::vector<KEEPOUT::Keepout> keepouts = HELPER::genearte_keepouts(2);       // pre-specified 
    std::vector<KEEPOUT::Keepout> keepouts = HELPER::genearte_random_keepouts(4);   // randomly generated
    rrtplanner.keepouts_setup(keepouts);

    // Max Planning time
    rrtplanner.max_planningtime_setup(300);

    // start state
    Eigen::VectorXd x_start(6);
    // x_start << 0.127680370, 0.0, 0.084952359, 0.0, 1.445775202, 0.0;  // orbit 1
    x_start << -0.106189341, 0.0, 0.110648109, 0.0, 0.842527695, 0.0; // orbit 2
    // x_start << 0.0019, 0.0488, 0.0, 0.0, 0.0, 4.7163; // orbit 4

    // goal state
    double km2m = 1000.0;
    std::vector<double> goal_accurate = rrtplanner.ode.sphere_to_cartesian(250.0, 0.0, 0.0);
    std::cout << "x goal pos (X,Y,Z) [meter]: ";
    rrtplanner.ode.log_vector(goal_accurate);
    Eigen::VectorXd x_goal(6);
    x_goal << goal_accurate[0]/km2m/rrtplanner.ode.unit_length, 
              goal_accurate[1]/km2m/rrtplanner.ode.unit_length,
              goal_accurate[2]/km2m/rrtplanner.ode.unit_length,
              0.0, 0.0, 0.0;

    rrtplanner.plan(x_start, x_goal);

    if(rrtplanner.node_path.size() > 0){
        std::string solutionfile = "/Users/chko1829/src/SolarSailLanding/file_dump/MySetRRT/orbit2_mysetRRT_solution.csv";
        // write solution and use it to simulate trajectory
        rrtplanner.write_solution_data(solutionfile);
        std::vector<Eigen::VectorXd> trajectory = rrtplanner.construct_trajectory(solutionfile);
        
        std::string filename = "/Users/chko1829/src/SolarSailLanding/file_dump/MySetRRT/orbit2_mysetRRT_traj.csv";
        rrtplanner.ode.write_traj_csv(trajectory, filename);

        std::string filename_keepouts = "/Users/chko1829/src/SolarSailLanding/file_dump/MySetRRT/orbit2_keepouts.csv";
        // write keepouts
        rrtplanner.write_keepouts_data(filename_keepouts);
    }
}


void mysetrrt_class_withnoise(std::string test_label){
    std::string solutionfile = "/Users/chko1829/src/SolarSailLanding/file_dump/replanning_monte/";
    solutionfile.append(test_label); solutionfile.append("solution");

    int max_planning_time = 120;
    int max_replanning_counts = 2;
    double total_control_duration = 0.0;
    double max_total_control_duration = 2.0;

    double km2m = 1000.0;
    std::vector<Eigen::VectorXd> trajectory;
    double W_test = static_cast<double>(HELPER::getRandomInt(0,200));
    
    std::string trajfile =     "/Users/chko1829/src/SolarSailLanding/file_dump/replanning_monte/";
    trajfile.append(test_label); trajfile.append("traj.csv");

    std::string valfile =      "/Users/chko1829/src/SolarSailLanding/file_dump/replanning_monte/";
    valfile.append(test_label); valfile.append("values.csv");

    // Start state
    Eigen::VectorXd x_start(6);
    // x_start << 0.127680370, 0.0, 0.084952359, 0.0, 1.445775202, 0.0;  // orbit 1
    x_start << -0.106189341, 0.0, 0.110648109, 0.0, 0.842527695, 0.0; // orbit 2
    // x_start << 0.0019, 0.0488, 0.0, 0.0, 0.0, 4.7163; // orbit 4
    // random start state by random time propagation
    Eigen::VectorXd x_start_random = HELPER::generate_random_initial_states(x_start);

    // Goal state
    Eigen::VectorXd x_goal(6);
    x_goal << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // Data
    std::vector<std::string> valueNames = {"W","Success"};
    std::vector<int> values = {static_cast<int>(W_test), 0};
    std::cout << "W: " << W_test << "\n";

    // Init
    int k = 0;
    Eigen::VectorXd xi = x_start_random;
    while(true){
        // planning stage
        int replan_count = 0;
        std::vector<double> u;
        double controlDuration;

        MySetRRT rrtplanner;
        while(true){
            rrtplanner.problem_setup();
            rrtplanner.max_planningtime_setup(max_planning_time);
            rrtplanner.ode.set_W_intenstiy(0.0);
            // std::cout << "planning\n";

            rrtplanner.plan(xi, x_goal); // std::cout << "node path size after planning:" << rrtplanner.node_path.size() << "\n";
            std::cout << "tree size: " << rrtplanner.tree.size() << "\n";

            if(rrtplanner.node_path.size() > 1){
                // get the first control
                // std::cout << "Plan Success\n";
                // write solution file
                rrtplanner.write_solution_data(solutionfile+std::to_string(k)+".csv");
                std::pair<int, int> edge(rrtplanner.node_path[0], rrtplanner.node_path[1]);
                std::vector<double> u_sample = rrtplanner.edge_to_control[edge];
                u.push_back(u_sample[0]);  u.push_back(u_sample[1]);
                controlDuration = u_sample[2];
                total_control_duration = total_control_duration + controlDuration;
                break;
            }

            replan_count = replan_count + 1;
            std::cout << "replan count: " << replan_count << "\n";
            
            // check replan count
            if(replan_count >= max_replanning_counts){
                std::cout << "[FAIL]: replanning exceed max. time " << max_planning_time * max_replanning_counts << " sec.\n";
                if(trajectory.size() > 1){
                    std::cout << "write partial trajectory\n";
                    rrtplanner.ode.write_traj_csv(trajectory, trajfile);
                }
                HELPER::writeValueToCSV(valfile, valueNames, values);
                values[1] = -1;
                HELPER::writeValueToCSV(valfile, valueNames, values);
                return;
            }
        }

        // check total control duration
        std::cout << "total control duration: " << total_control_duration << "\n";
        if(total_control_duration > max_total_control_duration){
            std::cout << "[FAIL]: total control duration exceed max. time " << max_total_control_duration << " sec.\n";
            if(trajectory.size() > 1){
                std::cout << "write partial trajectory\n";
                rrtplanner.ode.write_traj_csv(trajectory, trajfile);
            }
            HELPER::writeValueToCSV(valfile, valueNames, values);
            values[1] = -2;
            HELPER::writeValueToCSV(valfile, valueNames, values);
            return;
        }

        // simulate stage (over t_sim time) real trajectory!
        // std::cout << "x(" << k << ") "; rrtplanner.ode.log_vector(xi);
        rrtplanner.ode.set_W_intenstiy(W_test);
        std::vector<Eigen::VectorXd> traj_segment = rrtplanner.ode.rungeKutta(0.0, xi, u, rrtplanner.tstep_Traj, controlDuration);
        trajectory.insert(trajectory.end(), traj_segment.begin(), traj_segment.end());
        xi = traj_segment.back();
        
        k = k + 1;
        
        // std::cout << "apply u "; rrtplanner.ode.log_vector(u);
        // std::cout << "over time: " << controlDuration << "\n";
        // std::cout << "reaches x(" << k << ") "; rrtplanner.ode.log_vector(xi);

        // reach goal check (pure landing)
        double x =  abs(xi[0]*rrtplanner.ode.unit_length*km2m);
        double y =  abs(xi[1]*rrtplanner.ode.unit_length*km2m);
        double z =  abs(xi[2]*rrtplanner.ode.unit_length*km2m);
        double r = pow(x*x + y*y + z*z, 0.5);
        std::cout << "radius becomes: " << r << "\n";
        if(r <= 250.0){
            std::cout << "Success\n";
            rrtplanner.ode.write_traj_csv(trajectory, trajfile);
            values[1] = 1;
            HELPER::writeValueToCSV(valfile, valueNames, values);
            return;
        }
    }
}

void test_mysetrrt_class_withnoise(){
    for(int i=0; i<100; i++){
        std::string test_label = std::to_string(i);
        test_label.append("/");
        mysetrrt_class_withnoise(test_label);
    }
}


int main() {
    
    // test_ode_class();

    // test_graph_class();

    // test_mysetrrt_class();

    // test_mysetrrt_class_optimal_standard();

    test_mysetrrt_class_optimal_asymptoptic();

    // test_mysetrrt_construct_traj();

    // BenchMark_mysetrrt_class();

    // test_mysetrrt_class_withkeepout();

    // test_mysetrrt_class_withnoise();

    std::cout << "[Run Complete]\n";

    return 0;
}

