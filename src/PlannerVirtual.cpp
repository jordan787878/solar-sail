#include "PlannerVirtual.h"
#include<random>


void PlannerVirtual::set_cost_threshold(double threshold){
    cost_threshold = threshold;
}


void PlannerVirtual::set_plan_time_max(double time){
    plan_time_max = time;
}


bool PlannerVirtual::exceed_plan_time_max(std::chrono::time_point<std::chrono::high_resolution_clock> startTime){
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime);
    double elapsedSeconds = elapsedTime.count();
    // std::cout << "[DEBUG] plan time: " << elapsedSeconds << "\n";
    if (elapsedSeconds >= plan_time_max) {
        std::cout << "exceeded max. planning time: " << plan_time_max << " sec.\n";
        return true;
    }
    return false;
}


void PlannerVirtual::print_eigen_vector(const Eigen::VectorXd v){
    std::cout << "vector: ";
    for(int i = 0; i < v.size(); i ++){
        std::cout << v[i] << " ";
    }
	std::cout << "\n";
}


int PlannerVirtual::getRandomInt(int N){
    // Seed the random number generator with a random device
    std::random_device rd;
    std::mt19937 gen(rd());

    // Define the distribution for integers in the range [0, N-1]
    std::uniform_int_distribution<int> distribution(0, N - 1);

    // Generate a random integer
    return distribution(gen);
}


int PlannerVirtual::getRandomIntBounded(const int min, const int max) {
    // Create a random number generator engine
    std::random_device rd;  // Random seed from hardware entropy
    std::mt19937 gen(rd());  // Mersenne Twister engine

    // Define a distribution for integer values between min and max (inclusive)
    std::uniform_int_distribution<int> dis(min, max);

    // Generate and return a random integer
    return dis(gen);
}