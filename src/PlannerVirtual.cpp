#include "PlannerVirtual.h"
#include<random>

void PlannerVirtual::print_eigen_vector(const Eigen::VectorXd v){
    std::cout << "vector: ";
    for(int i = 0; i < v.size(); i ++){
        std::cout << v[i] << " ";
    }
	std::cout << "\n";
}


double PlannerVirtual::getRandomDouble(const double x_min, const double x_max, const int digit){
    // Seed the random number generator with a random device
    std::random_device rd;
    std::mt19937 gen(rd());

    // Define the distribution for double values in the range [x_min, x_max]
    std::uniform_real_distribution<double> distribution(x_min, x_max);

    double value = distribution(gen);
    double multiplier = std::pow(10.0, digit);
    return std::round(value * multiplier) / multiplier;
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