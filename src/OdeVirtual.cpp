#include "OdeVirtual.h"
#include <random>


Eigen::VectorXd OdeVirtual::generateRandomVector(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance) {
    // Initialize random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Create normal distribution for each dimension
    std::vector<std::normal_distribution<double>> distributions;
    for (int i = 0; i < mean.size(); ++i) {
        distributions.emplace_back(mean[i], std::sqrt(covariance(i, i)));
    }
    
    // Generate random values for each dimension
    Eigen::VectorXd random_vector(mean.size());
    for (int i = 0; i < mean.size(); ++i) {
        random_vector[i] = distributions[i](gen);
    }
    
    return random_vector;
}