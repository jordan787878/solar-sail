#include "OdeVirtual.h"


Eigen::VectorXd OdeVirtual::generateRandomVector(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance) {
    // Create an Eigen vector of the specified size
    Eigen::VectorXd random_vector(mean.size());
    // Fill the vector with random values drawn from the normal distribution
    for (int i = 0; i < mean.size(); ++i) {
        random_vector(i) = dist(gen);
    }
    return random_vector;
}