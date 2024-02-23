#pragma once

#include<iostream>
#include<random>
// #include "Keepout.h"
// #include "MyODE.h"
#include "Eigen/Dense"
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <complex>

namespace HELPER{


void log_vector(const Eigen::VectorXd v){
	std::cout << "vector: ";
    for(int i = 0; i < v.size(); i ++){
        std::cout << v[i] << " ";
    }
	std::cout << "\n";
}


void log_vector(const std::vector<double> v){
	std::cout << "vector: ";
    for(int i = 0; i < v.size(); i ++){
        std::cout << v[i] << " ";
    }
	std::cout << "\n";
}


void log_trajectory(const std::vector<Eigen::VectorXd> traj){
    if(traj.empty()){
        std::cout << "traj is empty\n";
    }
    else{
        for (const auto& vec : traj) {
            log_vector(vec);
        }
    }
}


void log_matrix(const Eigen::MatrixXd& mat){
    for (int i = 0; i < mat.rows(); ++i) {
        for (int j = 0; j < mat.cols(); ++j) {
            std::cout << mat(i, j) << " ";
        }
        std::cout << std::endl;
    }
}


void write_traj_to_csv(const std::vector<Eigen::VectorXd>& data, const std::string filename){
    std::ofstream outputFile(filename);
    if (!outputFile.is_open()) {
        std::cerr << "Error opening the file for writing." << std::endl;
    }
    // Set a high precision for the output stream
    outputFile << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (const auto& vectorData : data) {
        for (int i = 0; i < vectorData.size(); ++i) {
            outputFile << vectorData(i);
            if (i < vectorData.size() - 1) {
                outputFile << ',';
            }
        }
        outputFile << std::endl;
    }
    outputFile.close();
    std::cout << "Save file to: " << filename << "\n";
}


void write_vector_to_csv(const std::vector<unsigned int>& data, const std::string& filename) {
    std::ofstream file(filename);
    // Check if the file is open
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    // Write data to the file
    for (size_t i = 0; i < data.size(); ++i) {
        file << data[i];
        // Add comma if it's not the last element
        if (i != data.size() - 1) {
            file << ",";
        }
    }
    // Close the file
    file.close();
}


int getRandomInt(int min, int max) {
    // Create a random number generator engine
    std::random_device rd;  // Random seed from hardware entropy
    std::mt19937 gen(rd());  // Mersenne Twister engine

    // Define a distribution for integer values between min and max (inclusive)
    std::uniform_int_distribution<int> dis(min, max);

    // Generate and return a random integer
    return dis(gen);
}


double getRandomDouble(double x_min, double x_max) {
    // Seed the random number generator with a random device
    std::random_device rd;
    std::mt19937 gen(rd());

    // Define the distribution for double values in the range [x_min, x_max]
    std::uniform_real_distribution<double> distribution(x_min, x_max);

    double value = distribution(gen);

    return value;
    // double multiplier = std::pow(10.0, 3);
    // return std::round(value * multiplier) / multiplier;
}


std::string doubleToString(double value) {
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << value;
    return stream.str();
}

// source: https://github.com/TakaHoribe/Riccati_Solver/blob/master/riccati_solver.h
bool solveRiccatiIterationD(const Eigen::MatrixXd &Ad,
                            const Eigen::MatrixXd &Bd, const Eigen::MatrixXd &Q,
                            const Eigen::MatrixXd &R, Eigen::MatrixXd &P,
                            const double &tolerance = 1.E-5,
                            const uint iter_max = 100000) {
  P = Q; // initialize

  Eigen::MatrixXd P_next;

  Eigen::MatrixXd AdT = Ad.transpose();
  Eigen::MatrixXd BdT = Bd.transpose();
  Eigen::MatrixXd Rinv = R.inverse();

  double diff;
  for (uint i = 0; i < iter_max; ++i) {
    // -- discrete solver --
    P_next = AdT * P * Ad -
             AdT * P * Bd * (R + BdT * P * Bd).inverse() * BdT * P * Ad + Q;

    diff = fabs((P_next - P).maxCoeff());
    P = P_next;
    if (diff < tolerance) {
      std::cout << "iteration mumber = " << i << std::endl;
      log_matrix(P);
      return true;
    }
  }
  return false; // over iteration limit
}


// source: https://scicomp.stackexchange.com/questions/30757/discrete-time-algebraic-riccati-equation-dare-solver-in-c
bool solveRiccatiIterationD2(const Eigen::MatrixXd &Ad,
                            const Eigen::MatrixXd &Bd, const Eigen::MatrixXd &Q,
                            const Eigen::MatrixXd &R, Eigen::MatrixXd &P,
                            const double &tolerance = 1.E-5,
                            const uint iter_max = 100000) {
  P = Q; // initialize

  Eigen::MatrixXd A = Ad;
  Eigen::MatrixXd G = Bd*(R.inverse())*Bd.transpose();
  Eigen::MatrixXd H = Q;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(Ad.rows(), Ad.rows());

  double diff;
  for (uint i = 0; i < iter_max; ++i) {
    // -- discrete solver --
    Eigen::MatrixXd A_new = A*((I+G*H).inverse())*A;
    Eigen::MatrixXd G_new = G + A*((I+G*H).inverse())*G*A.transpose();
    Eigen::MatrixXd H_new = H + A.transpose() * H * ((I+G*H).inverse()) * A;

    diff = (H_new - H).norm() / H_new.norm();
    if (diff < tolerance) {
      std::cout << "iteration mumber = " << i << std::endl;
      P = H_new;
      log_matrix(P);
      return true;
    }
    A = A_new; G = G_new; H = H_new;
  }
  return false; // over iteration limit
}


double angleDifference(const double& angleA, const double& angleB){
    // std::complex<double> a(cos(angleA), sin(angleA));
    // std::complex<double> b(cos(angleB), sin(angleB));
    // double angle_a = std::arg(a);
    // double angle_b = std::arg(b);
    // std::cout << "[DEBUG] " << angle_a << "\t" << angle_b << "\n";
    
    double diff = angleA - angleB;
    // Adjust difference to the range [-pi, pi]
    while (diff > M_PI)
        diff -= 2 * M_PI;
    while (diff <= -M_PI)
        diff += 2 * M_PI;
    return diff;
}


std::vector<Eigen::VectorXd> read_csv_data(const std::string filename){
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


}