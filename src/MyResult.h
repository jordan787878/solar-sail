#pragma once

#include <iostream>
#include <iomanip>
#include <limits>
#include <Eigen/Dense>

struct PlannerResult{
    double computationTime = 0;
    int isSuccess = 0; // 0: false, 1: true
    double trajLength = 0;   
    double timeOfFlight = 0;
    int treeSize = 0;
};

double calPosDistance(const Eigen::VectorXd x1, const Eigen::VectorXd x2){
    Eigen::VectorXd dx(3);
    dx << x1[0]-x2[0], x1[1]-x2[1], x1[2]-x2[2];
    return dx.norm();
}

double cal_trajLength(const std::vector<Eigen::VectorXd> trajectory){
    double Length = 0;

    for(int i=0; i<trajectory.size()-1; i++){
        Eigen::VectorXd x0 = trajectory[i];
        Eigen::VectorXd x1 = trajectory[i+1];
        Length = Length + calPosDistance(x0, x1);
    }
    
    return Length;
}


void write_result_csv(const std::vector<PlannerResult>& data, const std::string filename){
    std::ofstream outputFile(filename);
    if (!outputFile.is_open()) {
        std::cerr << "Error opening the file for writing." << std::endl;
    }
    // Set a high precision for the output stream
    // outputFile << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (const auto& vectorData : data) {
        outputFile << vectorData.computationTime << "," 
                   << vectorData.isSuccess << "," 
                   << vectorData.trajLength << ","
                   << vectorData.timeOfFlight << ","
                   << vectorData.treeSize << "\n";
    }
    outputFile.close();
    std::cout << "Save file to: " << filename << "\n";
}
