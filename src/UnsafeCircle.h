#pragma once
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <vector>
#include <map>

class UnsafeCircle{
public:
    // Parameters
    double dimension;
    std::vector<Eigen::VectorXd> center;
    std::vector<double> radius;
};