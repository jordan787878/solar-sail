// src/myownlibrary.cpp
#include "myownlibrary.h"
#include <Eigen/Dense>
// #include <yaml-cpp/yaml.h>

Eigen::MatrixXd addMatrices(const Eigen::MatrixXd& matrix1, const Eigen::MatrixXd& matrix2) {
    return matrix1 + matrix2;
}

