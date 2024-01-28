#pragma once

#include <iostream>
#include <Eigen/Dense>

namespace KEEPOUT{
    struct Keepout
    {
        // define in units of [meter]
        Eigen::VectorXd center;
        double  radius;
    };

}

