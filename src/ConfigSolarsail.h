#pragma once

#include <iostream>
#include <vector>

namespace CONFIG_SOLARSAIL{

    std::vector<double> get_parameters(const int env){
        std::vector<double> params;
        if(env == 1){
            params = {2.0, 0.0, 0.0, 4.2118e-11, 6.51688e+06, 108.4094};
        }
        if(env == 2){
            params = {2.0, 0.0, 0.0, 4.2118e-11, 6.51688e+06, 108.4094};
        }
        if(env == 3){
            params = {2.0, 0.0, 0.0, 3.446e-11, 6.51688e+06, 108.4094};
        }
        if(env == 4){
            params = {2.0, 0.0, 0.0, 3.1244e-11, 6.0048e+06, 52.4430};
        }
        return params;
    }

    std::vector<double> get_state_init(const int env){
        std::vector<double> state_init;
        if(env == 1){
            state_init = {0.127680370, 0, 0.084952359, 0, 1.445775202, 0};
        }
        if(env == 2){
            state_init = {-0.1062, 0, 0.1106, 0, 0.8425, 0};
        }
        if(env == 3){
            state_init = {-0.0359, 0.0, 0.1202, 0.0, 1.9244, 0.0};
        }
        if(env == 4){
            state_init = {0.0019, 0.0488, 0.0, 0.0, 0.0, 4.7163};
        }
        return state_init;
    }

}