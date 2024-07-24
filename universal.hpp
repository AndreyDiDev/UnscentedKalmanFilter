#ifndef UNIVERSAL_HPP
#define UNIVERSAL_HPP

#include "C:\\Users\\andin\\OneDrive\\Documents\\AllRepos\\UnscentedKalmanFilter\\eigen-3.4.0\\Eigen\\Cholesky"
#include <cmath>
#include <string>
#include <vector>
#include <iostream>

struct Scenario {
    std::vector<float> beforeApogeeAccel;
    std::vector<float> afterApogeeAccel;

    
    std::string name;


    Scenario(std::vector<float> beforeApogeeCoefficientsAccel, std::vector<float> afterApogeeCoefficientsAccel, std::string Name)
        : beforeApogeeAccel(beforeApogeeCoefficientsAccel), afterApogee(afterApogeeCoefficientsAccel), name(Name) {}

    // state machine on apogee
    float evaluateAcceleration(float time, bool isbeforeApogee) {
        if(isBeforeApogee){
            // evaluate acceleration at timestep before apogee by using the coefficients at 3rd degree polynomial
            return beforeApogee[0] * pow(time, 3) + beforeApogee[1] * pow(time, 2) + beforeApogee[2] * time + beforeApogee[3];
        }
        else{
            // evaluate acceleration at timestep after apogee by using the coefficients at 3rd degree polynomial
            return afterApogee[0] * pow(time, 3) + afterApogee[1] * pow(time, 2) + afterApogee[2] * time + afterApogee[3];
        } 
    }

    float evaluateVelocity(float time){
        if(isBeforeApogee){
            // evaluate velocity at timestep before apogee by using the coefficients at 3rd degree polynomial

        }
        else{
            return time * (-1);;
    }

    float evaluateAltitude(float time){
        if(isBeforeApogee){
            return time;
        }
        else{
            return time * (-1);;
        }
    }
    
    }
};


class Universal{
    public:
        void init(VectorXf &X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in);

        void update();

        void uniTransform();

        void stateUpdate();

        void prediction();

    private:
        float Uaccel;
        float Ualt;
        float Uvelo;

        VectorXf X_in;
        VectorXf X_pred;

        float timeStep = 0.1;

        bool isBeforeApogee = false;

        std::vector<Scenario> scenarios;

        std::vector<float> weights = {0.5, 0.5};

    protected:

}

#endif