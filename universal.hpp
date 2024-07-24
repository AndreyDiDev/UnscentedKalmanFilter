#ifndef UNIVERSAL_HPP
#define UNIVERSAL_HPP

#include "C:\\Users\\andin\\OneDrive\\Documents\\AllRepos\\UnscentedKalmanFilter\\eigen-3.4.0\\Eigen\\Cholesky"
#include <cmath>
#include <string>
#include <vector>
#include <iostream>

/**
 * @brief Scenario struct to store the coefficients of the 3rd degree polynomial for acceleration, velocity and altitude
 *      before and after apogee, also evaluates the acceleration, velocity and altitude at a given time
 */
struct Scenario {
    std::vector<float> beforeApogeeAccel;
    std::vector<float> afterApogeeAccel;

    std::vector<float> beforeApogeeVelo;
    std::vector<float> afterApogeeVelo;

    std::vector<float> beforeApogeeAlt;
    std::vector<float> afterApogeeAlt;

    std::string name;

    Scenario(std::vector<float> beforeApogeeCoefficientsAccel, std::vector<float> afterApogeeCoefficientsAccel,
        std::vector<float> beforeApogeeCoefficientsVelo, std::vector<float> afterApogeeCoefficientsVelo,
        std::vector<float> beforeApogeeCoefficientsAlt, std::vector<float> afterApogeeCoefficientsAlt, std::string Name)

        : beforeApogeeAccel(beforeApogeeCoefficientsAccel), afterApogee(afterApogeeCoefficientsAccel), 
        beforeApogeeVelo(beforeApogeeCoefficientsVelo), afterApogeeVelo(afterApogeeCoefficientsVelo), 
        beforeApogeeAlt(beforeApogeeCoefficientsAlt), afterApogeeAlt(afterApogeeCoefficientsAlt), name(Name) {};

    // state machine on apogee
    float evaluateAcceleration(float time, bool isbeforeApogee) {
        if(isBeforeApogee){
            // evaluate acceleration at timestep before apogee by using the coefficients at 3rd degree polynomial
            return beforeApogeeAccel[0] * pow(time, 3) + beforeApogeeAccel[1] * pow(time, 2) 
            + beforeApogeeAccel[2] * time + beforeApogeeAccel[3];
        }
        else{
            // evaluate acceleration at timestep after apogee by using the coefficients at 3rd degree polynomial
            return afterApogeeAccel[0] * pow(time, 3) + afterApogeeAccel[1] * pow(time, 2) 
            + afterApogeeAccel[2] * time + afterApogeeAccel[3];
        } 
    }

    float evaluateVelocity(float time){
        if(isBeforeApogee){
            // evaluate velocity at timestep before apogee by using the coefficients at 3rd degree polynomial
            return beforeApogeeVelo[0] * pow(time, 3) + beforeApogeeVelo[1] * pow(time, 2)
            + beforeApogeeVelo[2] * time + beforeApogeeVelo[3];
        }
        else{
            // evaluate velocity at timestep after apogee by using the coefficients at 3rd degree polynomial
            return afterApogeeVelo[0] * pow(time, 3) + afterApogeeVelo[1] * pow(time, 2) 
            + afterApogeeVelo[2] * time + afterApogeeVelo[3];
    }

    float evaluateAltitude(float time){
        if(isBeforeApogee){
            // evaluate altitude at timestep before apogee by using the coefficients at 3rd degree polynomial
            return beforeApogeeAlt[0] * pow(time, 3) + beforeApogeeAlt[1] * pow(time, 2) + beforeApogeeAlt[2] * time + beforeApogeeAlt[3];
        }
        else{
            // evaluate altitude at timestep after apogee by using the coefficients at 3rd degree polynomial
            return afterApogeeAlt[0] * pow(time, 3) + afterApogeeAlt[1] * pow(time, 2) + afterApogeeAlt[2] * time + afterApogeeAlt[3];
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