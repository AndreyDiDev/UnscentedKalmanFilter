#ifndef UNIVERSAL_HPP
#define UNIVERSAL_HPP

#include "C:\\Users\\Andrey\\Downloads\\eigen-3.4.0\\eigen-3.4.0\\Eigen\\Cholesky"
#include "C:\\Users\\Andrey\\Downloads\\eigen-3.4.0\\eigen-3.4.0\\Eigen\\Dense"
#include <cmath>
#include <string>
#include <vector>
#include <iostream>


bool isBeforeApogeeBool = false;

using namespace Eigen;

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

        : beforeApogeeAccel(beforeApogeeCoefficientsAccel), afterApogeeAccel(afterApogeeCoefficientsAccel), 
        beforeApogeeVelo(beforeApogeeCoefficientsVelo), afterApogeeVelo(afterApogeeCoefficientsVelo), 
        beforeApogeeAlt(beforeApogeeCoefficientsAlt), afterApogeeAlt(afterApogeeCoefficientsAlt), name(Name) {};

    // state machine on apogee
    float evaluateAcceleration(float time) {
        if(isBeforeApogeeBool){
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
        if(isBeforeApogeeBool){
            // evaluate velocity at timestep before apogee by using the coefficients at 3rd degree polynomial
            return beforeApogeeVelo[0] * pow(time, 3) + beforeApogeeVelo[1] * pow(time, 2)
            + beforeApogeeVelo[2] * time + beforeApogeeVelo[3];
        }
        else{
            // evaluate velocity at timestep after apogee by using the coefficients at 3rd degree polynomial
            return afterApogeeVelo[0] * pow(time, 3) + afterApogeeVelo[1] * pow(time, 2) 
            + afterApogeeVelo[2] * time + afterApogeeVelo[3];
        }
    }

    float evaluateAltitude(float time){
        if(isBeforeApogeeBool){
            // evaluate altitude at timestep before apogee by using the coefficients at 3rd degree polynomial
            return beforeApogeeAlt[0] * pow(time, 3) + beforeApogeeAlt[1] * pow(time, 2) + beforeApogeeAlt[2] * time + beforeApogeeAlt[3];
        }
        else{
            // evaluate altitude at timestep after apogee by using the coefficients at 3rd degree polynomial
            return afterApogeeAlt[0] * pow(time, 3) + afterApogeeAlt[1] * pow(time, 2) + afterApogeeAlt[2] * time + afterApogeeAlt[3];
        }
    }
    
};


class Universal{
    public:
        void init(MatrixXf &X0, MatrixXf &P0, VectorXf &Z_in);

        void update();

        void unscentedTransform();

        void stateUpdate();

        void prediction();

        float fAccel, fVelo, fAlt, GPS_Alt;

        void setFilteredValues(float FAccel, float fVelo, float fAlt);

        float getFAlt();

        float getFVelo();

        float getFAccel();

        float getGPSAlt();

        void setAlt(float gps_alt);

        std::vector<float> getGains(float x, float scenario1Distance, float scenario2Distance);

        void predictNextValues(float time, std::vector<Scenario> &scenarios, VectorXf &X_in);

        void setStateVector(float filteredAcc, float filteredVelo, float filteredAlt);

        float interpolate(float x, float scenario1Distance, float scenario2Distance);

        std::vector<std::pair<float, Scenario>> findNearestScenarios(const std::vector<Scenario>& scenarios, float time, float targetValue, char measure);

        float interpolateScenarios(VectorXf &X_in, std::vector<Scenario> &scenarios);
    private:
        float Uaccel;
        float Ualt;
        float Uvelo;

        VectorXf X_in;
        VectorXf X_pred;

        float timeStep = 0.1;

        std::vector<Scenario> scenarios;

        std::vector<float> weights = {0.5, 0.5};

    protected:

};

bool getIsBeforeApogee(){
    return isBeforeApogeeBool;
};

void Universal::setFilteredValues(float FAccel, float FVelo, float FAlt){
    this->fAccel = FAccel;
    this->fVelo = FVelo;
    this->fAlt = FAlt;
}

float Universal::getFAlt(){
    return this->fAlt;
}

float Universal::getFVelo(){
    return this->fVelo;
}

float Universal::getFAccel(){
    return this->fAccel;
}

void Universal::setAlt(float gps_alt){
    this->GPS_Alt = gps_alt;
}

float Universal::getGPSAlt(){
    return this->GPS_Alt;
}

#endif