#ifndef UNIVERSAL_HPP
#define UNIVERSAL_HPP

#include "C:\\Users\\andin\\Downloads\\eigen-3.4.0\\Eigen\\Cholesky"
#include "C:\\Users\\andin\\Downloads\\eigen-3.4.0\\Eigen\\Dense"
#include <cmath>
#include <string>
#include <vector>
#include <iostream>

#include "Kalman.hpp"


bool isBeforeApogeeBool = false;

using namespace Eigen;

/**
 * Keeps track of the kinematics of Universal. Updated
 * with UKF state update
 */
typedef struct{
    float initialVelo;
    float initialAlt;
    float finalAltitude;
} kinematics;

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


class Universal : public Kalman{
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
    
        // Aiden
        /* Initalize the Kalman filter
        
        F_in -> State transition matrix (dim_x X dim_x) (For linear system dynamics only)
        H_in -> Observation matrix (dim_z X dim_x) (For linear observation equations only)
        Qa_in -> Process noise covariance initializer, see below (dim_x X dim_x) (For linear system dynamics only)
        Z_in -> Measurement vector. Update passed vector with new values each time step. (dim_z)
        R_in -> Measurement Covariance matrix. Update passed matrix with new values each time step. (dim_z X dim_z)
        X0 -> inital state vector. (dim_x)
        P0 -> inital estimate covariance (dim_x X dim_x)

        Note: Qa should be initialized as s^2[M], where M has a 1 on the main diagonal corresponding to each
        * state variable associated with s^2. For instance, state vector [x,y] where only y is affected by variance has Qa =
        * [0,  0]
        * [0,s^2]
        */
        virtual void init(VectorXf &X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in);

        virtual void init(VectorXf &X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in, MatrixXf &H_in);

        virtual void init(VectorXf &X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in, MatrixXf &F_in, MatrixXf &Qa_in);

        virtual void init(VectorXf &X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in, MatrixXf &H_in, MatrixXf &F_in, MatrixXf &Qa_in);

        /*Update the filter with new measurements
        Note that this reads the variables Z and R passed to init() for new data.*/
        virtual void update();

        /*
        * Constructor
        * dim_x: number of state variables (size of state vector X)
        * dim_z: number of measurement inputs (size of measurement vector Z)
        */
        Universal(int dim_x, int dim_z);
    
    private:
        float Uaccel;
        float Ualt;
        float Uvelo;

        VectorXf X_in;
        VectorXf X_pred;

        float timeStep = 0.1;

        std::vector<Scenario> scenarios;

        std::vector<float> scenarioWeights = {0.5, 0.5};

    protected:

        MatrixXf sigmaPoints;
        MatrixXf Xprediction;
        MatrixXf Pprediction;
        MatrixXf P;
        MatrixXf Q;
        MatrixXf projectError;
        MatrixXf X0;
        MatrixXf WeightsUKF;

        MatrixXf F; // state to next state transition matrix
        MatrixXf H; // state to measurement matrix
        MatrixXf R; // measurement noise covariance matrix
        MatrixXf K; // Kalman gain matrix

        MatrixXf X; // state vector
        MatrixXf Z;

        kinematics Kinematics;

        kinematics* getKinematics();

        /*Takes state sigma points and transforms them to the measurement space (Ex: returning a Measurement sigma point matrix). Must be overwritten with 
        non-linear observation equations. Uses H matrix passed to init() function by default (useful if observation equations are linear).*/
        MatrixXf observe(MatrixXf sigmaX);

        /*Takes state vector X and calculates state based off of previous state, to be stored in Xpred.  Must be overwritten with
        non-linear system dynamics. Uses F matrix passed to init() by default (useful if system dynamics are linear) 
        X will be a dim_x X 1 matrix, output must also be a dim_x X 1 matrix.*/
        MatrixXf predict(MatrixXf sigmaX);

        /*Function to perform the unscented transform*/
        void uTransform();

        float k; // Tuning parameter for linearization.
        MatrixXf sPoints; //to be assigned an array of sigma points
        MatrixXf sigma; //Decomposed matrix used for finding sigma points
        VectorXf W; //weight matrix / vector for sigma points
        VectorXf W1; //Matrix for filling W
        float w0, w1; //weights for W matrix
        MatrixXf projError; //Matrix for calculating projected covariance
        MatrixXf sMeasurements; //Matrix of measurement predictions from Unscented transform
        VectorXf Zpred; // expected measurement vector - needs to be explicitly defined for UKF
        MatrixXf WeightedZvariance; //weighted measurement variance matrix
        MatrixXf XZCovariance; // state-measuremnt cross covariance matrix
        MatrixXf measurementProjError; //Matrix for calculating weighted measurement variance matrix


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

kinematics* Universal::getKinematics(){
    return &Kinematics;
}

#endif