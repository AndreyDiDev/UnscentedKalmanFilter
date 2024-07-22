// Integration of Everest with UKF

#ifndef UNIVERSAL_CPP
#define UNIVERSAL_CPP

#include "universal.hpp"

#define REFRESH_RATE 20

// Madgwick -> IMUs, Baros

// UKF -> GPS, Dynamic Model

// Madgwick -(Xi = Filtered Altitude)> z = GPS

void init(float X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in){
    // Input: Estimate Uncertainty -> system state
    // Initial Guess
}

// Update Step-------------------------------------
void update(){
    uniTransform();

    stateUpdate();
}

void uniTransform(){
    // measurement vector
    // Z = h (X) = altitude 

    //  N = number of dimensions

    // number of s points = 2N +1

    // first one is the mean 

    // all others = xn,n + sqrt((N+k)Pn,n) for 1 to N

    // change sign to negative when i = N+1, .... 2N

    //  propagate s points measurment to state equation

    // compute weigths oof s points

    // w0 = k/(N+k) for the first mean s point

    // wi = 1/2(N+k)

    // Zn = sigma(wi Zn)

    // approx mean and covar pf output distribution

    // mean = xhat n+1, n = sum 2N for wi Xn+1,n

    // covar P n+1, n = sum to 2N for wi (Xn+1,n - xhatn+1,n)(same transposed)

    // for gaussian distribution, set N + k = 3
}

void stateUpdate(){
    // Xn = Xn-1 + K (Zn - EstZn)
}
// ------------------------------------------------



// Prediction--------------------------------------
void prediction(){

    // initialize scenario to default model-A(vg)
    // given the Madgwick acc, velo, alt
    // interpolate between 2 models 

    // predict scenario t+1 based on interpolated values

}
// R = control noise 
// Q = measurement noise 

/**
 * Take the filtered values from Everest filter 
*/
void setStateVector(float filteredAcc, float filteredVelo, float filteredAlt){
    this.Uaccel = filteredAcc;
    this.Uvelo = filteredVelo;
    this.Ualt = filteredAlt;
}

int main(){
    
}

#endif


// have 10 sec zero offset procedure
// then wait for sigificant movement to start UKF
// then catch up to the logged but not submitted
// to UKF meassurment for the start delay