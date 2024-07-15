// Integration of Everest with UKF
#include "universal.hpp"

// Madgwick -> IMUs, Baros

// UKF -> GPS, Dynamic Model

// Madgwick -(Xi = Filtered Altitude)> z = GPS

void init(VectorXf &X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in){
    // Input: Estimate Uncertainty -> system state
    // Initial Guess
    R = &

}

// Update Step-------------------------------------
void update(){
    uniTransform();

    stateUpdate();
}

void uniTransform(){
    // measurement space
    // Z = h (X) 

    // Zn = sigma(wi Zn)
}

void stateUpdate(){
    // Xn = Xn-1 + K (Zn - EstZn)
}
// ------------------------------------------------



// Prediction--------------------------------------
void prediction(){

}
// R = control noise 
// Q = measurement noise 

int main(){
    
}
