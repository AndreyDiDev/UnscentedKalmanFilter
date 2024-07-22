#ifndef UNIVERSAL_HPP
#define UNIVERSAL_HPP


#include "eigen-3.4.0\\Eigen\\Cholesky"
#include "cmath"

struct Scenario {
    float acceleration;
    float velocity;
    float altitude;


    Scenario(float acc, float vel, float alt)
        : acceleration(acc), velocity(vel), altitude(alt) {}

    // state machine on apogee
    float evaluateAcceleration(float time, bool beforeApogee) {
        if(beforeApogee){
            return time;
        }
        else{
            return time * (-1);;
        } 
    }

    float evaluateVelocity(float time){
        if(beforeApogee){
            return time;
        }
        else{
            return time * (-1);;
    }

    float evaluateAltitude(float time){
        if(beforeApogee){
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

    protected:

}

#endif