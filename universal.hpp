#ifndef UNIVERSAL_HPP
#define UNIVERSAL_HPP


#include "eigen-3.4.0\\Eigen\\Cholesky"
#include "cmath"


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

    protected:

}

#endif