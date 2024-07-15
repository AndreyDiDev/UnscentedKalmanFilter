#include "eigen-3.4.0\\eigen-3.4.0\\Eigen\\Cholesky"


class Universal{
    public:
        void init(VectorXf &X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in);

        void update();

        void uniTransform();

        void stateUpdate();

        void prediction();

    private:

    protected:

}