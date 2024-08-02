#include "universal.hpp"
#include "C:\\Users\\andin\\Downloads\\eigen-3.4.0\\Eigen\\Dense"
// #include "everestTaskHPP.hpp"

#ifndef UNIVERSAL_CPP
#define UNIVERSAL_CPP

// Integration of Everest with  UKF

#define REFRESH_RATE 20

// Constants for the UKF
// #define N 2
// #define dim 2
// #define alpha 0.1
// #define beta 2
// #define k 3 - dim // 3 dimensions

using namespace Eigen;

// Madgwick -> IMUs, Baros

// UKF -> GPS, Dynamic Model

// Madgwick -(Xi = Filtered Altitude)> z = GPS

void Universal::init(MatrixXf &X0, MatrixXf &P0, VectorXf &Z_in){
    // Input: Estimate Uncertainty -> system state
    // Initial Guess
    // this->X0 = X0;
    // this->P = P0;
    // this->Z = Z_in;

    // // // F is for state to next state transition
    // // // P0 = initial guess of state covariance matrix
    // // // Q = process noise covariance matrix
    // // // R = measurement noise covariance matrix

    // // Weights for sigma points
    // float lambda = std::pow(alpha, 2) * (dim + k) - dim;
    // float w0_m = lambda / (dim + k);                                         // weight for first sPoint when cal mean
    // float w0_c = lambda/(3 + lambda) + (1 - std::pow(alpha, 2) + beta);    // weight for first sPoint when cal covar

    // MatrixXf Weights(2 * dim + 1);
    // Weights.setZero((2 * 3) + 1);   // 2N + 1
    // Weights.setConstant(2 * dim + 1, w0_c);
    // Weights(0) = w0_m;

    // this->WeightsUKF = Weights;

    // std::cout << "Weights: " << Weights << std::endl;

    // X0 = [acceleration, velocity, altitude]
    // X0 << this->getFAccel(), this->getFVelo(), this->getFAlt();

    // Z_in = [GPS altitude]
    // Z_in << this->getGPSAlt();

    // unscentedTransform();
}

// Update Step-------------------------------------
void Universal::update(){
    unscentedTransform();

    stateUpdate();
}

void Universal::unscentedTransform(){
    // measurement vector
    // Z = h (X) = altitude

    // N = number of dimensions
    // number of s points = 2N +1
    // all others = xn,n + sqrt((N+k)Pn,n) for 1 to N
    // change sign to negative when i = N+1, .... 2N

    //  propagate s points measurment to state equation

    // compute weights of s points

    // w0 = k/(N+k) for the first mean s point

    // wi = 1/2(N+k)

    // Zn = sigma(wi Zn)

    // approx mean and covar pf output distribution

    // mean = xhat n+1, n = sum 2N for wi Xn+1,n

    // covar P n+1, n = sum to 2N for wi (Xn+1,n - xhatn+1,n)(same transposed)

    // for gaussian distribution, set N + k = 3
}

void Universal::stateUpdate(){
    // Xn = Xn-1 + K (Zn - EstZn)
}
// ------------------------------------------------



// Prediction--------------------------------------
void Universal::prediction(){

    // initialize scenario to default model-A(vg)

    // given the Madgwick acc, velo, alt

    // MatrixXf sigmaPoints(2*N+1, 3);
    // sigmaPoints.setZero(2*N+1, 3);

    // calculate sigma points
    // sigmaPoints = calculateSigmaPoints();

    // interpolate between 2 models


    // predict scenario t+1 based on interpolated values

}

// MatrixXf calculateSigmaPoints(MatrixXf &X0, MatrixXf &P0, MatrixXf &Q, MatrixXf &projectError, MatrixXf &Weights) {
//     // MatrixXf sigmaPoints(2,2);
//     // Calculate the square root of (N+k) * P0 using Cholesky decomposition
//     // float lambda = std::pow(alpha, 2) * (N + k) - N;
//     // std::cout << "lambda = " << lambda << std::endl;

//     // MatrixXf sqrtP0 = (N + k) * P0;
//     // MatrixXf sqrtP0(2,2);
//     // sqrtP0 << 0, 0,
//     //         0, 0;

//     MatrixXf P(2,2);
//     P << 5, 0, 
//         0, 5;

//     MatrixXf L( ((dim + k) *P).llt().matrixL());
//     std::cout << L.col(0) << std::endl;


//     std::cout << "calc sPts" << std::endl;
//     // std::cout << "SqrtP0: " << sqrtP0 << std::endl;

//     // LDLT<MatrixXf> lltOfP0(P0); // Perform Cholesky decomposition
//     // MatrixXf L = lltOfP0.matrixL(); // Retrieve the lower triangular matrix

//     // // Initialize sigma points matrix
//     MatrixXf sigmaPoints(dim, (2 * N) + 1);
//     sigmaPoints.setZero();

//     // // Set the first sigma point
//     sigmaPoints.col(0) = X0;

//     // Set the remaining sigma points
//     for (int i = 1; i < dim + 1; i++) {
//         // sigmaPoints.row(i + 1) = X0 + L.col(i);
//         // sigmaPoints.row(i + 1 + N) = X0 - L.col(i);
//     }

//     for(int j = dim + 1; j < (2 * N) + 1; j++){
//         // sigmaPoints.col(j) = X0 + L.col(j - dim - 1);
//     }

//     // propagate sigma points through the dynamic model
//     for(int i = 0; i < (2 * N) + 1; i++){
//         // sigmaPoints.col(i) = dynamicModel(sigmaPoints.col(i));
//     }

//     // calculate the mean and covariance of the sigma points
//     MatrixXf Xprediction;
//     // Xprediction = sigmaPoints * Weights;

//     // MatrixXf projectError;
//     // projectError.setZero(dim, (2 * N) + 1);
//     for(int j = 0; j < dim; j++){
//         // projectError.row(j) = (sigmaPoints.row(j).array() - Xprediction.row(j).value()).matrix();
//     }

//     // assuming non linear dynamics
//     // MatrixXf Pprediction = projectError * Weights.asDiagonal() * projectError.transpose() + Q;

//     return sigmaPoints;
// }

// Function to interpolate between two nearest scenarios
float Universal::interpolateScenarios(VectorXf &X_in, std::vector<Scenario> &scenarios) {

    // Find the nearest 2 scenarios to the current state for each measure
    auto predicted_acc = findNearestScenarios(scenarios, X_in(0), X_in(1), 'a');
    auto predicted_velo = findNearestScenarios(scenarios, X_in(0), X_in(1), 'v');
    auto predicted_alt = findNearestScenarios(scenarios, X_in(0), X_in(1), 'h');

    // Interpolate between the two scenarios
    // float interpolated_acc = interpolate(X_in(1), predicted_acc[0].first, predicted_acc[1].first);
    // float interpolated_velo = interpolate(X_in(2), predicted_velo[0].first, predicted_velo[1].first);
    // float interpolated_alt = interpolate(X_in(3), predicted_alt[0].first, predicted_alt[1].first);

    // X_in << interpolated_acc, interpolated_velo, interpolated_alt;

    // // Save the interpolated scenarios for the next iteration
    // Scenario scenario1(predicted_acc[0].second, predicted_velo[0].second, predicted_alt[0].second);
    // Scenario scenario2(predicted_acc[1].second, predicted_velo[1].second, predicted_alt[1].second);

    // Store the scenarios in a vector or any other suitable data structure
    std::vector<Scenario> nextScenarios;
    // nextScenarios.push_back(scenario1);
    // nextScenarios.push_back(scenario2);

    // predictNextValues(time, nextScenarios);

    // return X_in;
    return 0;

}


/**
 * @brief Given a list of scenarios, find the nearest 2 scenarios to a target value
 */
std::vector<std::pair<float, Scenario>> Universal::findNearestScenarios(const std::vector<Scenario>& scenarios, float time, float targetValue, char measure) {
    std::vector<std::pair<float, Scenario>> distances;
    float value;

    switch (measure) {
        case 'a': // Acceleration
            for (const auto& scenario : scenarios) {
                // float value = scenario.evaluateAcceleration(time, this.isBeforeApogee);
                float value = 0;
                float distance = std::abs(value - targetValue);
                distances.emplace_back(distance, scenario);
            }
            break;
        case 'v': // Velocity
            for (const auto& scenario : scenarios) {
                // float value = scenario.evaluateVelocity(time, this.isBeforeApogee);
                float value = 0;
                float distance = std::abs(value - targetValue);
                distances.emplace_back(distance, scenario);
            }
            break;
        case 'h': // Altitude
            for (const auto& scenario : scenarios) {
                // float value = scenario.evaluateAltitude(time, this.isBeforeApogee);
                float value  =0;
                float distance = std::abs(value - targetValue);
                distances.emplace_back(distance, scenario);
            }
            break;
        default:
            throw std::invalid_argument("Invalid measure type");
    }

    /**
     * @brief Sorts a vector of distances based on the first element of each pair in ascending order.
     */
    std::sort(distances.begin(), distances.end(), [](const auto& a, const auto& b) {
        return a.first < b.first;
    });

    std::vector<std::pair<float, Scenario>> nearestScenarios;
    nearestScenarios.push_back(distances[0]);
    nearestScenarios.push_back(distances[1]);

    return nearestScenarios;
}

/**
 * @brief Interpolates between two values based on a given x value
 */
float Universal::interpolate(float x, float scenario1Distance, float scenario2Distance) {
    // Get gains for scenarios
    std::vector<float> gains = getGains(x, scenario1Distance, scenario2Distance);

    double gain1 = gains[0];
    double gain2 = 1.0 - gain1;

    return gain1 * scenario1Distance + gain2 * scenario2Distance;
}

/**
 * @brief Get gains for scenarios
 */
std::vector<float> Universal::getGains(float x, float scenario1Distance, float scenario2Distance) {
    float gain1 = 1.0 / std::abs(x - scenario1Distance);
    float gain2 = 1.0 - gain1;

    this->scenarioWeights = {gain1, gain2};

    return {gain1, gain2};
}

/**
 * @brief Interpolates between two scenarios based on the gains
 */
float interpolateWithgains(float gain1, float gain2, float scenario1Distance, float scenario2Distance) {
    return gain1 * scenario1Distance + gain2 * scenario2Distance;
}

/**
 * @brief Predicts the next values based on the interpolated scenarios
 */
void Universal::predictNextValues(float time, std::vector<Scenario> &scenarios, VectorXf &X_in){
    // evaluate scenarios at time t+1
    float firstAccDist  =  std::abs(X_in(1) - scenarios[0].evaluateAcceleration(time + this->timeStep));
    float firstVeloDist = std::abs(X_in(2) - scenarios[0].evaluateVelocity(time + this->timeStep));
    float firstAltDist  =  std::abs(X_in(3) - scenarios[0].evaluateAltitude(time + this->timeStep));

    float secondAccDist = std::abs(X_in(1) - scenarios[1].evaluateAcceleration(time + this->timeStep));
    float secondVeloDist= std::abs(X_in(2) - scenarios[1].evaluateVelocity(time + this->timeStep));
    float secondAltDist = std::abs(X_in(3) - scenarios[1].evaluateAltitude(time + this->timeStep));

    // interpolate between the two scenarios to get predicted values
    float predicted_interpolated_acc = interpolate(X_in(1), firstAccDist, secondAccDist);
    float predicted_interpolated_velo = interpolate(X_in(2), firstVeloDist, secondVeloDist);
    float predicted_interpolated_alt = interpolate(X_in(3), firstAltDist, secondAltDist);

    // this->X_pred << predicted_interpolated_acc, predicted_interpolated_velo, predicted_interpolated_alt;
}

/**
 * @brief Check if the rocket is before apogee, based on Everest filter values
 */
bool isBeforeApogee(float acceleration, float velocity, float altitude, float lastAltitude){

    if(acceleration < 1 || velocity < 1 || altitude < lastAltitude){
        return false;
    }

    return true;
}

// R = control noise
// Q = measurement noise

/**
 * @brief Take the filtered values from Everest filter
*/
void Universal::setStateVector(float filteredAcc, float filteredVelo, float filteredAlt){
    this->Uaccel = filteredAcc;
    this->Uvelo = filteredVelo;
    this->Ualt = filteredAlt;

    // VectorXf X_in(3);

    /** X_in = [acceleration, velocity, altitude] */
    this->X << this->Uaccel, this->Uvelo, this->Ualt;
}

// prediction step based on the dynamic model
MatrixXf dynamicModel(MatrixXf &X){
    // X = [acceleration, velocity, altitude]
    MatrixXf Xprediction(3, 1);



    return Xprediction;
}

// Aiden
Universal::Universal(int dim_x, int dim_z)
{
    this->dim_x = dim_x;
    this->dim_z = dim_z;
    k = 3 - dim_x;
}

void Universal::init(VectorXf &X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in)
{
    R = &R_in;
    Z = &Z_in;

    linearDynamics = false;

    // Calculate weights
    w0 = k / (dim_x + k);
    w1 = 1 / (2 * (dim_x + k));
    W.setZero((2 * dim_x) + 1);
    W1.setConstant(2 * dim_x, w1);
    W << w0, W1;

    // initalize and predict next state
    X = X0;
    P = P0;
    uTransform();
}

void Universal::init(VectorXf &X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in, MatrixXf &H_in)
{
    H = &H_in;
    R = &R_in;
    Z = &Z_in;

    linearDynamics = false;

    // Calculate weights
    k = 3 - dim_x;
    w0 = k / (dim_x + k);
    w1 = 1 / (2 * (dim_x + k));
    W.setZero((2 * dim_x) + 1);
    W1.setConstant(2 * dim_x, w1);
    W << w0, W1;

    // initalize and predict next state
    X = X0;
    P = P0;
    uTransform();
}

void Universal::init(VectorXf &X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in, MatrixXf &F_in, MatrixXf &Qa_in)
{
    F = &F_in;
    R = &R_in;
    Z = &Z_in;

    linearDynamics = true;

    Q = *F * Qa_in * F->transpose();

    // Calculate weights
    k = 3 - dim_x;
    w0 = k / (dim_x + k);
    w1 = 1 / (2 * (dim_x + k));
    W.setZero((2 * dim_x) + 1);
    W1.setConstant(2 * dim_x, w1);
    W << w0, W1;

    // initalize and predict next state
    X = X0;
    P = P0;
    uTransform();
}

void Universal::init(VectorXf &X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in, MatrixXf &H_in, MatrixXf &F_in, MatrixXf &Qa_in)
{
    H = &H_in;
    F = &F_in;
    R = &R_in;
    Z = &Z_in;

    linearDynamics = true;

    Q = *F * Qa_in * F->transpose();

    // Calculate weights
    k = 3 - dim_x;
    w0 = k / (dim_x + k);
    w1 = 1 / (2 * (dim_x + k));
    W.setZero((2 * dim_x) + 1);
    W1.setConstant(2 * dim_x, w1);
    W << w0, W1;

    // initalize and predict next state
    X = X0;
    P = P0;
    uTransform();
}

void Universal::update()
{
    sMeasurements.setZero(dim_z, (2 * dim_x) + 1);
    for (int i = 0; i < (2 * dim_x) + 1; i++)
    {
        sMeasurements.col(i) = observe(sPoints.col(i));
    }
    Zpred = sMeasurements * W;
    
    measurementProjError.setZero(dim_z, (2 * dim_x) + 1);
    for (int i = 0; i < dim_z; i++)
    {
        measurementProjError.row(i) = (sMeasurements.row(i).array() - Zpred.row(i).value()).matrix();
    }

    WeightedZvariance = (measurementProjError * W.asDiagonal() * measurementProjError.transpose()) + *R; // Compute measurement variance matrix
    XZCovariance = projError * W.asDiagonal() * measurementProjError.transpose();                        // Compute state-measurement cross covariance matrix
    K = XZCovariance * WeightedZvariance.inverse();                                                      // Compute Kalman gain
    X = Xpred + K * (*Z - Zpred);                                                                        // update state prediction
    P = Ppred - (K * WeightedZvariance * K.transpose());                                                 // update Covariance
    uTransform();                                                                                        // make next predictions
}

MatrixXf Universal::observe(MatrixXf sigmaX)
{
    // return *H * sigmaX; //linear observation equation default.
    VectorXf Zout(2);
    Zout << sqrt(pow(sigmaX(0), 2) + pow(sigmaX(3), 2)),
            atan(sigmaX(3)/sigmaX(0));
    return Zout;
}

MatrixXf Universal::predict(MatrixXf sigmaX)
{
    return *F * sigmaX; // linear system dynamics default.
}

void Universal::uTransform()
{
    // Generate sigma points
    sPoints.setZero(dim_x, (2 * dim_x) + 1);
    sPoints.col(0) = X;
    sigma = ((dim_x + k) * P).llt().matrixL();
    for (int i = 1; i < dim_x + 1; i++)
    {
        sPoints.col(i) = X + sigma.col(i - 1);
    }
    for (int i = dim_x + 1; i < (2 * dim_x) + 1; i++)
    {
        sPoints.col(i) = X - sigma.col(i - dim_x - 1);
    }

    // Propigate points
    for (int i = 0; i < (2 * dim_x) + 1; i++)
    {
        sPoints.col(i) = predict(sPoints.col(i));
    }

    // Calculate weighted mean for points
    Xpred = sPoints * W;
    projError.setZero(dim_x, (2 * dim_x) + 1);
    for (int i = 0; i < dim_x; i++)
    {
        projError.row(i) = (sPoints.row(i).array() - Xpred.row(i).value()).matrix();
    }
    if (linearDynamics == true)
    {
        Ppred = projError * W.asDiagonal() * projError.transpose() + Q;
    }
    else
    {
        Ppred = projError * W.asDiagonal() * projError.transpose();
    }
}


int main(){
    VectorXf X0u(6);
    X0u << 400, 0, 0, -300, 0, 0;

    MatrixXf P0u(6, 6);
    P0u << 500, 0, 0, 0, 0, 0,
        0, 500, 0, 0, 0, 0,
        0, 0, 500, 0, 0, 0,
        0, 0, 0, 500, 0, 0,
        0, 0, 0, 0, 500, 0,
        0, 0, 0, 0, 0, 500;

    VectorXf Zu(2);

    MatrixXf Ru(2, 2);
    Ru << 25, 0,
        0, 0.00007569;

    MatrixXf Fu(6, 6);
    Fu << 1, 1, 0.5, 0, 0, 0,
        0, 1, 1, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 1, 0.5,
        0, 0, 0, 0, 1, 1,
        0, 0, 0, 0, 0, 1;

    MatrixXf Qau(6, 6);
    Qau << 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0.04, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0.04;

    double posu[2][35] = {{502.55, 477.34, 457.21, 442.94, 427.27, 406.05, 400.73, 377.32, 360.27, 345.93, 333.34, 328.07, 315.48,
                          301.41, 302.87, 304.25, 294.46, 294.29, 299.38, 299.37, 300.68, 304.1, 301.96, 300.3, 301.9, 296.7, 297.07,
                          295.29, 296.31, 300.62, 292.3, 298.11, 298.07, 298.92, 298.04},
                         {-0.9316, -0.8977, -0.8512, -0.8114, -0.7853, -0.7392, -0.7052, -0.6478, -0.59, -0.5183, -0.4698, -0.3952, -0.3026,
                          -0.2445, -0.1626, -0.0937, 0.0085, 0.0856, 0.1675, 0.2467, 0.329, 0.4149, 0.504, 0.5934, 0.667, 0.7537, 0.8354,
                          0.9195, 1.0039, 1.0923, 1.1546, 1.2564, 1.3274, 1.409, 1.5011}};

    Universal car(6, 2);
    car.init(X0u, P0u, Zu, Ru, Fu, Qau);
    for (int l = 0; l < 35; l++)
    {
        Zu << posu[0][k], posu[1][k];
        car.update();
    }
    std::cout << car.getState() << std::endl;
    std::cout << car.getCov() <<   std::endl;


    return 0;

}

#endif


// have 10 sec zero offset procedure
// then wait for sigificant movement to start UKF
// then catch up to the logged but not submitted
// to UKF meassurment for the start delay