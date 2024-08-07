#include "universal.hpp"
#include "C:\\Users\\andin\\OneDrive\\Documents\\AllRepos\\UnscentedKalmanFilter\\eigen-3.4.0\\Eigen\\Dense"
// #include "everestTaskHPP.hpp"

#ifndef UNIVERSAL_CPP
#define UNIVERSAL_CPP

// Integration of Everest with  UKF

#define REFRESH_RATE 20

// Constants for the UKF
#define N 2
#define dim 2
#define alpha 0.1
#define beta 2
#define k 3 - dim // 3 dimensions

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

    // // F is for state to next state transition
    // // P0 = initial guess of state covariance matrix
    // // Q = process noise covariance matrix
    // // R = measurement noise covariance matrix

    // // Weights for sigma points
    float lambda = std::pow(alpha, 2) * (dim + k) - dim;
    lambda = -1.98;
    float w0_m = lambda / (2 + lambda);    // weight for first sPoint when cal covar                                     // weight for first sPoint when cal mean
    float w0_c = lambda/(2 + lambda) + (1 - std::pow(alpha, 2) + beta);
    float w_i = 1/ (2 * ( N + lambda));

    std::cout << "lambda: " << lambda << std::endl; 
    
    std::cout << "w0_m: " << w0_m << " w0_c " << w0_c << std::endl;
    
    std::cout << "w_i: " << w_i << std::endl;

    MatrixXf Weights(5, 5);
    VectorXf W(5, 1);
    // Weights.setZero((2 * 3) + 1);   // 2N + 1
    W.setConstant(2 * dim + 1, w_i);
    // Weights << w0_m, W;
    // Weights.diagonal(w_i);
    for(int i = 1; i < 5; i++){
        Weights.diagonal()[i] = w_i;
    }

    Weights(0) = w0_c;

    this->WeightsUKF = Weights;

    std::cout << "Weights: \n" << Weights << std::endl;

    // errors can be because you didnt instatiate the matrix
    // or trying to make a vector and declaring as a matrix
    VectorXf WeightsForSigmaPoints(5, 1);
    WeightsForSigmaPoints.setConstant(5, w_i);
    WeightsForSigmaPoints(0) = w0_m;
    this->WeightsForSigmaPoints = WeightsForSigmaPoints;

    std::cout << "WeightsForSigmaPoints: \n" << WeightsForSigmaPoints << std::endl;

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

// working
MatrixXf newDynamic(MatrixXf sigmaPoints){
    float time = 0.05;

    for(int i = 0; i < (2 * dim) + 1; i++){
        sigmaPoints(0, i) = sigmaPoints(0, i) + sigmaPoints(1, i) * time;
        sigmaPoints(1, i) = sigmaPoints(1, i) - ((9.8/0.5)*(sigmaPoints(0, i) * time));
    }

    std::cout << "New Sigma Points: \n" << sigmaPoints << std::endl;
    std::cout << "New Sigma Points: \n" << sigmaPoints(1,2) << std::endl;

    return sigmaPoints;

}

MatrixXf Universal::calculateSigmaPoints(MatrixXf &X0, MatrixXf &P0, MatrixXf &Q, MatrixXf &projectError, MatrixXf &Weights) {
    // MatrixXf sigmaPoints(2,2);
    // Calculate the square root of (N+k) * P0 using Cholesky decomposition
    float lambda = std::pow(alpha, 2) * (N + k) - N;
    std::cout << "lambda = " << lambda << std::endl;

    // MatrixXf sqrtP0 = (N + k) * P0;
    // MatrixXf sqrtP0(2,2);
    // sqrtP0 << 0, 0,
    //         0, 0;

    MatrixXf P(2,2);
    P << 5, 0, 
        0, 5;

    float mutliplier = 0.02; // N - lambda

    MatrixXf L( ((mutliplier) *P).llt().matrixL());
    std::cout << L.col(0) << std::endl;


    std::cout << "calc sPts" << std::endl;
    // std::cout << "SqrtP0: " << sqrtP0 << std::endl;

    // LDLT<MatrixXf> lltOfP0(P0); // Perform Cholesky decomposition
    // MatrixXf L = lltOfP0.matrixL(); // Retrieve the lower triangular matrix

    // // Initialize sigma points matrix
    MatrixXf sigmaPoints(dim, (2 * N) + 1);
    sigmaPoints.setZero();

    // // Set the first sigma point
    sigmaPoints.col(0) = X0;

    // Set the remaining sigma points
    for (int i = 1; i < dim + 1; i++) {
        sigmaPoints.col(i) = X0 + L.col(i - 1);
    }

    for(int j = dim + 1; j < (2 * N) + 1; j++){
        sigmaPoints.col(j) = X0 - L.col(j - dim - 1);
    }

    // propagate sigma points through the dynamic model
    sigmaPoints = newDynamic(sigmaPoints);

    // std::cout << "Sigma Points row: " << sigmaPoints.rows() << " col: " << sigmaPoints.cols() << std::endl;
    // std::cout << "Sigma Points row 0 \n" << sigmaPoints.row(0) << std::endl;
    // std::cout << "Sigma Points row 0\n" << sigmaPoints(0, all) << std::endl;

    // std::cout << "WeightsForSigmaPoints row: " << WeightsForSigmaPoints.rows() 
    // << " col: " << WeightsForSigmaPoints.cols() << std::endl;

    // calculate the mean and covariance of the sigma points
    VectorXf xPreMean(2,1);
    // Xprediction = sigmaPoints * WeightsForSigmaPoints;
    for(int row = 0; row < N; row++){
        float sum00 = 0;
        for(int col = 0; col < 5; col++){
            std::cout << "sP (" << row << ", " << col << ")" << "= " << sigmaPoints(row, col) << std::endl;
            sum00 += float(sigmaPoints(row, col)) * float(WeightsForSigmaPoints(col));
        }
        xPreMean(row) = sum00;
        std::cout << "XpreMean: \n" << xPreMean << std::endl;
    }
    // this->Xprediction(0, 0) = sum00;

    std::cout << "XpreMean: \n" << xPreMean << std::endl;
    // std::cout << "Xprediction: \n" << Xprediction << std::endl;

    MatrixXf projError(2, 5);
    
    std::cout << "Sigma Points row: " << sigmaPoints.rows() << " col: " << sigmaPoints.cols() << std::endl;

    for(int j = 0; j < dim; j++){
        projectError.row(j) = (sigmaPoints.row(j).array() - Xprediction.row(j).value()).matrix();
    }

    std::cout << "Project Error: \n" << projectError << std::endl;

    // assuming non linear dynamics
    MatrixXf Pprediction = projectError * Weights.asDiagonal() * projectError.transpose() + Q;

    return sigmaPoints;
}

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

int main(){
    // Initialize the state vector
    // setStateVector(Everest::filteredAcc, Everest::filteredVelo, Everest::filteredAlt);
    // setStateVector(0, 0, 1000);

    // // Initialize the covariance matrix
    // MatrixXf P0(3, 3);
    // P0 << 3, 0, 0,
    //       0, 3, 0,
    //       0, 0, 3;

    // // Initialize the UKF
    // init(P0, Z_in, R_in);

    // // Update the UKF
    // update();

    // // Predict the next values
    // prediction();

    MatrixXf X0(2, 1);
    X0 << 0.0873,
          0;

    MatrixXf X(10, 1);
    X << 0.199, 0.113, 0.12, 0.101, 
    0.099, 0.063, 0.008, -0.017, -0.037, -0.05;

    MatrixXf P0(2, 2);
    P0 << 5, 0,
          0, 5;

    VectorXf Z_in(2,1);
    Z_in << 0, 0;

    std::cout << "X0:\n" << X0 << std::endl;

    Universal uni = Universal();

    uni.init(X0, P0, Z_in);

    MatrixXf sigmaPoints = uni.calculateSigmaPoints(X0, P0, P0, P0, P0);

    std::cout << "Sigma Points:\n" << sigmaPoints << std::endl;

    return 0;

}

#endif


// have 10 sec zero offset procedure
// then wait for sigificant movement to start UKF
// then catch up to the logged but not submitted
// to UKF meassurment for the start delay