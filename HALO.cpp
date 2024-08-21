#include "HALO.hpp"
#include "C:\Users\andin\OneDrive\Documents\AllRepos\UnscentedKalmanFilter\eigen-3.4.0\Eigen\Dense"
// #include "everestTaskHPP.hpp"
#include <fstream>

#ifndef HALO_CPP
#define HALO_CPP

// Integration of Everest with UKF = HALO

#define REFRESH_RATE 20

// Constants for the UKF
#define N 6
#define dim 6
#define alpha 0.1
#define beta 2
#define k 3 - dim // 3 dimensions

using namespace Eigen;

// Madgwick -> IMUs, Baros

// UKF -> GPS, Dynamic Model = augmented state vector

// Madgwick -(Xi = Filtered Altitude)> z = GPS

void HALO::init(MatrixXf &X0, MatrixXf &P0, MatrixXf Q_input, VectorXf &Z_input, MatrixXf &F){
    // Input: Estimate Uncertainty -> system state
    // Initial Guess
    this->X0 = X0;
    this->P = P0;
    this->Z = Z_input;
    this->Q = Q_input;
    this->F = F;

    // F is for state to next state transition
    // P0 = initial guess of state covariance matrix
    // Q = process noise covariance matrix -> dynamic model std
    // R = measurement noise covariance matrix -> sensor std

    std::cout << "k: " << k << std::endl;

    std::cout << "N: " << N << std::endl;

    // Weights for sigma points
    // float lambda = std::pow(alpha, 2) * (dim + k) - dim;
    // lambda = -1.98;
    float dime = 6;
    float k1 = 3 - dime;
    float w0_m = k1 / (dime + k1);    // weight for first sPoint when cal covar                                     // weight for first sPoint when cal mean
    // float w0_c = lambda/(2 + lambda) + (1 - std::pow(alpha, 2) + beta);
    float w_i = 1/ (2 * ( dime + k1));

    // std::cout << "lambda: " << lambda << std::endl; 
    
    std::cout << "w0_m: " << w0_m << std::endl;
    
    std::cout << "w_i: " << w_i << std::endl;

    MatrixXf Weights(13, 13);
    VectorXf W(13, 1);

    W.setConstant(2 * dim + 1, w_i);

    for(int i = 1; i < 13; i++){
        Weights.diagonal()[i] = w_i;
    }

    Weights(0) = w0_m;

    this->WeightsUKF = Weights;

    // std::cout << "Weights: \n" << Weights << std::endl;

    // errors can be because you didnt instatiate the matrix
    // or trying to make a vector and declaring as a matrix
    VectorXf WeightsForSigmaPoints(13, 1);
    WeightsForSigmaPoints.setConstant(13, w_i);
    WeightsForSigmaPoints(0) = w0_m;
    this->WeightsForSigmaPoints = WeightsForSigmaPoints;

    std::cout << "WeightsForSigmaPoints: \n" << WeightsForSigmaPoints << std::endl;

    // X0 = [acceleration, velocity, altitude]
    // X0 << this->getFAccel(), this->getFVelo(), this->getFAlt();

    // Z_in = [GPS altitude]
    // Z_in << this->getGPSAlt();
    this->N1 = 6;

    calculateSigmaPoints();
}

// Update Step-------------------------------------
void HALO::update(){
    unscentedTransform();

    // stateUpdate();
}

void HALO::unscentedTransform(){
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

void HALO::stateUpdate(){
    // Xn = Xn-1 + K (Zn - EstZn)
    // std::cout << "sigmaPoints state update: \n" << sigPoints << std::endl;

    MatrixXf observedValues(2, 13);
    observedValues.setZero(2, 13);

    for (int i = 0; i < (2 * this->N1) + 1; i++){
        observedValues.col(i) = observe(sigPoints.col(i));
    }

    // std::cout << "Observed Values: \n" << observedValues << std::endl;

    // calculate the mean of the observed values
    VectorXf zMean(2);
    zMean = observedValues * WeightsForSigmaPoints;

    // std::cout << "Z_1: " << zMean << std::endl;
    this->Z = zMean;

    MatrixXf R(2,2);
    R <<std::pow(5, 2), 0,
        0, std::pow(0.0087, 2);

    // calculate covariance of Z
    MatrixXf zCovar(2, 13);
    zCovar.setZero(2, (2 * 6) + 1);
    for (int i = 0; i < 2; i++)
    {
        zCovar.row(i) = (observedValues.row(i).array() - zMean.row(i).value()).matrix();
    }
 
    // std::cout << "Z Covar: " << zCovar << std::endl;

    // calculate the innovation covariance
    MatrixXf Pz(2, 2);
    Pz.setZero(2, 2);

    Pz = (zCovar * WeightsForSigmaPoints.asDiagonal() * zCovar.transpose()) + R;

    // std::cout << "Pz: " << Pz << std::endl;

    // calculate the cross covariance
    MatrixXf Pxz(6,2);
    Pxz.setZero();

    // std::cout << "projectError: " << projectError << std::endl;

    Pxz = projectError * WeightsForSigmaPoints.asDiagonal() * zCovar.transpose(); 

    // std::cout << "Pxz: " << Pxz << std::endl;

    // calculate the Kalman gain
    MatrixXf K(6, 2);
    K.setZero();

    K = Pxz * Pz.inverse();

    // std::cout << "X: \n" << this->X << std::endl;
    // std::cout << "Kalman Gain: \n" << K << std::endl;

    // update the state vector
    X0 = this->Xprediction + K * (this->X - zMean); 

    std::cout << "X: \n" << X0 << std::endl;

    // update the covariance matrix
    MatrixXf P1(6,6);

    // // std::cout << "Pprediction: " << Pprediction << std::endl;
    // // std::cout << "K: " << K << std::endl;
    // // std::cout << "Pz: " << Pz << std::endl;

    P1 = Pprediction - (K * Pz * K.transpose());

    this->P = P1;

    std::cout << "P(1,1): " << P << std::endl;

    std::cout << "\n end of state Update\n " << std::endl;

    calculateSigmaPoints();

}
// ------------------------------------------------



// Prediction--------------------------------------
void HALO::prediction(){

    // initialize scenario to default model-A(vg)

    // given the Madgwick acc, velo, alt

    // MatrixXf sigmaPoints(2*N+1, 3);
    // sigmaPoints.setZero(2*N+1, 3);

    // calculate sigma points
    // sigmaPoints = calculateSigmaPoints();

    // interpolate between 2 models


    // predict scenario t+1 based on interpolated values

}

MatrixXf HALO::observe(MatrixXf sigmaPoints){
    VectorXf Z_out(2,1);

    Z_out << sqrt(pow(sigmaPoints(0), 2) + pow(sigmaPoints(3), 2)),
            atan(sigmaPoints(3)/sigmaPoints(0));

    return Z_out;

}

MatrixXf HALO::predict(MatrixXf sigmaPoints){
    return this->F * sigmaPoints;
}

void HALO::calculateSigmaPoints() {

    // std::cout << "X0: " << X0 << std::endl;

    // std::cout << "Q: " << Q << std::endl;

    float mutliplier = this->N1 - 3; // N - lambda

    // std::cout << "Multiplier: " << mutliplier << std::endl;

    MatrixXf L( ((mutliplier) *P).llt().matrixL());
    // std::cout << L.col(0) << std::endl;

    // std::cout << "N " << this->N1 << std::endl;

    // std::cout << "L: \n" << L << std::endl;

    // Initialize sigma points matrix
    MatrixXf sigmaPoints(6, 13);
    sigmaPoints.setZero();

    // Set the first sigma point
    sigmaPoints.col(0) = X0;

    // Set the remaining sigma points
    for (int i = 1; i < this->N1 + 1; i++) {
        sigmaPoints.col(i) = X0 + L.col(i - 1);
    }

    for(int j = this->N1 + 1; j < (2 * this->N1) + 1; j++){
        sigmaPoints.col(j) = X0 - L.col(j - this->N1 - 1);
    }

    // before dynamics
    // std::cout << "before dynamics sPoints: \n" << sigmaPoints << std::endl;

    // propagate sigma points through the dynamic model
    // sigmaPoints = predict(sigmaPoints);
    for (int i = 0; i < (2 * this->N1) + 1; i++){
        sigmaPoints.col(i) = predict(sigmaPoints.col(i));
    }

    // std::cout << "after predict sPoints: \n" << sigmaPoints << std::endl;

    // std::cout << "Sigma Points row: " << sigmaPoints.rows() << " col: " << sigmaPoints.cols() << std::endl;
    // std::cout << "Sigma Points row 0 \n" << sigmaPoints.row(0) << std::endl;
    // std::cout << "Sigma Points row 0\n" << sigmaPoints(all, all) << std::endl;

    // std::cout << "WeightsForSigmaPoints row: " << WeightsForSigmaPoints.rows() 
    // << " col: " << WeightsForSigmaPoints.cols() << std::endl;

    // calculate the mean and covariance of the sigma points
    VectorXf xPreMean(6,1);
    for(int row = 0; row < this->N1; row++){
        float sum00 = 0;
        for(int col = 0; col < 2*this->N1 + 1; col++){
            // std::cout << "sP (" << row << ", " << col << ")" << "= " << sigmaPoints(row, col) << std::endl;
            sum00 += sigmaPoints(row, col) * WeightsForSigmaPoints(col);
        }
        xPreMean(row) = sum00;
        // std::cout << "XpreMean: \n" << xPreMean << std::endl;
    }

    // std::cout << "XpreMean: \n" << xPreMean << std::endl;
    // std::cout << "Xprediction: \n" << Xprediction << std::endl;
    this->Xprediction = xPreMean;

    MatrixXf projError(6, 13);
    projError.setZero(6, 13);
    
    // std::cout << "Sigma Points row: " << sigmaPoints.rows() << " col: " << sigmaPoints.cols() << std::endl;
    // std::cout << "xPreMean row: " << xPreMean.rows() << " col: " << xPreMean.cols() << std::endl;
    // std::cout << "sigmaPoints (0,3) " << projError(0,3) << std::endl;

    for (int i = 0; i < this->N1; i++)
    {
        projError.row(i) = (sigmaPoints.row(i).array() - (this->Xprediction).row(i).value()).matrix();
    }

    // std::cout << "Project Error: \n" << projError << std::endl;

    this->projectError = projError;

    MatrixXf Pprediction(6,6);

    Pprediction = projError * WeightsForSigmaPoints.asDiagonal() * projError.transpose() + this->Q;

    std::cout << "\nPprediction: \n" << Pprediction << std::endl;

    this->Pprediction = Pprediction;

    this->sigPoints =  sigmaPoints;
}

// Function to interpolate between two nearest scenarios
float HALO::interpolateScenarios(VectorXf &X_in, std::vector<Scenario> &scenarios) {

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
std::vector<std::pair<float, Scenario>> HALO::findNearestScenarios(const std::vector<Scenario>& scenarios, float time, float targetValue, char measure) {
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
float HALO::interpolate(float x, float scenario1Distance, float scenario2Distance) {
    // Get gains for scenarios
    std::vector<float> gains = getGains(x, scenario1Distance, scenario2Distance);

    double gain1 = gains[0];
    double gain2 = 1.0 - gain1;

    return gain1 * scenario1Distance + gain2 * scenario2Distance;
}

/**
 * @brief Get gains for scenarios
 */
std::vector<float> HALO::getGains(float x, float scenario1Distance, float scenario2Distance) {
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
void HALO::predictNextValues(float time, std::vector<Scenario> &scenarios, VectorXf &X_in){
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

/**
 * @brief Take the filtered values from Everest filter
*/
void HALO::setStateVector(float filteredAcc, float filteredVelo, float filteredAlt){
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

    // only able to measure angle and extrapolate for velocity
    MatrixXf X0(6, 1);
    X0 << 400, 0, 0, -300, 0, 0;

    int deltaT = 1;


    MatrixXf F(6 , 6);
    F << 1, deltaT, 0.5 * std::pow(deltaT,2), 0, 0, 0,
         0, 1, deltaT, 0, 0, 0,
         0, 0, 1, 0, 0, 0,
         0, 0, 0, 1, deltaT, 0.5 * std::pow(deltaT, 2),
         0, 0, 0, 0, 1, deltaT,
         0, 0, 0, 0, 0, 1;

    MatrixXf X(2, 35);
    X << 502.55, 477.34, 457.21, 442.94, 427.27, 406.05, 400.73, 377.32, 360.27, 345.93, 333.34, 328.07, 315.48,
                          301.41, 302.87, 304.25, 294.46, 294.29, 299.38, 299.37, 300.68, 304.1, 301.96, 300.3, 301.9, 296.7, 297.07,
                          295.29, 296.31, 300.62, 292.3, 298.11, 298.07, 298.92, 298.04,
                          
        -0.9316, -0.8977, -0.8512, -0.8114, -0.7853, -0.7392, -0.7052, -0.6478, -0.59, -0.5183, -0.4698, -0.3952, -0.3026,
                          -0.2445, -0.1626, -0.0937, 0.0085, 0.0856, 0.1675, 0.2467, 0.329, 0.4149, 0.504, 0.5934, 0.667, 0.7537, 0.8354,
                          0.9195, 1.0039, 1.0923, 1.1546, 1.2564, 1.3274, 1.409, 1.5011;

    MatrixXf P(6, 6);
    P <<500, 0, 0, 0, 0, 0,
        0, 500, 0, 0, 0, 0,
        0, 0, 500, 0, 0, 0,
        0, 0, 0, 500, 0, 0,
        0, 0, 0, 0, 500, 0,
        0, 0, 0, 0, 0, 500;


    VectorXf Z_in(2,1);
    Z_in << 0, 0;

    MatrixXf Q(6,6);
    Q << 0.25, 0.5, 0.5, 0, 0, 0,
         0.5, 1, 1, 0, 0, 0,
         0.5, 1, 1, 0, 0, 0,
         0, 0, 0, 0.25, 0.5, 0.5,
         0, 0, 0, 0.5, 1, 1,
         0, 0, 0, 0.5, 1, 1;

    Q = Q * std::pow(0.2, 2);

    std::cout << "Q:\n" << Q << std::endl;

    std::cout << "X:\n" << X << std::endl;

    // Open a file for writing
    std::ofstream outFile("filtered_values_2D_HALO.csv");

    // Check if the file is open
    if (!outFile.is_open()) {
        std::cerr << "Failed to open filtered_values_2D.csv" << std::endl;
        return 1;
    }

    // Write the header to the file
    outFile << "Time,X,Y,XVelo,YVelo\n";

    HALO halo = HALO();

    halo.init(X0, P, Q, Z_in, F);

    VectorXf X1(2,1);
    halo.X = X1;

    for(int i = 0; i < 35; i++){
        std::cout << "\n\nIteration: " << i + 1<< std::endl;

        halo.X << X(0, i), X(1, i);

        halo.stateUpdate();

        // Write the filtered values to the file
        outFile << i << "," << halo.X0(0) << "," << halo.X0(3) << "," << halo.X0(1) << "," << halo.X0(4) << "\n";
    }

    // Close the file
    outFile.close();

    return 0;

}

#endif