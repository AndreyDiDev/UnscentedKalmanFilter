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

// Function to interpolate between two nearest scenarios
float interpolateScenarios(VectorXf &X_in, std::vector<Scenario> &scenarios) {

    // Find the nearest 2 scenarios to the current state for each measure
    auto predicted_acc = findNearestScenarios(scenarios, X_in(0), X_in(1), 'a');
    auto predicted_velo = findNearestScenarios(scenarios, X_in(0), X_in(1), 'v');
    auto predicted_alt = findNearestScenarios(scenarios, X_in(0), X_in(1), 'h');

    // Interpolate between the two scenarios
    float interpolated_acc = interpolate(X_in(1), predicted_acc[0].first, predicted_acc[1].first);
    float interpolated_velo = interpolate(X_in(2), predicted_velo[0].first, predicted_velo[1].first);
    float interpolated_alt = interpolate(X_in(3), predicted_alt[0].first, predicted_alt[1].first);

    X_in << interpolated_acc, interpolated_velo, interpolated_alt;

    // Save the interpolated scenarios for the next iteration
    Scenario scenario1(predicted_acc[0].second, predicted_velo[0].second, predicted_alt[0].second);
    Scenario scenario2(predicted_acc[1].second, predicted_velo[1].second, predicted_alt[1].second);

    // Store the scenarios in a vector or any other suitable data structure
    std::vector<Scenario> nextScenarios;
    nextScenarios.push_back(scenario1);
    nextScenarios.push_back(scenario2);

    predictNextValues(time, nextScenarios);

    return X_in;

}


/**
 * Given a list of scenarios, find the nearest 2 scenarios to a target value
 */
vector<Scenario> findNearestScenarios(const std::vector<Scenario>& scenarios, float time, float targetValue, char measure) {
    std::vector<std::pair<float, Scenario>> distances;
    float value;

    switch (measure) {
        case 'a': // Acceleration
            for (const auto& scenario : scenarios) {
                float value = scenario.evaluateAcceleration(time, this.beforeApogee);
                float distance = std::abs(value - targetValue);
                distances.emplace_back(distance, scenario);
            }
            break;
        case 'v': // Velocity
            for (const auto& scenario : scenarios) {
                float value = scenario.evaluateVelocity(time, this.beforeApogee);
                float distance = std::abs(value - targetValue);
                distances.emplace_back(distance, scenario);
            }
            break;
        case 'h': // Altitude
            for (const auto& scenario : scenarios) {
                float value = scenario.evaluateAltitude(time, this.beforeApogee);
                float distance = std::abs(value - targetValue);
                distances.emplace_back(distance, scenario);
            }
            break;
        default:
            throw std::invalid_argument("Invalid measure type");
    }

    /**
     * Sorts a vector of distances based on the first element of each pair in ascending order.
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
 * Interpolates between two values based on a given x value
 */
float interpolate(float x, float scenario1Distance, float scenario2Distance) {
    double weight1 = 1.0 - (x - scenario1Distance) / (scenario2Distance - scenario1Distance);
    double weight2 = 1.0 - weight1;

    return weight1 * scenario1Distance + weight2 * scenario2Distance;
}

/**
 * Predicts the next values based on the interpolated scenarios
 */
void predictNextValues(float time, std::vector<Scenario> &scenarios, VectorX0 &X_in){
    // evaluate scenarios at time t+1
    float firstAccDist  =  std::abs(X_in(1) - scenarios[0].evaluateAcceleration(time + this.timeStep, this.beforeApogee));
    float firstVeloDist = std::abs(X_in(2) - scenarios[0].evaluateVelocity(time + this.timeStep,     this.beforeApogee));
    float firstAltDist  =  std::abs(X_in(3) - scenarios[0].evaluateAltitude(time + this.timeStep,     this.beforeApogee));

    float secondAccDist = std::abs(X_in(1) - scenarios[1].evaluateAcceleration(time + this.timeStep, this.beforeApogee));
    float secondVeloDist= std::abs(X_in(2) - scenarios[1].evaluateVelocity(time + this.timeStep,    this.beforeApogee));
    float secondAltDist = std::abs(X_in(3) - scenarios[1].evaluateAltitude(time + this.timeStep,    this.beforeApogee));

    // interpolate between the two scenarios to get predicted values
    float predicted_interpolated_acc = interpolate(X_in(1), firstAccDist, secondAccDist);
    float predicted_interpolated_velo = interpolate(X_in(2), firstVeloDist, secondVeloDist);
    float predicted_interpolated_alt = interpolate(X_in(3), firstAltDist, secondAltDist);

    this.X_pred << predicted_interpolated_acc, predicted_interpolated_velo, predicted_interpolated_alt;
 
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

    VectorXf X_in(3);

    /** X_in = [acceleration, velocity, altitude] */ 
    X_in << this.Uaccel, this.Uvelo, this.Ualt;
}

int main(){
    
}

#endif


// have 10 sec zero offset procedure
// then wait for sigificant movement to start UKF
// then catch up to the logged but not submitted
// to UKF meassurment for the start delay