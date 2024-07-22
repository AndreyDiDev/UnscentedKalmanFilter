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

    // Find the nearest 2 scenarios to the current state
    auto predicted_acc = findNearestScenarios(scenarios, X_in(0), X_in(1), 'a');
    auto predicted_velo = findNearestScenarios(scenarios, X_in(0), X_in(1), 'v');
    auto predicted_alt = findNearestScenarios(scenarios, X_in(0), X_in(1), 'h');

    // Interpolate between the two scenarios
    float interpolated_acc = interpolate(X_in(1), predicted_acc.acceleration, predicted_velo.acceleration);

    // Simple average for interpolation
    return (y1 + y2) / 2;
}


/**
 * Given a list of scenarios, find the nearest 2 scenarios to a target value
 */
vector<Scenario> findNearestScenarios(const std::vector<Scenario>& scenarios, float x, float targetValue, char measure) {
    std::vector<std::pair<float, Scenario>> distances;

    for (const auto& scenario : scenarios) {
        float value;
        switch (measure) {
            case 'a': // Acceleration
                value = scenario.evaluateAcceleration(x);
                break;
            case 'v': // Velocity
                value = scenario.evaluateVelocity(x);
                break;
            case 'h': // Altitude
                value = scenario.evaluateAltitude(x);
                break;
            default:
                throw std::invalid_argument("Invalid measure type");
        }

        float distance = std::abs(value - targetValue);
        distances.emplace_back(distance, scenario);
    }

    std::sort(distances.begin(), distances.end(), [](const auto& a, const auto& b) {
        return a.first < b.first;
    });

    return distances.front().second;
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
    X_in << this.Uaccel, this.Uvelo, this.Ualt;
}

int main(){
    
}

#endif


// have 10 sec zero offset procedure
// then wait for sigificant movement to start UKF
// then catch up to the logged but not submitted
// to UKF meassurment for the start delay