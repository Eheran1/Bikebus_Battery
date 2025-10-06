#ifndef Kalman_h
#define Kalman_h

#include <BasicLinearAlgebra.h>
//#include <stdexcept>
// Using the namespace for BasicLinearAlgebra to simplify matrix operations
using namespace BLA;

class KalmanFilter {
public:
  // State vector [position, velocity]
  BLA::Matrix<2, 1> x;
  // Covariance matrix for state
  BLA::Matrix<2, 2> P;
  // Process noise matrix
  BLA::Matrix<2, 2> Q;
  // Measurement noise scalar
  float R;
  // Kalman gain vector
  BLA::Matrix<2, 1> K;

  // Default constructor
  KalmanFilter() {}

  // Parameterized constructor for initialization
  KalmanFilter(float x_init, float p11, float p22, float q, float r) {
    init(x_init, p11, p22, q, r);
  }

  // Update function for the Kalman filter with a new measurement
  void update(float measurement, float dt) {
    // Prediction Step

    // State transition matrix for constant velocity model
    BLA::Matrix<2, 2> A = { 1, dt, 0, 1 };
    x = A * x;

    // Define process noise due to acceleration
    Q(0, 1) = Q(1, 0) = 0.5 * dt * dt;
    Q(1, 1) = dt;

    // Predicted covariance
    P = A * P * ~A + Q;

    // Measurement Update (Correction) Step

    // Innovation: difference between measurement and predicted state
    float y = measurement - x(0);
    // Innovation covariance
    float S = P(0, 0) + R;

    // Compute Kalman gain
    K(0) = P(0, 0) / S;
    K(1) = P(1, 0) / S;

    // Update state and covariance matrix using the Kalman gain
    x += K * y;


    BLA::Matrix<2, 2> I = { 1, 0, 0, 1 };  // Identity matrix
    BLA::Matrix<1, 2> H = { 1, 0 };        // Measurement matrix
    P = (I - K * H) * P;                   // Covariance update
  }

  // Initialization function
  void init(float x_init, float p11, float p22, float q, float r) {
    // Initial state
    x(0) = x_init;
    x(1) = 0;

    // Initial covariance
    P(0, 0) = p11;
    P(1, 1) = p22;
    P(0, 1) = P(1, 0) = 0;

    // Process noise
    Q(0, 0) = Q(1, 1) = q;
    Q(0, 1) = Q(1, 0) = 0;

    // Measurement noise
    R = r;
  }

  // Getter function to retrieve a specific state value
  float getState(int index) {
    if (index == 0 || index == 1) {
      return x(index);
    }
    else {
      Serial.println("Index out of bounds in getState");
      delay(5000);
      return 0.0;
    }
    // Error handling for index out of bounds
    //Serial.println("Index out of bounds in getState");
    //throw std::out_of_range("Index out of bounds in getState");
  }

  // Getter function to retrieve a specific element from the covariance matrix
  float getCovarianceElement(int i, int j) {
    if (i >= 0 && i < 2 && j >= 0 && j < 2) {
      return P(i, j);
    } else {
      // Error handling for invalid indices
      Serial.println("Indices out of bounds in getCovarianceElement");
      delay(5000);
      return 0.0;
      //throw std::out_of_range("Indices out of bounds in getCovarianceElement");
    }
  }
};

#endif  // Kalman_h
