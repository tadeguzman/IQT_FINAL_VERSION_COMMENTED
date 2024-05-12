#include "Arduino.h"
#include "IQT_ANGLE_EKF.h"

// Theo DeGuzman
// Johns Hopkins University
// IQT24 Senior Design Project
// V2 Prototype Code
// Not for production use -- end user assumes responsibility for usage and testing of this code


/* This is a 1-D Kalman-like filter for one angle represneting the boat orientation

    This is technically not a kalman Filter because it uses measurements in both the
    state update and measurement step, but follows all the fundamental principles of the KF.

    The state update step takes the angular rate in the direction of consideration and
    adds its value, multiplied by the step time, to get a new orientation estimate.
    The acceleration term is used to determine orientation based on the direction of the 
    gravity vector.

    The gyroscope prevents translational accelerations from being interpreted as orientation
    changes over short time intervals. The acceleration limits gyroscopic drift away from the
    true value.

    This system calcualtes ptich, then roll. Yaw is ignored by IMU and handled by the GPS.
*/

ANGLE_EKF::ANGLE_EKF(){
    this->prior_est.Fill(0);
    this->post_est.Fill(0);
    this->prior_cov.Fill(0);
    this->post_cov.Fill(0);
    this->kalman_gain.Fill(0);
}

void ANGLE_EKF::begin(BLA::Matrix<2, 2> q, BLA::Matrix<2, 2> r) {
    this->Q = q.Submatrix<2, 2>(0,0);
    this->R = r.Submatrix<2, 2>(0,0);
    
}
BLA::Matrix<2, 1> ANGLE_EKF::step(BLA::Matrix<2, 1> measurement, double time_step) {
    BLA::Matrix<2, 2> A = {1, time_step, 0, 1};
    BLA::Matrix<2, 2> I = {1, 0, 0, 1}; // note that H = I and therefore will be removed to reduce calcualtions

    this->prior_est = A*this->post_est;
    this->prior_cov = A*this->post_cov*(~A) + Q;
    // this->kalman_gain = this->prior_cov*(~H)*Inverse(H*this->prior_cov*(~H)+R);
    this->kalman_gain = this->prior_cov*Inverse(this->prior_cov+R);
    // this->post_est = this->prior_est + this->kalman_gain*(measurement - H*this->prior_est);
    this->post_est = this->prior_est + this->kalman_gain*(measurement - this->prior_est);
    // this->post_cov = (I - this->kalman_gain*H)*this->prior_cov*(~(I - this->kalman_gain*H)) + this->kalman_gain*R*(~this->kalman_gain);
    this->post_cov = (I - this->kalman_gain)*this->prior_cov*(~(I - this->kalman_gain)) + this->kalman_gain*R*(~this->kalman_gain);

    return this->post_est;
}



BLA::Matrix<2, 1> ANGLE_EKF::get_prior_est(){
    return this->prior_est;
}
BLA::Matrix<2, 1> ANGLE_EKF::get_post_est() {
    return this->post_est;
}
BLA::Matrix<2, 2> ANGLE_EKF::get_prior_cov(){
    return this->prior_cov;
}
BLA::Matrix<2, 2> ANGLE_EKF::get_post_cov(){
    return this->post_cov;
}
BLA::Matrix<2, 2> ANGLE_EKF::get_kalman_gain(){
    return this->kalman_gain;
}
void ANGLE_EKF::debug(){
    Serial << "[" << this->prior_est << ", " << this->post_est << ", " << this->prior_cov << ", " << this->post_cov << ", " << this->kalman_gain << "]\n";
}



double get_pitch(float x, float y, float z) {
    return 180 * atan2(x, (z/abs(z))*sqrt(y*y + z*z))/PI;
}
double get_roll(float x, float y, float z) {
    return 180 * atan2(y, z)/PI; // technically not correct but close enough approximation
}


