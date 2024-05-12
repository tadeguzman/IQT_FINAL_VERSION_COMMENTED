#ifndef IQT_ANGLE_EKF_H
#define IQT_ANGLE_EKF_H

#include "Arduino.h"
#include "BasicLinearAlgebra.h"

using namespace BLA;

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

class ANGLE_EKF 
{
    public:
        ANGLE_EKF();

        // start the Kalman filter, with Q and R initalized
        void begin(BLA::Matrix<2, 2> Q, BLA::Matrix<2, 2> R);

        // update the EKF state
        // function inputs from the GPS  unit directly
        //    - measurement: values (angle measurement from accelerometer, angular rate from gyroscope)
        //    - time step: time elapsed since last KF update (deg)
        BLA::Matrix<2, 1> step(BLA::Matrix<2, 1> measurement, double time_step);
        
        // Return the prior est
        BLA::Matrix<2, 1> get_prior_est();

        // Return the posterior est
        BLA::Matrix<2, 1> get_post_est();

        // Return the prior covariance
        BLA::Matrix<2, 2> get_prior_cov();

        // Return the posterior covariance
        BLA::Matrix<2, 2> get_post_cov();

        // Return the kalman gain
        BLA::Matrix<2, 2> get_kalman_gain();

        // print debug messages
        void debug();


    private:
        // internally used variables
        BLA::Matrix<2, 1> prior_est; // prior estimate 
        BLA::Matrix<2, 1> post_est; // posterior estimate
        BLA::Matrix<2, 2> prior_cov; // prior covariance
        BLA::Matrix<2, 2> post_cov; // posterior covariance
        BLA::Matrix<2, 2> kalman_gain; // kalman gain
        BLA::Matrix<2, 2> Q;  // process covariance
        BLA::Matrix<2, 2> R;  // measurement update matrix
};




// Globally accessible methods //


/* Returns the pitch value (in the IMU frame)
   Angles defined as first roll, then pitch

    Inputs:
    x: acceleration in x
    y: acceleration in y
    z: acceleration in z
*/
double get_pitch(float x, float y, float z);


/* Returns the roll value (in the IMU frame)
   Angles defined as first roll, then pitch

    Inputs:
    x: acceleration in x
    y: acceleration in y
    z: acceleration in z
*/
double get_roll(float x, float y, float z);

#endif