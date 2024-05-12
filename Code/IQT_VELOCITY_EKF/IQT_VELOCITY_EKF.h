#ifndef IQT_VELOCITY_EKF_H
#define IQT_VELOCITY_EKF_H

#include "Arduino.h"
#include "BasicLinearAlgebra.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS


using namespace BLA;
#define HISTORY 10

// Theo DeGuzman
// Johns Hopkins University
// IQT24 Senior Design Project
// V2 Prototype Code
// Not for production use -- end user assumes responsibility for usage and testing of this code


/* Initally the "IQT_GPS_EKF" class (deleted), this class is essentially a 1-D noise filter on speed and heading, from GPS data.
    In it's entirety, it is probably not necessary for this to exist, but it's well-tested and integrated into the IQT24 V2 code architecture.
*/


class VELOCITY_EKF 
{
    public:
        VELOCITY_EKF();
        void begin();

        // update the EKF state
        // function inputs from the GPS  unit directly
        //    - speed (m/s)
        //    - heading (deg)
        //    - latitude (deg)
        //    - longitude (deg)
        //    - speed accuracy (m/s)
        //    - heading accuracy (deg)
        BLA::Matrix<2, 1, double> step(double speed, double heading, double lat, double lon, double speed_acc, double heading_acc);
        
        // Retrieve the current process model based on current EKF state
        //   This version just assumes heading and speed are constant.
        void step_process_model();
        

        // this method just records the values passed to "step" into particular variables
        void take_measurement(double speed, double heading, double lat, double lon);
        
        // Retrieve the current process covariance based on current EKF state
        //    This verison has a constant Q
        void update_Q();
        
        // Retrieve the current measurement covariance based on estimated heading accuracies
        //    This verison DOES update R based on the GPS estimated accuracy
        void update_R(double speed_acc, double heading_acc);
        
        // Return the prior est
        BLA::Matrix<2, 1, double> get_prior_est();
        
        // Return the posterior est
        BLA::Matrix<2, 1, double> get_post_est();
        
        // Return the prior covariance
        BLA::Matrix<2, 2, double> get_prior_cov();
        
        // Return the posterior covariance
        BLA::Matrix<2, 2, double> get_post_cov();
        
        // Return the kalman gain
        BLA::Matrix<2, 2, double> get_kalman_gain();
        
        // Return the measurement [speed; heading]
        BLA::Matrix<2, 1, double> get_measurement();
        
        // Return the coordinates [latitude; longitude]
        BLA::Matrix<2, 1, double> get_coords();
        
        // Return the accuracy vector
        BLA::Matrix<2, 1, double> get_accuracy();
        
        // Return the state update matrix
        BLA::Matrix<2, 2, double> get_A();
        
        // Return the state update covariance matrix
        BLA::Matrix<2, 2, double> get_Q();
        
        // Return the measurement update matrix
        BLA::Matrix<2, 2, double> get_R();

    private:
        // internally used variables

        BLA::Matrix<2, 1, double> prior_est; // prior estimate 
        BLA::Matrix<2, 1, double> post_est;  // posterior estimate
        BLA::Matrix<2, 2, double> prior_cov; // prior covariance
        BLA::Matrix<2, 2, double> post_cov;  // posterior covariance
        BLA::Matrix<2, 2, double> kalman_gain; // kalman gain
        BLA::Matrix<2, 2, double> A; // state update matrix 
        BLA::Matrix<2, 2, double> Q; // process covariance
        BLA::Matrix<2, 2, double> R; // measurement update matrix
        BLA::Matrix<2, 1, double> measurement; // measurement values [speed; heading]
        BLA::Matrix<2, 1, double> accuracy;  // accuracy [speed accuracy; heading accuracy]
        BLA::Matrix<2, 1, double> coords;  // coordinates [latitude; longitude] 
        BLA::Matrix<HISTORY, 2, double> history; // not used - added for experimentation with history-based control
        
        int hist_index;
        SFE_UBLOX_GNSS myGNSS;
};

#endif