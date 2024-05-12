#include "Arduino.h"
#include "IQT_VELOCITY_EKF.h"
#include "math.h"

// Theo DeGuzman
// Johns Hopkins University
// IQT24 Senior Design Project
// V2 Prototype Code
// Not for production use -- end user assumes responsibility for usage and testing of this code


/* Initally the "IQT_GPS_EKF" class (deleted), this class is essentially a 1-D noise filter on speed and heading, from GPS data.
    In it's entirety, it is probably not necessary for this to exist, but it's well-tested and integrated into the IQT24 V2 code architecture.
*/

// inialize velocity EKF variables
VELOCITY_EKF::VELOCITY_EKF(){
    this->prior_est.Fill(0);
    this->post_est.Fill(0);
    this->prior_cov.Fill(0);
    this->post_cov.Fill(0);
    this->kalman_gain.Fill(0);
    this->A.Fill(0);
    this->Q.Fill(0);
    this->R.Fill(0);
    this->hist_index = HISTORY - 1;
}

void VELOCITY_EKF::begin() {
    return;    
}

BLA::Matrix<2, 1, double> VELOCITY_EKF::step(double speed, double heading, double lat, double lon, double speed_acc, double heading_acc) {
    BLA::Matrix<2, 2, double> I =  {1, 0,
                                    0, 1}; // note that H = I and therefore will be remove to reduce calcualtions
    this->take_measurement(speed, heading, lat, lon);
    this->step_process_model();
    this->update_Q();
    this->update_R(speed_acc, heading_acc);

    this->prior_cov = this->A*this->post_cov*(~this->A) + this->Q;
    // this->kalman_gain = this->prior_cov*(~H)*Inverse(H*this->prior_cov*(~H)+R);    
    this->kalman_gain = this->prior_cov*Inverse(this->prior_cov+this->R);
    // this->post_est = this->prior_est + this->kalman_gain*(measurement - H*this->prior_est);

    // adjusts for wrap-around from 360 to 0
    if (this->prior_est(1) > 340E5 && this->measurement(1) < 20E5) {
        this->prior_est(1) = this->prior_est(1) - 360E5;
    } else if (this->prior_est(1) < 20E5 && this->measurement(1) > 340E5) {
        this->prior_est(1) = this->prior_est(1) + 360E5;
    }

    // update posterior estimate
    double last_heading = this->post_est(1);
    this->post_est = this->prior_est + this->kalman_gain*(this->measurement - this->prior_est);

    // normalizes back to 0 to 360
    if (this->post_est(1) > 360E5) {
        this->post_est(1) = this->post_est(1) - 360E5;
    } else if (this->post_est(1) < 0) {
        this->post_est(1) = this->post_est(1) + 360E5;
    }

    // return last value if errored
    if (this->post_est(1) > 360E5 || this->post_est(1) < 0) {
        this->post_est(1) = last_heading;
        return this->post_est;
    }

    // this->post_cov = (I - this->kalman_gain*H)*this->prior_cov*(~(I - this->kalman_gain*H)) + this->kalman_gain*R*(~this->kalman_gain);
    this->post_cov = (I - this->kalman_gain)*this->prior_cov*(~(I - this->kalman_gain)) + this->kalman_gain*this->R*(~this->kalman_gain);
    return this->post_est;
}

void VELOCITY_EKF::step_process_model() {
    this->prior_est(0) = this->post_est(0);

    // if (this->measurement(0) <= 1000) {
    this->A(0, 0) = 1.0;
    this->A(1, 1) = 1.0;
    this->prior_est(1) = this->post_est(1);
    // Serial.println("Too slow");
    return;
    // }
    
    // int last_index = this->hist_index;
    // this->hist_index = (this->hist_index + 1) % HISTORY;

    // this->history(this->hist_index, 0) = this->measurement(0);
    // this->history(this->hist_index, 1) = this->measurement(1);

    // int count = 0;
    // for (int i = 0; i < HISTORY; i++) {
    //     if (this->history(i, 0) != 0.0) {
    //         count++;
    //     }
    // }
    
    // if (count < HISTORY) {
    //     this->A(0, 0) = 1.0;
    //     this->A(1, 1) = 1.0;
    //     this->prior_est(1) = this->post_est(1);
    //     Serial.println("Too few points");
    //     return;
    // }

    // const int c_count = count;
    // // BLA::Matrix<20, 3, double> lsr;
    // BLA::Matrix<HISTORY, 1, double> x;
    // BLA::Matrix<HISTORY, 1, double> y;
    // // BLA::Matrix<3, 1, double> ABC;
    // BLA::Matrix<1, 1, double> ABC;
    // BLA::Matrix<HISTORY, 1, double> ones;
    // ones.Fill(1);
    // double inv = 0;
    // double xy = 0;
    // // lsr.Fill(0);
    // for (int i = 0; i < c_count; i++) {
    //     // lsr(i, 1) = (this->history(i, 1)/1.0E7 * M_PI /180) * cos(this->history(this->hist_index, 0)/1.0E7 * M_PI /180); // x
    //     // lsr(i, 2) = (this->history(i, 0)/1.0E7 * M_PI /180); // y
    //     // lsr(i, 0) = sqrt(lsr(i, 1)*lsr(i, 1) + lsr(i, 2)*lsr(i, 2)); // R
    //     x(i, 0) = (this->history(i, 1)-this->history(this->hist_index, 1)) * cos(this->history(this->hist_index, 0)/1.0E7 * M_PI /180); // x
    //     // x(i, 1) = 1; // one
    //     y(i, 0) = (this->history(i, 0)-this->history(this->hist_index, 0)); // y


        
    //     Serial.print("C: ");
    //     Serial.print(x(i, 0));
    //     inv += (x(i, 0))*(x(i, 0));
        
    //     Serial.print(", ");
    //     Serial.println(y(i, 0));
    //     xy += (x(i, 0))*(y(i, 0));

        
    // }

    // // run circular regression
    // ABC = Inverse((~x)*x) * (~x) * y;

    // // double x_soln = -lsr(1)/(2*lsr(0));
    // // double y_soln = -lsr(2)/(2*lsr(0));
    // // double r_soln = sqrt(4*lsr(0)+lsr(1)*lsr(1)+lsr(2)*lsr(2))/(2*lsr(0));

    
    
    // // double dx_tangent = (measurement(1)/1.0E7 * M_PI /180) - x_soln;
    // // double dy_tangent = (measurement(0)/1.0E7 * M_PI /180) - y_soln;
    
    // // double slope = -dx_tangent/dy_tangent;
    // double slope = xy/inv;
    // Serial.print("slope: ");
    // Serial.println(slope);

    // this->prior_est(1) = (-(atan(slope) * 180 / M_PI) + 90)*1E5;
    // Serial.print("Pre=prior: ");
    // Serial.println(this->prior_est(1));


    // double temp_est = this->prior_est(1);
    // if (temp_est > 340E5 && this->measurement(1) < 20E5) {
    //     temp_est = temp_est - 360E5;
    // } else if (temp_est < 20E5 && this->measurement(1) > 340E5) {
    //     temp_est = temp_est + 360E5;
    // }

    // if (abs(temp_est-this->measurement(1)) > min(abs((temp_est+180E5)-this->measurement(1)), abs((temp_est-180E5)-this->measurement(1)))) {
    //     this->prior_est(1) = ((long) (this->prior_est(1) + 180E5)) % ((long) 360E5);
    // }

    // if (abs(this->post_est(1) - this->prior_est(1)) > 15E5 || abs(this->post_est(1) - this->prior_est(1)) < 340E5) {
    //     this->A(0, 0) = 1.0;
    //     this->A(1, 1) = 1.0;
    //     this->prior_est(1) = this->post_est(1);
    //     Serial.println("Extreme variance");
    //     return;
    // }

    // if (this->prior_est(1) > 360E5) {
    //     this->prior_est(1) = this->prior_est(1) - 360E5;
    // } else if (this->prior_est(1) < 0) {
    //     this->prior_est(1) = this->prior_est(1) + 360E5;
    // }

    // Serial.println(this->prior_est(1));
    // if (!(this->prior_est(1) > 0 && this->prior_est(1) < 360E5)) {
    //     this->A(0, 0) = 1.0;
    //     this->A(1, 1) = 1.0;
    //     this->prior_est(1) = this->post_est(1);
    //     Serial.println("Overflow");
    //     return;
    // }
    // this->A(0, 0) = 1.0;
    // this->A(1, 1) = this->prior_est(1) - this->post_est(1);  
    // this->Q(1, 1) = (x*ABC-y)(0)*(x*ABC-y)(0)+(x*ABC-y)(1)*(x*ABC-y)(1);
    // Serial.print("R^2: ");
    // Serial.println(this->Q(1, 1));
}

void VELOCITY_EKF::update_Q() {

    this->Q.Fill(0);
    this->Q(0, 0) = 500*500; // m/s variance
    this->Q(1, 1) = 5.0E5*5.0E5; // 5 degrees things
}

void VELOCITY_EKF::take_measurement(double speed, double heading, double lat, double lon) {
    this->measurement(0) = speed;
    this->measurement(1) = heading;
    this->coords(0) = lat;
    this->coords(1) = lon;

}

void VELOCITY_EKF::update_R(double speed_acc, double heading_acc) {

    this->accuracy.Fill(0);
    this->accuracy(0) = speed_acc;
    this->accuracy(1) = heading_acc;

    this->R.Fill(0);
    this->R(0, 0) = this->accuracy(0)*this->accuracy(0); // assuming its returning accuracy in mm -> m
    this->R(1, 1) = this->accuracy(1)*this->accuracy(1); // assuming its retunring accuracy in deg E-5

}



BLA::Matrix<2, 1, double> VELOCITY_EKF::get_prior_est(){
    return this->prior_est;
}
BLA::Matrix<2, 1, double> VELOCITY_EKF::get_post_est() {
    return this->post_est;
}
BLA::Matrix<2, 2, double> VELOCITY_EKF::get_prior_cov(){
    return this->prior_cov;
}
BLA::Matrix<2, 2, double> VELOCITY_EKF::get_post_cov(){
    return this->post_cov;
}
BLA::Matrix<2, 2, double> VELOCITY_EKF::get_kalman_gain(){
    return this->kalman_gain;
}
BLA::Matrix<2, 1, double> VELOCITY_EKF::get_measurement(){
    return this->measurement;
}
BLA::Matrix<2, 1, double> VELOCITY_EKF::get_coords(){
    return this->coords;
}
BLA::Matrix<2, 1, double> VELOCITY_EKF::get_accuracy(){
    return this->accuracy;
}
BLA::Matrix<2, 2, double> VELOCITY_EKF::get_A(){
    return this->A;
}
BLA::Matrix<2, 2, double> VELOCITY_EKF::get_Q(){
    return this->Q;
}
BLA::Matrix<2, 2, double> VELOCITY_EKF::get_R(){
    return this->R;
}






