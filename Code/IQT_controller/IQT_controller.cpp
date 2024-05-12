#include "Arduino.h"
#include "IQT_COMMUNICATIONS.h"
#include "IQT_vehicle.h"
#include "IQT_controller.h"

// Theo DeGuzman
// Johns Hopkins University
// IQT24 Senior Design Project
// V2 Prototype Code
// Not for production use -- end user assumes responsibility for usage and testing of this code


IQT_CONTROLLER::IQT_CONTROLLER(){
    return;
}

void IQT_CONTROLLER::begin(int t1_pin, int t2_pin, int t3_pin, int t1_pin_rev, int t2_pin_rev, int t3_pin_rev, int t1_relay, int t2_relay, int t3_relay, int payload_relay, int manual_steer, int manual_thrust, Mission mission){

        this->t1_relay = t1_relay;
        this->t2_relay = t2_relay;
        this->t3_relay = t3_relay;
        this->payload_relay = payload_relay;

        Serial.println("Starting thruster init");


        /* thruster 2 disabled by comments */
        this->t1.attach(t1_pin, 1000, 2000);
        // this->t2.attach(t2_pin, 1000, 2000);
        this->t3.attach(t3_pin, 1000, 2000);
        this->t1_rev.attach(t1_pin_rev, 1000, 2000);
        // this->t2_rev.attach(t2_pin_rev, 1000, 2000);
        this->t3_rev.attach(t3_pin_rev, 1000, 2000);
        delay(1000);


        this->t1.write(0);
        this->t1_rev.write(0);
        // this->t2.write(0);  
        this->t3.write(0);
        this->t3_rev.write(0);
        delay(1000);


        Serial.println("Finish thruster init");


        this->manual_steer = manual_steer;
        this->manual_thrust = manual_thrust;

        this->last_command = 0;
        this->time_to_next = 333; //execute control every 333 ms

        this->target_speed = mission.cruise_speed;
        this->error_brng = 0;
        this->error_speed = 0;
        this->steering = 0;
        this->thrust = 0;
        this->trim = 0;

        // controller parameters - hardcoded for now
        this->steer_big_p_gain = mission.steer_big_p_gain;
        this->steer_small_p_gain = mission.steer_small_p_gain;
        this->steer_i_gain = mission.steer_i_gain;
        this->thrust_p_gain = mission.thrust_p_gain;  // increase throttle percent by 1 for 
        this->transition_threshold = mission.transition_threshold;

}

void IQT_CONTROLLER::record_state(tm t){

    char date[12];
    char time[8];
    sprintf(time, "%02d:%02d:%02d", t.tm_hour, t.tm_min, t.tm_sec);
    sprintf(date, "bl%02d%02d%02d.csv", (t.tm_mon + 1) % 100, (t.tm_mday) % 100, (t.tm_hour) % 100);

    File myFile = SD.open(date, FILE_WRITE);     
    // if the file opened okay, write to it:

    if (myFile) 
    {
        myFile.print(time);
        myFile.print(","); 
        myFile.print(millis()); 
        myFile.print(","); 

        myFile.print(this->target.latitude);
        myFile.print(",");
        myFile.print(this->target.longitude);
        myFile.print(",");
        myFile.print(this->target.radius);
        myFile.print(",");
        myFile.print(this->target.type);
        myFile.print(",");
        myFile.print(this->target.time_on_station);
        myFile.print(",");
        myFile.print(this->time_to_next);
        myFile.print(",");

        myFile.print(this->manual_steer);
        myFile.print(",");
        myFile.print(this->manual_thrust);
        myFile.print(",");
        myFile.print(this->target_speed);
        myFile.print(",");
        myFile.print(this->target_bearing);
        myFile.print(",");        
        myFile.print(this->error_speed);
        myFile.print(",");
        myFile.print(this->error_brng);
        myFile.print(",");
        myFile.print(this->steering);
        myFile.print(",");
        myFile.print(this->thrust);
        myFile.print(",");
        myFile.print(this->trim);
        myFile.print(",");
        myFile.print(this->steer_big_p_gain);
        myFile.print(",");
        myFile.print(this->steer_small_p_gain);
        myFile.print(",");
        myFile.print(this->steer_i_gain);
        myFile.print(",");
        myFile.print(this->thrust_p_gain);
        myFile.print(","); 
           
        myFile.println();
        myFile.close();
    } 
    else 
    {
    //   Serial.println("error opening csv.txt");
    }
}

void IQT_CONTROLLER::command(Mode mode, long state[], Waypoint wp, long battery[], bool motor_enabled[], long est_heading, unsigned long tar_speed){

    this->target = wp;

    if (mode == power_safe) {
        digitalWrite(this->t1_relay, LOW);
        digitalWrite(this->t2_relay, LOW);
        digitalWrite(this->t3_relay, LOW);
        digitalWrite(this->payload_relay, LOW);
        return;
    } else if (mode == turbulence_safe || mode == loiter) {
        digitalWrite(this->t1_relay, LOW);
        digitalWrite(this->t2_relay, LOW);
        digitalWrite(this->t3_relay, LOW);
        digitalWrite(this->payload_relay, HIGH);
        this->thrust = 0;
        this->steering = 0;
        this->trim = 0;
        return;
    } else if (mode == loiter_cruise) {
        digitalWrite(this->t1_relay, motor_enabled[0]);
        digitalWrite(this->t2_relay, motor_enabled[1]);
        digitalWrite(this->t3_relay, motor_enabled[2]);
        digitalWrite(this->payload_relay, HIGH);
    } else {
        digitalWrite(this->t1_relay, motor_enabled[0]);
        digitalWrite(this->t2_relay, motor_enabled[1]);
        digitalWrite(this->t3_relay, motor_enabled[2]);
        digitalWrite(this->payload_relay, LOW);
    }

    if (mode != manual) {
        while(millis() - this->last_command < time_to_next) {}
        this->last_command = millis();
    }

    
    switch(mode) {
        case manual:
            this->execute_manual_command();
            break;
        case cruise:
        case loiter_cruise:
            this->execute_cruise(state, est_heading, tar_speed);
            break;
    }

    this->command_motors(motor_enabled);

}


void IQT_CONTROLLER::execute_manual_command() {
    
    // Read out the values off the transciever 
    this->steering = readChannel(this->manual_steer, -100, 100, 0);
    this->thrust = readChannel(this->manual_thrust, 100, -100, 0)*2.0;
    this->trim = 0;


    // Bound the output parameters to their limits
    // Steering [-100, 100]
    if (this->steering > 100) {
        this->steering = 100;
    } else if (this->steering < -100) {
        this->steering = -100;
    } else if (this->steering < 10 && this->steering > -10) {
        this->steering = 0;
    }
    
    // Thrust [0, 100]
    if (this->thrust > 100) {
        this->thrust = 100;
    } else if (this->thrust < -100) {
        this->thrust = -100;
    }
    return;
}

void IQT_CONTROLLER::execute_cruise(long state[], long est_heading, unsigned long tar_speed) {
    
    // Determine or retrieve target bearing or speed
    this->target_bearing = determine_bearing(state[0], state[1], this->target.latitude, this->target.longitude);
    this->target_speed = tar_speed;
    
    // Find error bearing
    this->error_brng = determine_err_bearing(est_heading, this->target_bearing);
    this->error_speed = this->target_speed - state[2];

    // using the gps heading to position craft at large errors with basic P control
    if (abs(this->error_brng) > this->transition_threshold) {
        this->steering = error_brng*steer_big_p_gain;
        this->thrust = 50;

    } else {  // use a more refined control law at small breaing erros

        this->steering = error_brng*steer_small_p_gain;
        this->trim += error_brng*steer_i_gain;
        this->thrust += error_speed*thrust_p_gain;

        if (this->trim > 0 && this->error_brng < -4E5 ) {
            this->trim = 0;
        } else if (this->trim < 0 && this->error_brng > 4E5 ) {
            this->trim = 0;
        }
    }

    // Bound the output parameters to their limits
    // Steering [-100, 100]
    if (this->steering > 100) {
        this->steering = 100;
    } else if (this->steering < -100) {
        this->steering = -100;
    }

    // Trim [-50, 50]
    if (this->trim > 50) {
        this->trim = 50;
    } else if (this->trim < -50) {
        this->trim = -50;
    }
    
    // Thrust [0, 100]
    if (this->thrust > 100) {
        this->thrust = 100;
    } else if (this->thrust < 0) {
        this->thrust = 0;
    }

}

void IQT_CONTROLLER::command_motors(bool motor_enabled[]) {

    // TODO: Add for if one motor goes offline

    /*for two-motor with reverse activated through auxillary pin*/ 

    // int left = this->thrust + this->steering + this->trim;  
    // int middle = this->thrust - abs(this->steering);  
    // int right = this->thrust - this->steering - this->trim;

    // int left_rev;
    // int mid_rev;
    // int right_rev;

    // if (left > 100) { 
    //     left = 100;
    //     left_rev = 0;
    // } else if (left < 0) { 
    //     left = abs(left);
    //     // left = 0;
    //     left_rev = 180;
    //     if (left > 40) {left = 40;}
    // }

    // if (right > 100) {
    //     right = 100;
    //     right_rev = 0;
    // } else if (right < 0) { 
    //     right = abs(right);
    //     // right = 0;
    //     right_rev = 180;
    //     if (right > 40) {right = 40;}
    // }

    // this->t1_rev.write(map(left_rev, 0, 100, 0, 180));
    // // this->t2.write(map(middle, 0, 100, 0, 180));
    // this->t3_rev.write(map(right_rev, 0, 100, 0, 180));


    /* for two-motor without reverse */
    int left = this->thrust + this->steering + this->trim;  
    int middle = this->thrust - abs(this->steering);  
    int right = this->thrust - this->steering - this->trim;

    // Limit motor values 
    if (left > 100) { 
        left = 100;
    } else if (left < 3) { 
        left = 0;
    }
    if (right > 100) {
        right = 100;
    } else if (right < 3) { 
        right = 0;
    }

    /* Write thruster values back to motors */
    this->t1.write(map(left, 0, 100, 0, 180));
    // this->t2.write(map(middle, 0, 100, 0, 180));
    this->t3.write(map(right, 0, 100, 0, 180));
    
    return; 
}

int IQT_CONTROLLER::get_steer() {return this->steering;}
int IQT_CONTROLLER::get_thrust() {return this->thrust;}
int IQT_CONTROLLER::get_trim() {return this->trim;}

void IQT_CONTROLLER::get_control_state(long control[7]) {
    control[0] = this->target_speed;
    control[1] = this->error_speed;
    control[2] = this->target_bearing;
    control[3] = this->error_brng;
    control[4] = this->steering;
    control[5] = this->thrust;
    control[6] = this->trim;
}

long determine_err_bearing(long curr, long target) {
    long err;

    if (abs(curr-target) < 180E5) {
        err = curr-target;
    } else if (abs(curr-(target + 360E5)) < 180E5) {
        err = curr-(target + 360E5);
    } else if (abs(curr-(target - 360E5)) < 180E5) {
        err = curr-(target - 360E5);
    } else {
        err = 0;
    }

    return -err;
    
}


