#ifndef IQT_CONTROLLER_H
#define IQT_CONTROLLER_H

#include "Arduino.h"
#include "IQT_vehicle.h"
#include "Servo.h"

#define HIST_LEN 50

// Theo DeGuzman
// Johns Hopkins University
// IQT24 Senior Design Project
// V2 Prototype Code
// Not for production use -- end user assumes responsibility for usage and testing of this code

/*
    IQT_Controller

    On the highest level, class takes in vehicle state information (from and IQT_vehicle object) and outputs motor/payload commands
*/

class IQT_CONTROLLER
{
    public:
        IQT_CONTROLLER();
        

        /* 
            begin - intializing controller for use
                t1_pin: thruster 1 pwm control pin
                t2_pin: thruster 2 pwm control pin
                t3_pin: thruster 3 pwm control pin
                t1_pin_rev: thruster 1 auxillary control pin (sometimes used as reverse)
                t2_pin_rev: thruster 2 auxillary control pin (sometimes used as reverse)
                t3_pin_rev: thruster 3 auxillary control pin (sometimes used as reverse)
                t1_relay:  on/off relay for thruster 1
                t2_relay:  on/off relay for thruster 2
                t3_relay:  on/off relay for thruster 3
                payload_relay:  on/off relay for the payload
                manual_steer: digital input pin from remote control transciever to read steering PWM off
                manual_thrust: digital input pin from remote control transciever to read thrust PWM off
                mission: general mission parameters pointer
            */
        void begin(int t1_pin, int t2_pin, int t3_pin, int t1_pin_rev, int t2_pin_rev, int t3_pin_rev, int t1_relay, int t2_relay, int t3_relay, int payload_relay, int manual_steer, int manual_thrust, Mission mission);
        

        /*
            command - called on every time step, takes state input and call functions to control motors
                mode: current mode the vehicle is in
                state: the vehicle state vector
                wp: the current target waypoint 
                battery: the current battery state vector
                motors: the enablement status of all the motors (not implemented yet)
                est_heading: the internally derived heading estimate
                target_speed: the target speed of the system
        */
        void command(Mode mode, long state[], Waypoint wp, long battery[], bool motors[], long est_heading, unsigned long target_speed);
        
        
        /*
            Record the current vehicle state into BL______.csv
            The numbers after BL is MMDDHH (i.e. different number/file for every hour)
                t: time struct from SATCOMM clock

            CSV Format
            Column  Value
            1       Time from satellite clock
            2       Clock time (millis)
            3       Target latitude (degrees E7) (of the aim point, given by guidance)
            4       Target longitude (degrees E7) (of the aim point, given by guidance)
            5       Target waypoint radius (m) (of the final point)
            6       Target waypoint type (of the final point)
            7       Target waypoint time_on_station (of the final point)
            8       Time to next (milliseconds)
            9       Manual steering reading [-100, 100]
            10      Manual thrust reading [0, 100]
            11      Target speed (m/s E3)
            12      Target bearing (degrees E5)
            13      Error speed (m/s E3)
            14      Error bearing (deg E3)
            15      Steering output [-100, 100]
            16      Thrust output [0, 100]
            17      Trim output [-50, 50]
            18      Large steering proporitional gain
            19      Small steering proporitional gain
            20      Small steering normalized integral term
            21      Thruster additive term

        */
        void record_state(tm t); 
        
        
        /* get_control_state - Store the current vehicle control state in vector "control"
        
            Index       Value
            0           target speed
            1           current error in speed
            2           target bearing
            3           current error in bearing
            4           current steering value [-100, 100]
            5           current thrust value [0, 100]
            6           current trim value [-50, 50]
        */        
        void get_control_state(long control[7]); 
        

        // void check_thruster_health(); // Never implemented
        
        /* Return current steer value [-100, 100]*/
        int get_steer();

        /* Return current thrust value [0, 100]*/
        int get_thrust();

        /* Return current trim value [-50, 50]*/
        int get_trim();


    private:
        Waypoint target; // current waypoint to aim at
       
        // Objects for pwm output to various servo points
        Servo t1;
        Servo t2;
        Servo t3;
        Servo t1_rev;
        Servo t2_rev;
        Servo t3_rev;

        // Digital output pins to relays
        int t1_relay;
        int t2_relay;
        int t3_relay;
        int payload_relay;

        int manual_steer;   // steering pwm input pin
        int manual_thrust;  // thrust pwm input pin

        unsigned long last_command;     // last time controller commanded a new motor state
        unsigned long time_to_next;     // hard coded 333 ms minimum time delay 

        long target_speed;      // the ideal speed
        long target_bearing;    // the ideal bearing
        long error_brng;        // the difference between the current and target headings
        long error_speed;       // the difference between the current and target speeds
        float steering;         // -100% to 100%
        float thrust;           // 0% to 100%
        float trim;             // -50% to 50%

        float steer_big_p_gain;     // Large steering proporitional gain
        float steer_small_p_gain;   // Small steering proporitional gain
        float steer_i_gain;         // Small steering normalized integral term
        float thrust_p_gain;        // Thruster additive term
        long transition_threshold;  // The transition threshold between big and small bearing error


        /* execute_manual_command - set steer and thrust based off of RC transciever values  */
        void execute_manual_command();

        /*
            execute_cruise - called from command, if in cruise or loiter cruise
                state: the vehicle state vector
                est_heading: the internally derived heading estimate
                target_speed: the target speed of the system
            
            Sets steer, trim, and thrust values
        */
        void execute_cruise(long state[], long est_heading, unsigned long tar_speed);
        
        /*
            command_motors - turn steer/thrust/trim values into motor output values
                motor_enabled: a list of motors active
        */       
        void command_motors(bool motor_enabled[]);

};


/*
    determine_err_bearing - determine the difference between two bearings, using shortest arc approach
        curr: current bearing in degrees E5
        target: target bearing in degrees E5

    Return values in the range of [-180, 180]
*/
long determine_err_bearing(long curr, long target);

#endif