#ifndef IQT_VEHICLE_H
#define IQT_VEHICLE_H

#include "Arduino.h"
#include "IQT_COMMUNICATIONS.h"
#include "IridiumSBD.h"

// Theo DeGuzman
// Johns Hopkins University
// IQT24 Senior Design Project
// V2 Prototype Code
// Not for production use -- end user assumes responsibility for usage and testing of this code

/*
    IQT_Vechicle

    This class holds all the vehicle information, determines vehicle states, and runs the guidance algorithms
*/

class IQT_VEHICLE 
{
    public:
        IQT_VEHICLE();

        /*
            Begin - start a vehicle object

                mission: pointer to mission struct that defines important vehicle parameters
                exit_brown_out: defines the voltage improvement required to exit the brownout state
                rc_pin: the pin reading the transciever channel that represents autonomous vs manual mode
                reset_pin: the pin to set low if the board needs a hardware reset
        */
        void begin(Mission *mission, unsigned long exit_brown_out, int rc_pin, int reset_pin);
        
        /*
            Determine the vehicle's mode based on the following criteria:

            If manual mode is on, enter manual mode and terminate method

            If the vehicle is in a safe mode (turbulence or brownout):
                - Exit safe mode if the acceptable voltage/turbulence threshold is met for the specified time
                - Remain in the safe mode if this is not true
                - Brownout safemode overrides turbulence safe mode
            
            If the vehicle is not in safe mode:
                - Check if battery voltage or measured turbulence violates thresholds
                    - If so, enter the appropriate safe mode

            If the vehicle is not in any safe mode (still):

                - if the current waypoint is a "naviagtion" type waypoint
                    - enter cruise mode

                - if the current waypoint is a "loitering" type waypoint
                    - if the boat is currently in "loiter" mode 
                        - if the boat is within 67% of the designated target radius
                            - remain in "loiter" mode
                        - if the boat has drifted outside of 67% the designated target radius
                            - enter "loiter_cruise" mode (to correct back to center)
                
                    - if the boat is in "loiter cruise mode"
                        - if the boat is outside of 33% of the designated target radius
                            - remain in "loiter_cruise" mode
                        - if the boar is inside 33% of the designated target radius
                            - switch to "loiter" mode
            
        */
        void determine_mode();
        
        /*
            determine_state - calcualtes a vechicle state vector based off peripheral state vectors

                s1: state vector from peripheral 1, long-based format of PERIPHERAL_STATE_VECTOR
                s2: state vector from peripheral 2, long-based format of PERIPHERAL_STATE_VECTOR
                g1: reset pin for peripheral 1
                g2: reset pin for peripheral 2

            First, error checks values using the following metrics.  Fails if any of the following are true:
                - GPS geometric dilution of precision is > 90%
                - GPS geometric dilution of precision reads < 0.01%
                - GPS location reads within 0.0000005 degrees of 0.0 N, 0.0 E
                    - (not possible to stay at this precise location (within 4 cm) due to noise)
                - Sensor has been force disabled
            
            If either sensor is reads faulty and has not been reset in the last 10 seconds, reset.

            If one sensor is bad, use the good sensor directly as the estimate of vehicle state

            If both sensores are good, or both are bad, use a weighted average of the measurements
                - Weights are the compelmement of geomtric dilution of precision
            
        */
        void determine_state(long s1[], long s2[], int g1, int g2);
        
        /*
            determine_heading - Estimate the heading and store it in est_heading

            If traveling faster than 0.75 m/s, use the heading as defined by the direction between the most-recently obstained GPS fix
            and the next-most-recent unique GPS fix. Tends to update faster than GPS intenally derived heading.

            If traveling slower than 0.75 m/s, use the internallly-derived GPS heading
        
        */
        void determine_heading();
        
        /*
            Read the various values off the analog pins

            User can define custom functions to store useful long output, or just calculate the raw values
        */
        void read_battery();
        
        /*
            Checks if a waypoint has been reached or visited for sufficient time

            Returns a pointer to the updated mission, but this returned value isn't used anywhere
        */
        Mission* check_waypoint_progression();
        
        /* Overwrites this->mission with Mission mis passed to the function*/
        void get_new_mission(Mission *mis);

        /*
            Record the current vehicle state into CL______.csv
            The numbers after CL is MMDDHH (i.e. different number/file for every hour)
                t: time struct from SATCOMM clock

            CSV Format
            Column  Value
            1       Time from satellite clock
            2       Clock time (millis)
            3       Vehicle Mode
            4       Estimated vehicle heading
            5       Last latitude (note: this updates before getting recorded so not really useful as-is)
            6       Last longitude (note: this updates before getting recorded so not really useful as-is)
            7-14    Obverved vehicle state as per VEHICLE_STATE_VECTOR
            15-22   Measured battery and sensor state
            23      Index of current waypoint in waypoint list
            24      Time elapsed at current waypoint (milliseconds)
            25      Bool: is motor 1 enabled
            26      Bool: is motor 2 enabled
            27      Bool: is motor 3 enabled
            28      Peripheral 1 enablment state
            29      Peripheral 2 enablment state
            
        */
        void record_state(tm t);
        
        /* 
            dist_from_wp - calls determine_error_distance with current position and current waypoint
                The result is the distance between the current position and current waypint in meters
        */
        long dist_from_wp();
        
        /*
            broadcast_satellite - check for incoming satellite messages and send updates as appropriate
                modem: the Iridium Short-Burst Data unit to be used
                state1: the vehicle state as detected by peripheral 1
                state2: the vehicle state as detected by peripheral 2
                steer: the current steer value [-100, 100]
                thrust: the current thrust value [0, 100]
                trim: the current trim value [-50, 50]
        */
        void broadcast_satellite(IridiumSBD modem, long state1[], long state2[], int steer, int thrust, int trim);
        
        /* Return current vehicle mode */
        Mode get_mode();
        
        /* Return current mission waypoint */
        Waypoint get_waypoint();
        
        /* Determines an aim-point based on the current waypoint (finish) and the previous waypoint (start)
        
            If this is the first waypoint in the mission, the aim point is the waypoint.

            If this is not the first waypoint and the boat currently is not further from the start
            than the finsih is from the start:
                - Set the aim point as follows:
                    1. Calculate the distance (d) from the start to the boat
                    2. Find the point that is d away from the start on the line (great circle) that connects start and finish
                    3. Progress some additional distance (lead distance) along that line, moving in the direction of the finish
                    4. Return this point as the target/aim point
        
        */
        Waypoint get_target();
        
        /* Store the vehicle state vector in "state" */
        void get_state(long state[]);
        
        /* Store the vehicle battery vector in "bat" */
        void get_battery(long bat[]);
        
        /* Store the motor enabled vector in "enb" */
        void get_motor_enabled(bool enb[]);
        
        /* Store the sensor/peripheral enabled vector in "enb" */
        void get_sensor_enabled(Sensor_enable enb[]);
        
        /* Return the stored, estimated heading */
        long get_est_heading();
        
        /* Check if it has been a week and reset the board */
        void check_for_reset();
        
        /* Return designated cruise speed */
        unsigned long get_target_speed();


    private:
        Mode mode;
        
        
        long observed_state[8]; // lat E7, long E7, speed E3, bearing E5, roll E5, pitch E5, turbulence, DOP E2
        long battery_state[8]; // Vbatt, Asolar, Apayload, Anav, At1, At2, At3, BatTemp
        
        
        Mission *mission;

        // estimated heading 
        long est_heading;

        // used for finite difference heading approximation
        long last_lat;
        long last_long;
        
        // Various mission parameters
        long update_interval; // how often a satellite update should be sent (milliseconds)
        long rough_sea_thresh; // the unitless gyroscope measurement that defines a rough sea state
        unsigned long brown_out_thresh; // the battery voltage at which the battery has "browned out" (V E2)
        unsigned long exit_brown_out_thresh; // how much battery voltage must increase before exiting battery brownout (V E2)
        unsigned long exit_safe_mode_time; // how long the boat must return to a safe state before exiting a safe mode (milliseconds)
        unsigned long time_mode_change; // how long it has been since the last mode change (milliseconds)
        unsigned long last_waypoint_check; // how long has it been since the last time it was checked if the boat should go to the next waypoint (milliseconds)
        int rc_mode; // pin for reading manual/autonomous channel from rc reciever
        unsigned long last_satellite_broadcast; // last time (milliseconds) when there was a satellite update sent
        int reset_pin; // pin for the boat hardware to reset itself 
        unsigned long last_periph_reset; // last time one of the peripheral sensing (GPS-INS) boards was reset

        /*
            unpack_request_waypoint_list - If a request for the list of waypoints comes in, send back list of waypoints
                modem: Iridium short-burst data module to use

            Sends data back in the WAYPOINT_LIST format

        */
        void unpack_request_waypoint_list(IridiumSBD modem);
        
        /*
            unpack_request_waypoint - If a request for a specific waypoint comes in, send back info about that waypoints
                modem: Iridium short-burst data module to use

            Sends data back in the WAYPOINT format

        */
        void unpack_request_waypoint(IridiumSBD modem, byte buf[]);
        
        /*
            unpack_change_waypoint - If a request for a specific waypoint comes in, send back info about that waypoints
                modem: Iridium short-burst data module to use
                buf: buffer that message data has been read into

            Reads data with a CHANGE_WAYPOINT flag

            Expected byte stream:
            Byte    Content
            0       START BYTE
            1       CHANGE_WAYPOINT
            2       index of waypoint under editing
            3-6     Long - New latitude E7 (deg)
            7-10    Long - New longitude E7 (deg)
            11-14   Long - Wyapoint radius E3 (meters)
            15-18   Long - Waypoint type
            19-22   Long - Waypoint ime on station (milliseconds)
            23      END BYTE

            Saves this data into current mission struct, and writes it back to the SD card
            Sends waypoint back to ground control station as confirmation of receipt

        */
        void unpack_change_waypoint(IridiumSBD modem, byte buf[]);
        
        /*
            unpack_get_mission_params - If a request for current mission parameters comes in, send back info
                modem: Iridium short-burst data module to use

            Sends data back in the MISSION_PARAMS format

        */
        void unpack_get_mission_params(IridiumSBD modem);
        
        /*
            unpack_change_mission_params - If a request to change mission parameters is sent, change those values
                modem: Iridium short-burst data module to use
                buf: buffer that message data has been read into

            Reads data with a CHANGE_MISSIONS_PARAMS flag

            Expected byte stream input is of identical format to MISSION_PARAMS format (except byte index 1 has a different message flag)

            Saves this data into current mission struct, and writes it back to the SD card
            Sends mission parameters back to ground control station as confirmation of receipt
            Resets controller to make sure mission is updaated across all objects

        */
        void unpack_change_mission_params(IridiumSBD modem, byte buf[]);
        
        /*
            unpack_force_mode_change - If a request is made to force mode change, do it
                buf: buffer that message data has been read into

            Reads data with a FORCE_MODE_CHANGE flag

            Expected byte stream:
            Byte    Content
            0       START BYTE
            1       FORCE_MODE_CHANGE
            2       enum value of new mode
            3       END BYTE
 
        */
        void unpack_force_mode_change(byte buf[]);

               

};

/* 
    determine_bearing - given a start point and a target point, determine the direction from the start point to the end point
        curr_lat: current latitude E7 (degrees)
        curr_lon: current longitude E7 (degrees)
        tar_lat: target latitude E7 (degrees)
        tar_lon: target longitude E7 (degrees)
        

    Direction is defined as degrees clockwise from North.  
    Great circles are used for the path from start to finsih.
*/
long determine_bearing(long curr_lat, long curr_lon, long tar_lat, long tar_lon);

/* 
    determine_error_distance - given a start point and a target point, determine the distance from the start point to the end point
        state: current vehicle state vector as per VEHICLE_STATE_VECTOR format
        wp: target waypoint
    
    Distances are in meters
    Great circles are used for the path from start to finsih.
*/
double determine_error_distance(long state[], Waypoint wp);

/*  Overloaded method header
    determine_error_distance - given a start point and a target point, determine the distance from the start point to the end point
        startLat: current latitude E7 (degrees)
        startLon: current longitude E7 (degrees)
        endLat: target latitude E7 (degrees)
        endLon: target longitude E7 (degrees)
        
    Distances are in meters.
    Great circles are used for the path from start to finsih.
*/
double determine_error_distance(long startLat, long startLon, long endLat, long endLon);

/* 
    determine_resultant_position - given a start point, bearing, and distance, calculate the resulting point and save to waypoint
        wp: a pointer to a waypoint struct to hold final aim point
        start: the starting point of the boat (usually the previous waypoint)
        bearing: the bearing, usually from the previous waypoint to the next waypoint
        distance: the distance to travel from start, along bearing, before setting the resulting point
    
    Great circles are used for the path from start to finsih.
*/
void determine_resultant_position(Waypoint *wp, Waypoint start, long bearing, long distance);

/* 
    manSwitch - A function to read whether a remote controller is present and requesting manual control
        channelInput: digital I/O pin to read manual controller requests off of 
        defaultValue: the default value to return if a channel reading can't be made 

    Returns bool: true puts boat in manual mode, false puts boat in autonomous mode

    If a controller is not detected, or no command is given, defaults into autonomous mode. 
*/
bool manSwitch(byte channelInput, bool defaultValue);


/*
    readChannel - reads a pwm signal in on a given pin, and returns pulse width in microseconds
        channelInput: digitial I/O pin to be reading off of
        minLimit: the minimum value to scale the controller output to
        maxLimit: the maximum value to scale the controller ouptut to 
        defaultValue: the value to return if a timeout or misread occurs

    Note that this function is necessary because on Arduino Due, pulseIn() does not work
    due to dynamic clock timing.

    This funciton is blocking, but does signal debouncing.

*/
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue);


#endif