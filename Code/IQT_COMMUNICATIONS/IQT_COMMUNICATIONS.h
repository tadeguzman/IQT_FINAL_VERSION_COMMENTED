#ifndef IQT_COMMUNICATIONS_H
#define IQT_COMMUNICATIONS_H

#include "Arduino.h"
#include <ArduinoJson.h>
#include <SD.h>
#include <SPI.h>

// Theo DeGuzman
// Johns Hopkins University
// IQT24 Senior Design Project
// V2 Prototype Code
// Not for production use -- end user assumes responsibility for usage and testing of this code

/* This is library contains a number of constants and values used to define internal
   and external communication protocols across all IQT libraries*/


//  ENUMERATIONS  //

/* Defines different types of modes
    Cruise - opertation with the intent of traveling from one place to another, payload not powered
    Loiter - opertation with the intent of staying in one place, playload powered, motors de-energized
    Loiter_cruise - operation with intent of returning to a loiter point after drifting away, payload powered
    Power_safe - operation when battery levels are below a certain level, payload and motors off
    Turbulence_safe - operation when boat experiences excessive turbulence, payload on (optionally), motors off
    Manual - opertaion in close range to an operator, allows for direct remote control, payload off
*/
enum Mode {cruise, loiter, loiter_cruise, power_safe, turbulence_safe, manual};

/* Defines types of waypoints
    Navigation - a waypoint intended to direct a boat from its previous waypoint to this location
    Loitering - a waypoint used as a position for the boat to loiter and activate sensor payload
*/
enum WaypointType {navigation, loitering};

/* Allows the user to force a GPS/INS peripheral to be ignored if found in error.
    Automatic - built-in data checking determines if GPS is healthy or not
    Force_enable - force a sensor to read as correct, regardless of its reading
    Force_disable - force a sensor to read as incorrect, regardless of its reading
*/
enum Sensor_enable {automatic, force_enable, force_disable};




//  STRUCTS  //

/* Waypoints: describe the type of path the boat is supposed to take
    latitude - latitude of the point in degrees E7
    longitude - longitude of the point in degrees E7
    radius - the radius of the waypoint in meters (double)
                - for navigation waypoints, this is how close the boat has to get before
                    progressing to the next waypoint
                - for loitering waypoints, this is used for how close the boat aims and 
                    allows itself to drift before correcting
    type - type of waypoint, as described to in the enumeration above
    time_on_station - for loitering waypoints, this determines how long the boat has to stay
                        within the specified radius before progressing to the next waypoint (milliseconds)
*/
typedef struct {
    long latitude;  // deg E7
    long longitude; // deg E7
    double radius; // m

    WaypointType type;
    long time_on_station; // seconds
} Waypoint;


/* MISSION: describe the control laws and threshold of the boat, and stores all the waypoints

    PATH PLANNING:
    waypoints - a list of waypoints (max 28, due to communication limits of satellite)
    current_waypoint - the index of the current waypoint in the waypoint list
    number_waypoints - the total number of waypoints defined in the list
    time_at_current - the total amount of time spent in the radius of the current waypoint
    cruise_speed - the target speed for the boat in cruise or loiter cruise mode (m/s E2)

    MOTOR CONTROLS:
    steer_big_p_gain - proportional control gain of boat when the heading error is large
    steer_small_p_gain - proportional control gain of boat when the heading error is small
    steer_i_gain - post-normalization integral constant gain of boat (only active when heading error small)
    thrust_p_gain - adjustment constant of boat speed (only active when heading error small)
    transition_threshold - value in degrees E5 at which a heading error is considered "large"
    motor_enabled - a bool representing whether or not the motor is enabled; would allow remote disabling of certain motors

    SAFE MODES:
    rough_sea - the unitless roughness index at which the boat considers itself in turbulent conditions
    brownout_thresh - the voltage in V E2 at which the battery level is considered too low, so the system will enter safe mode
    exit_safemode_time - the amount of time in milliseconds which a boat must be in a stable state before exiting a safe mode

    GUIDANCE:
    lead_distance - the distance for the "aim-point" ahead of the boat's current position, along the desired path

    COMMUNICATION:
    sat_update - time interval in milliseconds at which the boat will send an update to the satellite system

    SENSORS:
    sensor_force_enable - keeps track of the Sensor_enable state for each GPS/INS peripheral

*/
typedef struct {
    Waypoint waypoints[28];
    int current_waypoint;
    int number_waypoints;
    unsigned long time_at_current;
    unsigned long cruise_speed;

    float steer_big_p_gain;
    float steer_small_p_gain;
    float steer_i_gain;
    float thrust_p_gain;
    long transition_threshold;
    bool motor_enabled[3]; // NOT FULLY IMPLEMENTED

    unsigned long rough_sea;
    unsigned long brownout_thresh;
    unsigned long exit_safemode_time;
    
    long lead_distance;

    unsigned long sat_update;
    Sensor_enable sensor_force_enable[2];  // NOT FULLY IMPLEMENTED

} Mission;


// BYTE AND MESSAGE TYPE DECLARATIONS

#define START_BYTE 0xFF
#define END_BYTE 0xFE

// Internal Communications
#define PERIPHERAL_STATE_VECTOR 0xAA 
#define PERIPHERAL_STATE_VECTOR_LENGTH 35

// Vehicle to Ground Control Station Communication
#define VEHICLE_STATE_VECTOR 0xAB
#define VEHICLE_STATE_VECTOR_LENGTH 35

#define VEHICLE_BATTERY_VECTOR 0xAB
#define VEHICLE_BATTERY_VECTOR_LENGTH 35

#define CONTROLLER_STATE 0xAC
#define CONTROLLER_STATE_LENGTH 19

#define WAYPOINT_LIST 0xAD

#define WAYPOINT 0xAE
#define WAYPOINT_LENGTH 23

#define MODE_CHANGE 0xAF
#define MODE_CHANGE_LENGTH 11

#define WAYPOINT_REACHED 0xBA
#define WAYPOINT_REACHED_LENGTH 27

#define MISSION_PARAMS 0xCB
#define MISSION_PARAMS_LEN 59


// Ground Control Station to Vehicle Communication
#define REQUEST_WAYPOINT_LIST 0xBB

#define REQUEST_WAYPOINT 0xBC

#define CHANGE_WAYPOINT 0xBD

#define GET_MISSION_PARAMS 0xBE

#define CHANGE_MISSIONS_PARAMS 0xBF

#define FORCE_MODE_CHANGE 0xCA

#define RING_RECIEVED 0XBC




/*
    pack_gps_peripheral
        state: the vehicle state (array of longs) to convert into a byte buffer
        buf: the storage place for the resulting bytes

    pack_vehicle_state_vector
        state: the vehicle state (array of longs) to convert into a byte buffer
        buf: the storage place for the resulting bytes
        *** basically the same as pack_gps_peripheral, but done by the main board 
            instead of peripheral

    unpack_gps_peripheral
        state: the storage place for the resulting vehicle state longs
        buf: string of bytes representing a vehicle state


    BYTE FORMAT:

    Byte    Content
    0       START BYTE
    1       PERIPHERAL_STATE_VECTOR/VEHICLE_STATE_VECTOR
    2-5     Long - Current latitude E7 (deg)
    6-9     Long - Current longitude E7 (deg)
    10-13   Long - Current speed E3 (m/s)
    14-17   Long - Current heading E5 (deg from N)
    18-21   Long - Current pitch E5 (deg)
    22-25   Long - Current roll E5 (deg)
    26-29   Long - Turbulence value (unitless)
    30-33   Long - Position dilution of precision - PDOP E2 (scale 0-10)
    34      END BYTE

*/
void pack_gps_peripheral(long state[], byte buf[]);
void unpack_gps_peripheral(long state[], byte buf[]);
void pack_vehicle_state_vector(long state[], byte buf[]);

/*
    pack_vehicle_battery_vector
        state: the vehicle battery (array of longs) to convert into a byte buffer
        buf: the storage place for the resulting bytes

    This will be kinda set up based on however battery state vector is defined.
    The sample format is recommended and shown below, but highly dependent on
    where various sensors are plugged into the board (which analog read pin) 

    BYTE FORMAT:

    Byte    Content
    0       START BYTE
    1       VEHICLE_BATTERY_VECTOR
    2-5     Long - AO reading (Battery voltage in V E2)
    6-9     Long - A1 reading (Battery output current in A E2)
    10-13   Long - A2 reading (Solar output current in A E2)
    14-17   Long - A3 reading (Payload output current in A E2)
    18-21   Long - A4 reading (Automatic ilge pump output current in A E2)
    22-25   Long - A5 reading (ESC1 output current in A E2)
    26-29   Long - A6 reading (ESC2 output current in A E2)
    30-33   Long - A7 reading (Temperature sensor in degrees C E2)
    34      END BYTE
*/
void pack_vehicle_battery_vector(long state[], byte buf[]);

/*
    pack_controller_state_vector
        steer: the steering command for the boat
        trust: the thrust command for the boat
        trim: the current trim setting for the boat
        error: the current amount of heading error as calculated
        buf: the storage place for the resulting bytes

    BYTE FORMAT:

    Byte    Content
    0       START BYTE
    1       CONTROLLER_STATE
    2-5     Long - steering [-100, 100]
    6-9     Long - thrust [0, 100]
    10-13   Long - trim [-50, 50]
    14-17   Long - error [-180E5, 180E5]
    18      END BYTE
*/
void pack_controller_state_vector(int steer, int thrust, int trim, int error, byte buf[]);

/*
    pack_waypoint_list
        waypoints: waypoint list
        buf: the storage place for the resulting bytes
        num_wps: number of waypoints to send (maximum 28)

    ** the size of this transmission is not determined until runtime

    BYTE FORMAT: consider i = [0, num_wps)

    Byte    Content
    0       START BYTE
    1       WAYPOINT_LIST
    2       Byte - num_wps
    i*9+3 to i*9+6  Long - waypoint latitude in degrees E7
    i*9+7 to i*9+10 Long - waypoint longitude in degrees E7
    i*9+11          Long - waypoint type {loiter, cruise}
    (i+1)*9+3       END BYTE
*/
void pack_waypoint_list(Waypoint waypoints[], byte buf[], unsigned long num_wps);

/*
    pack_waypoint
        waypoint: single waypont to pack
        buf: the storage place for the resulting bytes

    BYTE FORMAT: 

    Byte    Content
    0       START BYTE
    1       WAYPOINT
    2-5     Long - waypoint latitude (degrees E7)
    6-9     Long - waypoint longitude (degrees E7)
    10-13   Long - radius (m)
    14-17   Long - type
    18-21   Long - time on station (millis)
    22      END BYTE
*/
void pack_waypoint(Waypoint wp, byte buf[]);

/*
    pack_mode_change
        last_mode: the previous vehicle mode
        mode: New mode to be changed to
        buf: the storage place for the resulting bytes

    BYTE FORMAT: 

    Byte    Content
    0       START BYTE
    1       MODE_CHANGE
    2-5     Long - previous mode
    6-9     Long - current mode
    10      END BYTE
*/
void pack_mode_change(int last_mode, int mode, byte buf[]);


/*
    pack_waypoint_reached
        last_lat: last latitude (deg E7)
        last_lon: last longitude (deg E7)
        last_type: last waypoint type
        lat: new latitude (deg E7)
        lon: new longitude (deg E7)
        type: new waypoint type
        buf: the storage place for the resulting bytes

    BYTE FORMAT:

    Byte    Content
    0       START BYTE
    1       WAYPOINT_REACHED
    2-5     Long - last latitude (deg E7)
    6-9     Long - last longitude (deg E7)
    10-13   Long - last waypoint type
    14-17   Long - new latitude (deg E7)
    18-21   Long - new longitude (deg E7)
    22-25   Long - new waypoint type
    26      END BYTE
*/
void pack_waypoint_reached(long last_lat, long last_lon, int last_type, long lat, long lon, int type, byte buf[]);

/*
    pack_mission_params
        mission: the mission who's parameters are to convert into a byte buffer
        buf: the storage place for the resulting bytes

    BYTE FORMAT:

    Byte    Content
    0       START BYTE
    1       MISSION_PARAMS
    2-5     Long - current waypoint
    6-9     Long - number of waypoints
    10-13   Long - time at current waypoint (milliseconds)
    14-17   Long - cruising speed
    18-21   Long - big_steer_p_gain
    22-25   Long - small_steer_p_gain
    26-29   Long - steer_i_gain
    30-33   Long - thrust_p_gain
    34-37   Long - heading error transition threshold
    38-41   Long - rough sea threshold
    42-45   Long - brownout threshold
    46-49   Long - satillite update interval
    50-53   Long - exit safemode time 
    54-57   Long - lead distance
    58      END BYTE
*/
void pack_mission_params(Mission *mis, byte buf[]);


/*
    loadMission - load the mission parameters out of an SD card
        mission: a pointer to the mission struct to load the mission parameters into
*/
int loadMission(Mission *mission);

/*
    loadWaypoints - a waypoint out of an SD card
        wp_num: the number of the waypoint to read-in
        mission: a pointer to the mission struct which contains the list to load the waypoints into
*/
int loadWaypoints(int wp_num, Mission *mission);

/*
    saveMission - save the mission parameters to an SD card
        mission: a pointer to the mission struct with the mission parameters to save
*/
int saveMission(Mission *mission);

/*
    saveWaypoint - save a waypoint to the SD card
        wp_num: the number of the waypoint to save
        mission: a pointer to the mission struct which contains the list of waypoints to save
*/
int saveWaypoint(int wp_num, Mission *mission);

/*
    gps_poll: from the main board, request and read a message from a gps peripheral
        int_pin: the number of the interrupt pin to activate the desired peripheral
        port: the serial connection to expect a response on
        state: vector of longs to write the recieved message into

    The message recieved is of type PERIPHERAL_STATE_VECTOR
*/
void gps_poll(int int_pin, Stream &port, long state[]);



// NOTE: for push_long_into_buffer and pull_long_out_of_buffer, I realized I could probably
// just write a long into position start_index and read one out normally, becuase the
// normal behavior is to just overwrite or overread the following bytes.  However, this is 
// probably better practice, but certainly less efficient in code (and also probably in time)

// take a long (val) and put it into buffer (buf), starting at position (start_index)
void push_long_into_buffer(long val, int start_index, byte buf[]);
// Return a long from four bytes of buffer (buf), starting at position (start_index)
long pull_long_out_of_buffer(int start_index, byte buf[]);



#endif