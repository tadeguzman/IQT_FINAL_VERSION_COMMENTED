#include <ArduinoJson.h>
#include <SD.h>
#include <SPI.h>
#include <IQT_COMMUNICATIONS.h>
#include <IQT_vehicle.h>
#include <IQT_controller.h>


// Theo DeGuzman
// Johns Hopkins University
// IQT24 Senior Design Project
// V2 Prototype Code
// Not for production use -- end user assumes responsibility for usage and testing of this code


/*  This script is the main function for the "main" or "driver" board.

In the IQT24 V2 architecture, this runs on an Arduino Due

This board is the primary board: responsible for motor control, rc radio communciation, satellite communication, SD card managment, and battery monitoring.

EXTERNAL CONNECTIONS:

Each peripheral board attached is to the main board with 3 communications connections:
   - Main board serial (1 or 2) UART TX line to peripheral pin UART RX
   - Main board serial (1 or 2) UART RX line to peripheral pin UART TX
   - Main board digital output pin (any) to peripheal pin 2 (interrupt)
   - Main board attached to the hardware reset on each peripheral. 

The watchdog is connected with the following lines:
   - A digital output pin from a main board representing heatbeats and attaching to watchdog pin TIMER
   - A digital output pin from a main board representing hard reset request and attaching to watchdog pin HARD_RESET
   - A digital output pin from watchdog connecting to main board's reset pin  

Various sensors are connected and can be read on pins A0 through A7.

An SD card reader is connected through ICSP_MOSI, ICSP_MISO, ICSP_SCK, chip select (Pin 10), GND, and 5V

The satellite is connected through a MAX_3232 breakout, with TX3 attached to T1IN and RX3 attached to R1OUT

A RC radio tranciever is connected to pins RC_THRUST (ch 1), RC_STEER (ch 2), and RC_ACTIVE (ch 3).

The ESCs are attached to the board, using the pins specifed under //thrusters, below.

The relays are attached to the board, using the pins specified with _RELAY, below. 


Power considerations for Arduino Due:
   - This board operates on 3.3V, if some other board operates on 5 volts, a logic shifter is required
   - It is acceptable to power the SD Card module from this board, and the sepecified relays
        - Check power requirements before powering any 3.3 V or 5 V sensors from the JST connectios supplied off the board


OPERATION: 

The operation is as follows:
   - The main board reads mission parameters and waypoints off SD card and stores them
   - The main board loops through the following steps:
        1.  Check if in manual mode on last time step, if so, skip steps 2-4
        2.  Check GPS/INS peripherals for vehicle position, orientation, speed, etc. 
        3.  Estimate vehicle state and heading based on that information
        4.  Read battery values and other sensor data
        5.  Determine mode
        6.  Determine if a new waypoint has been reached
        7.  Execute appropriate motor and relay commands
        8.  Record vehicle and controller states back to SD card
        9.  Check for incoming satellite messages and send messages if appropriate
        10. Send out heartbeat signal 



Known issues and current mitigations:
   1.  modem.begin() in the IridiumSBD library is blocking without easy access to a timeout function
          Therefore, if the satellite is physically disconnected when modem.begin() is called in the setup, the sketch will hang and not progress
          The system IS otherwise resillient to losing physical connection with the satellite (and all other onboard components)
          Risk is mitigated through robust mechanical fastening and sound engineering pracitces -- but further exporation may be worthwhile
   2.  Cheap SD cards are not robust to repeated, regular read-writes
          Mitigated by purchasing more reliable, well-made SD cards
              One known case of misread from an SD card address rewritten many times
          Only writes back at <3 Hz now, instead of 10+ Hz
          Default mission implemented as backup (generally a return-to-home protocol)
    3. Heat disssipation in the system
          Ability to add temperature systems in the battery state vector is not currently linked to any procedures to start a fan or reduce ESC load
          Would be simple to add
    4. SD card overflow
          With a 32 GB SD card (max allowed size with library), should be able to run for ~ 2 years before overflowing
              However, there is no system in place to remove oldest files first, meaning behavior is undefined.
              This would be easy to add, as files are labeled by month, day, and hour
                  Another downside to this system, however, is that file names loop over the course of a new year


*/

/* -----------PIN CONNECTION DEFINITIONS-------------- */
#define RST_PIN 3
#define HEARTBEAT_PIN 4

// Sensors 
#define RST_GPS_1 24
#define INT_GPS_1 25
#define SER_GPS_1 Serial1

#define RST_GPS_2 26
#define INT_GPS_2 27
#define SER_GPS_2 Serial2

// Sat serial
#define SER_SAT Serial3

// thrusters
#define T1_PIN 12
#define T2_PIN 9
#define T3_PIN 11
#define T1_PIN_REVERSE 46
#define T2_PIN_REVERSE 48
#define T3_PIN_REVERSE 47
#define T1_RELAY 28
#define T2_RELAY 30
#define T3_RELAY 32

// payload
#define PAYLOAD_RELAY 34

// current sensors
#define NAV_AMP A0
#define T1_AMP A1
#define T2_AMP A2
#define T3_AMP A3
#define PAYLOAD_AMP A4
#define SOLAR_AMP A5

// other sensors
#define V_BATT A6
#define V_TEMP A7

// RC pins
#define RC_THRUST 7
#define RC_STEER 6
#define RC_ACTIVE 5

#define RING 4 // don't actually need ring bc uses a serial read instead of interrupt

/* -----------END PIN CONNECTION DEFINITIONS-------------- */


// Mission parameter hardcodings
#define EXIT_BROWNOUT 75 // exit brownout when voltage raised .75V above minimum (to protect battery)

// Global variables for information storage
long state1[8];
long state2[8];
long last_update = 0;
bool motor_e[3];
Sensor_enable sense_e[2];
Waypoint wps[28];
long temp_state[8];
long temp_bat[8];

// Global objects for vehicle operation
IridiumSBD modem(SER_SAT, -1, RING);
IQT_VEHICLE boat;
IQT_CONTROLLER brain;
Mission mission;
Mission default_mission;


void setup() {

  digitalWrite(RST_PIN, HIGH);

  /* -------------------- Setup a default mission fallback if SD card has issues -------------------------*/
  default_mission.current_waypoint = 0;
  default_mission.number_waypoints = 1;
  default_mission.time_at_current = 0;
  default_mission.cruise_speed = 4000;
  default_mission.steer_big_p_gain = 1E-5;
  default_mission.steer_small_p_gain = 1E-5;
  default_mission.steer_i_gain = 1E-6;
  default_mission.thrust_p_gain = 0.001;
  default_mission.transition_threshold = 30E5;
  default_mission.rough_sea = 1000000;
  default_mission.brownout_thresh = 2240; //22.7;
  default_mission.sat_update = 1800000;// 1800000; // 30 min updates
  default_mission.exit_safemode_time = 0; // 0 ms to exit safemode
  default_mission.lead_distance = 20;

  default_mission.motor_enabled[0] = true;
  default_mission.motor_enabled[1] = false;
  default_mission.motor_enabled[2] = true;

  default_mission.sensor_force_enable[0] = automatic;
  default_mission.sensor_force_enable[1] = automatic;

  default_mission.waypoints[0].latitude = 392529771;
  default_mission.waypoints[0].longitude = -764889736;
  default_mission.waypoints[0].radius = 1000; //m
  default_mission.waypoints[0].type = loitering;
  default_mission.waypoints[0].time_on_station = 100000;  
  
  // default_mission.waypoints[1].latitude = 393261210;
  // default_mission.waypoints[1].longitude = -765179121;
  // default_mission.waypoints[1].radius = 125; //m
  // default_mission.waypoints[1].type = navigation;
  // default_mission.waypoints[1].time_on_station = 0;  
  
  // default_mission.waypoints[2].latitude = 393326709;
  // default_mission.waypoints[2].longitude = -766180515;
  // default_mission.waypoints[2].radius = 125; //m
  // default_mission.waypoints[2].type = navigation;
  // default_mission.waypoints[2].time_on_station = 0;  
  
  // default_mission.waypoints[3].latitude = 393358907;
  // default_mission.waypoints[3].longitude = -766232657;
  // default_mission.waypoints[3].radius = 125; //m
  // default_mission.waypoints[3].type = navigation;
  // default_mission.waypoints[3].time_on_station = 0;  
  
  // default_mission.waypoints[4].latitude = 393330444;
  // default_mission.waypoints[4].longitude = -766224611;
  // default_mission.waypoints[4].radius = 125; //m
  // default_mission.waypoints[4].type = navigation;
  // default_mission.waypoints[4].time_on_station = 0;  
  
  // default_mission.waypoints[5].latitude = 393252890;
  // default_mission.waypoints[5].longitude = -766238183;
  // default_mission.waypoints[5].radius = 125; //m
  // default_mission.waypoints[5].type = loitering;
  // default_mission.waypoints[5].time_on_station = 100000;
  
  /* -------------------------------- END DEFAULT MISSION SETUP --------------------------------------*/
  

  // Pin initalizations
  Serial.begin(9600);
  Serial.println("Began Setup");
  SER_GPS_1.begin(9600);
  SER_GPS_2.begin(9600);
  SER_SAT.begin(19200);

  pinMode(RST_PIN, OUTPUT);
  pinMode(HEARTBEAT_PIN, OUTPUT);
  pinMode(T1_RELAY, OUTPUT);
  pinMode(T2_RELAY, OUTPUT);
  pinMode(T3_RELAY, OUTPUT);
  pinMode(PAYLOAD_RELAY, OUTPUT);
  pinMode(RST_GPS_1, OUTPUT);
  pinMode(INT_GPS_1, OUTPUT);
  pinMode(RST_GPS_2, OUTPUT);
  pinMode(INT_GPS_2, OUTPUT);
  pinMode(RC_THRUST, INPUT);
  pinMode(RC_STEER, INPUT);
  pinMode(RC_ACTIVE, INPUT);
  pinMode(T1_PIN, OUTPUT);
  pinMode(T2_PIN, OUTPUT);
  pinMode(T3_PIN, OUTPUT);

  digitalWrite(RST_PIN, HIGH);
  digitalWrite(HEARTBEAT_PIN, HIGH);
  digitalWrite(T1_RELAY, HIGH);
  digitalWrite(T2_RELAY, HIGH);
  digitalWrite(T3_RELAY, HIGH);
  digitalWrite(PAYLOAD_RELAY, HIGH);
  digitalWrite(RST_GPS_1, HIGH);
  digitalWrite(INT_GPS_1, HIGH);
  digitalWrite(RST_GPS_2, HIGH);
  digitalWrite(INT_GPS_2, HIGH);

  
  // SD Card initalization (give it 5 tries - because it doesn't always work first time)
  for (int i = 0; i < 5; i++) {
    if (SD.begin(10)) {
      break;
    }
    Serial.println(F("Failed to initialize SD library, retrying"));
    delay(1000);
  }

  // load a mission from SD card, use default mission if cannot be loaded
  if (loadMission(&mission)) {
    mission = default_mission; // go to default if loading from sd card doesn't work
    Serial.println("Using default mission");
  } else {
    Serial.println("Using SD card mission");
    for (int i = 0; i < mission.number_waypoints; i++) {
      if (loadWaypoints(i, &mission)) {
        mission.number_waypoints = i + 1;
        break;
      }

      // debug outputs
      Serial.print(mission.waypoints[i].latitude);
      Serial.print(", ");      
      Serial.print(mission.waypoints[i].longitude);
      Serial.print(", ");      
      Serial.print(mission.waypoints[i].radius);
      Serial.print(", ");      
      Serial.println(mission.waypoints[i].time_on_station);
    }
  }

  // If there is no voltage sensor attached, disable brownout mode
  mission.brownout_thresh = 0;

  // debug outputs
  Serial.println(mission.number_waypoints);
  Serial.println(mission.sat_update);
  Serial.println(mission.waypoints[mission.number_waypoints].time_on_station);
  Serial.println("Sat Begin");

  // Initalize Satellite modem
  modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);
  modem.begin(); // Known issue 1
  modem.markRingFalse();
  
  // Initalize vehicle and controller objects, initalize servos
  boat.begin(&mission, EXIT_BROWNOUT, RC_ACTIVE, RST_PIN);
  brain.begin(T1_PIN, T2_PIN, T3_PIN, T1_PIN_REVERSE, T2_PIN_REVERSE, T3_PIN_REVERSE, T1_RELAY, T2_RELAY, T3_RELAY, PAYLOAD_RELAY, RC_STEER, RC_THRUST, mission);
  
  // check satellite signal quality
  int signalQuality = -1;
  modem.getSignalQuality(signalQuality);
  Serial.print("Signal quality is ");
  Serial.println(signalQuality);

  // reset both GPS/INS peripherals
  digitalWrite(RST_GPS_1, LOW);
  digitalWrite(RST_GPS_2, LOW);
  delay(100);
  digitalWrite(RST_GPS_1, HIGH);
  digitalWrite(RST_GPS_2, HIGH);
  
  Serial.println("Completed Setup");

}

void loop() {

  // Ignore gps read if last in manual mode
  long timer = millis();
  if (boat.get_mode() != manual) {

    // Check GPS/INS peripherals for data
    gps_poll(INT_GPS_1, SER_GPS_1, state1);
    gps_poll(INT_GPS_2, SER_GPS_2, state2);

    boat.determine_state(state1, state2, RST_GPS_1, RST_GPS_2);  // vehicle state estimation
    boat.determine_heading();
    boat.read_battery();
  }

  // shift to correct mode of operation and check if new waypoint has been reached
  boat.determine_mode(); 
  boat.check_waypoint_progression();
  

  // load vehicle physical state, power state, and electronics states into local arrays
  bool motor_enabled[3];
  boat.get_state(temp_state);
  boat.get_battery(temp_bat);
  boat.get_motor_enabled(motor_enabled);
  
  // execute command over vehicle (i.e. control motors and relays)
  brain.command(boat.get_mode(), temp_state, boat.get_target(), temp_bat, motor_enabled, boat.get_est_heading(), boat.get_target_speed());

  // SD card recording
  //    - Ignore SD recording in manual mode
  if (boat.get_mode() != manual) {
    struct tm t;
    modem.getSystemTime(t);
    boat.record_state(t);
    brain.record_state(t);
  }

  // broadcast state update if appropriate, and check for incoming messages
  boat.broadcast_satellite(modem, state1, state2, brain.get_steer(), brain.get_thrust(), brain.get_trim());
  
  // check if system needs a reset
  boat.check_for_reset();

  //send heartbeat
  digitalWrite(HEARTBEAT_PIN, !digitalRead(HEARTBEAT_PIN));

  // DEBUG OUTPUT MESSAGES

  // Serial.print(boat.get_waypoint());
  // Serial.print(", ");
  // Serial.print("  GPS: ");
  // Serial.print(mission.current_waypoint);
  // Serial.print(", ");  
  // Serial.print(boat.dist_from_wp());
  // Serial.print(", ");
  // Serial.print(boat.get_est_heading()/1.0E5);
  // Serial.print(", ");
  
  // Serial.print(temp_state[0]/1.0E7);
  // Serial.print(", "); 
  // Serial.print(temp_state[1]/1.0E7);
  // Serial.print(", ");
  // Serial.print(boat.get_mode());
  // Serial.print(", ");
  // Serial.print(temp_state[2]/1E3);
  // Serial.print(", ");  
  // Serial.print(temp_state[3]/1E5);
  // Serial.print(", "); 
  // Serial.print(temp_state[7]);

  // Serial.print(boat.dist_from_wp());
  // Serial.print(", ");
  
  // long ctrl[7];
  // brain.get_control_state(ctrl);
  // Serial.print(ctrl[0]/1E3); // tar speed
  // Serial.print(", ");  
  // Serial.print(ctrl[1]/1E3); // err speed 
  // Serial.print(", ");  
  // Serial.print(ctrl[2]/1.0E5); // tar bearing
  // Serial.print(", ");  
  // Serial.print(ctrl[3]/1.0E5); // error bearing
  // Serial.print(", ");  
  // Serial.print(ctrl[4]); // steer
  // Serial.print(", ");  
  // Serial.print(ctrl[5]); // thrust
  // Serial.print(", ");  
  // Serial.print(ctrl[6]); // trim
  // Serial.print(", ");   
  
  // Serial.print("  BAT: ");
  // Serial.print(temp_bat[0]/1.0E2);
  // Serial.print(", ");
  // Serial.print(temp_bat[1]/1.0E2);
  // Serial.print(", ");
  // Serial.print(temp_bat[2]/1.0E2);
  // Serial.print(", ");
  // Serial.print(temp_bat[3]/1.0E2);
  // Serial.print(", ");


  // Serial.print("  MODE: ");
  // Serial.print(boat.get_mode());
  // Serial.println();

}

