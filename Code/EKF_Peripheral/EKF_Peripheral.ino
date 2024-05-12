#include <IQT_ANGLE_EKF.h>
#include <IQT_VELOCITY_EKF.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
#include <IQT_COMMUNICATIONS.h>
#include "Arduino_BMI270_BMM150.h"
#include <Wire.h>

// Theo DeGuzman
// Johns Hopkins University
// IQT24 Senior Design Project
// V2 Prototype Code
// Not for production use -- end user assumes responsibility for usage and testing of this code


/*  This script is the main function for the "GPS Peripheral" boards.

In the IQT24 V2 architecture, there are two GPS peripherals running on an Arduino Nano BLE Sense 33

The purpose of each of these peripherals is to track inertial and GPS-based vehicle state information.
This is their only purpose, and the intent is to be able to track these items even while the main board 
may be in some blocking subroutine.


EXTERNAL CONNECTIONS:

This board attached to the main board with 3 communications connections:
   - Main board serial (1 or 2) UART TX line to peripheral pin UART RX
   - Main board serial (1 or 2) UART RX line to peripheral pin UART TX
   - Main board digital output pin (any) to peripheal pin 2 (interrupt)

Note, the main board also has the ability to do a hardware reset on this board. 

This boards is attached to the GPS module (a NEO-M9N breakout board from SparkFun) with 2 communications connections:
   - The peripheral board pin A4 goes to GPS SDA (I2C data line)
   - The peripheral board pin A5 goes to GPS SCL (I2C clock line)



Power considerations for Arduino Nano BLE Sense 33:
   - This board operates on 3.3V, if the main board operates on 5 volts, a logic shifter is required
   - It is acceptable to power the GPS module out of the 3.3V pin on this board, if nothing else is powered from this


OPERATION: 

The operation is as follows:
   - The peripheral board loops through, constantly checking inertial attitude and GPS position
   - The main board drops pin 2 low and this activates a flag to send transmission


The UART output is a 35 byte string specified in IQT_COMMUNICATIONS.h

Known issues and current mitigations:
   1.  The GPS request subroutines (myGNSS.get...()) are blocking for a non-trivial amount of time.  
        - This means that IMU integrations are suspended.  This has been emprically shown not to hinder performance.
   2.  The initally developed multi-dimensional extended kalman filter (EKF) is not numerically stable.
        - A single-dimensional EKF has been implemented for angle estimates
        - The GPS data output is already filtered internally by the module
   3.  The tramission of data occurs inside of an interrupt service routine
        - This is generally bad practice and operates under the assumption that multiple requests will not be made sequentially
        - However, this has shown to be plenty reliable and robust in this application


*/


using namespace BLA;

// accleration and gyroscope variable initalization
float ax, ay, az = 0.05; 
float gx, gy, gz = 10;

// Extended Kalman filter setup
ANGLE_EKF pitch;
ANGLE_EKF roll;

// Values for keeping track of avarage orientation over a short time interval
double pitch_sum = 0.0;
double roll_sum = 0.0;
unsigned long imu_count = 0;
unsigned long pitch_update_time = 0;
unsigned long roll_update_time = 0;

// Global variable for maximum sample rate
float max_sample_rate = 0.0;

// Gyroscope-based metrics to detect rough seas
long gyro_sum = 0;
const int gyro_array_length = 60; // 1 min at .5 sec intervals
long gyro_activity_metric = 0;
int gyro_activity_index = 0; // 1 min at .5 sec intervals
long gyro_activity_array[gyro_array_length];

// GPS variables
SFE_UBLOX_GNSS myGNSS;  // GPS communication library
VELOCITY_EKF myVelocity;  // Simplified EKF for velocity 
unsigned long gps_update_time = 0;
const unsigned long gps_sample_period = 400;

// state vector of values
long state_vector[8];

// Message Buffer Variable
byte buf[PERIPHERAL_STATE_VECTOR_LENGTH];

void setup() {
  
  pinMode(13, OUTPUT); // LED indicator pin - alternates hi/low twice per second under normal operation
  
  Serial.begin(9600);  // serial communication to monitor output
  Serial1.begin(9600); // serial communication back to main board

  // initialize gyro activity array
  for (unsigned int i = 0; i < gyro_array_length; i++) {
    gyro_activity_array[i] = 0;
  }
  
  // EKF covariance matricies 
  BLA::Matrix<2, 2> imu_process_cov = {0.1, 0.01, 0.01, 10.0};
  BLA::Matrix<2, 2> imu_measurement_cov = {0.2, 0, 0, 0.04};

  while (!IMU.begin()) {}  // begin onboard IMU

  max_sample_rate = min(IMU.accelerationSampleRate(), IMU.gyroscopeSampleRate()); // set IMU sampling rate

  Wire.begin(); // initalize i2c connection for GPS

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX);                 //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  // attach data request interrupt
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(2, send_status, LOW);

  // Begin extended kalman filters
  pitch.begin(imu_process_cov, imu_measurement_cov);
  roll.begin(imu_process_cov, imu_measurement_cov);
  myVelocity.begin();

}

void loop() {

  // Collect and filter IMU data
  if (IMU.accelerationAvailable()) { IMU.readAcceleration(ax, ay, az);}
  if (IMU.gyroscopeAvailable()) {IMU.readGyroscope(gx, gy, gz);}

  // Update pitch
  BLA::Matrix<2, 1> pitch_measurement = {get_pitch(ax, ay, az), gy};
  pitch.step(pitch_measurement, (millis()-pitch_update_time)*(1.0E-3));
  pitch_sum += pitch.get_post_est()(0);
  pitch_update_time = micros();

  // Update roll
  BLA::Matrix<2, 1> roll_measurement = {get_roll(ax, ay, az), gx};
  roll.step(roll_measurement, (millis()-roll_update_time)*(1.0E-3));
  roll_sum += roll.get_post_est()(0);
  roll_update_time = micros();

  // Update gyro
  gyro_sum += sqrt(gx*gx + gy*gy + gz*gz);
  imu_count++;


  // sample GPS every gps_sample_period milliseconds (.4 second)
  if (millis() - gps_update_time > gps_sample_period) {
    
    // update gyro activity metric
    gyro_activity_metric += gyro_sum/imu_count;
    gyro_activity_metric -= gyro_activity_array[gyro_activity_index];
    gyro_activity_array[gyro_activity_index] = gyro_sum/imu_count;
    gyro_activity_index = (gyro_activity_index + 1) % gyro_array_length;


    // get filtered GPS coordinates and velocity
    myVelocity.step(myGNSS.getGroundSpeed(), myGNSS.getHeading(), myGNSS.getLatitude(), myGNSS.getLongitude(), myGNSS.getSpeedAccEst(), myGNSS.getHeadingAccEst());
    gps_update_time = millis();
   
    // store gps reading
    state_vector[0] = myVelocity.get_coords()(0);
    state_vector[1] = myVelocity.get_coords()(1);
    state_vector[2] = myVelocity.get_post_est()(0);
    state_vector[3] = myVelocity.get_post_est()(1);
    state_vector[7] = myGNSS.getPDOP(100);

    // store IMU readings
    state_vector[4] = (roll_sum/(float) imu_count) * 1E5;    
    state_vector[5] = (pitch_sum/ (float) imu_count) * 1E5;
    state_vector[6] = gyro_activity_metric;

    
    imu_count = 0; 
    roll_sum = 0;
    pitch_sum = 0;
    gyro_sum = 0;

    // switch indicator LED
    digitalWrite(13, !digitalRead(13));

    // convert state vector of longs into byte vector
    pack_gps_peripheral(state_vector, buf);

`   /*  Print statements for debugging  */
    // output number of sattelires for debug
    // Serial.print(myGNSS.getSIV());
    // Serial.print(":  ");
    
    // Serial.print(state_vector[0]/1E7);
    // Serial.print(", ");
    // Serial.print(state_vector[1]/1E7);
    // Serial.print(", ");
    // Serial.print(state_vector[2]/1E3);
    // Serial.print(", ");
    // Serial.print(state_vector[3]/1E5);
    // Serial.print(", ");
    // Serial.print(myVelocity.get_measurement()(1)/1E5);
    // Serial.print(", ");
    // Serial.print(state_vector[4]/1E5);
    // Serial.print(", ");
    // Serial.print(state_vector[5]/1E5);
    // Serial.print(", ");
    // Serial.print(state_vector[6]);
    // Serial.print(", ");
    // Serial.println(state_vector[7]);

  }
 
  // Delay to not overload the buffer - intednded to limit rate of IMU readings
  delay((long) 1000/max_sample_rate + 1);

}

// When signal sent from main board, send GPS peripheral packet back
// Probably bad practice to serial write in an interrupt
void send_status(){
  Serial1.write(buf, PERIPHERAL_STATE_VECTOR_LENGTH);
  delayMicroseconds(10000);
}

