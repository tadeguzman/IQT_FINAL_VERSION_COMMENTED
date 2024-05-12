#define TIMER 2
#define HARD_RESET 3
#define SELF_RESET 8
#define OTHER_RESET 9


#define RESET_ALL_TIME 12.096E8 // 2 weeks
#define CYCLE_FAIL_TIME 120000 // 2 mins

// Theo DeGuzman
// Johns Hopkins University
// IQT24 Senior Design Project
// V2 Prototype Code
// Not for production use -- end user assumes responsibility for usage and testing of this code


/*  This script is the main function for the "Watchdog" boards.

In the IQT24 V2 architecture, there is a single watchdog policing the main board's operation, and takes the form of Arduino Nano IOT

The purpose of the watchdog is to reset the main board if the sketch hangs for an unexpected reason, for unacceptably long time.

An example of this might be if an Iridium short-burst satellite message is interrupted mid-transmission by some obstacle, and connection 
cannot be reestablished with the network right-away.


EXTERNAL CONNECTIONS:

This board attached to the main board with 3 communications connections:
   - A digital output pin from a main board representing heatbeats and attaching to watchdog pin TIMER
   - A digital output pin from a main board representing hard reset request and attaching to watchdog pin HARD_RESET
   - A digital output pin from watchdog connecting to main board's reset pin

   - Also, a digital output pin from watchdog, looped back to its reset pin


Power considerations for Arduino Nano IOT:
   - This board operates on 3.3V, if the main board operates on 5 volts, a logic shifter is required

OPERATION: 

The operation is as follows:
   - The board loops through, constantly checking that overall time in operation and time since last heartbeat do not exceed thresholds,
        and that a hardware reset has not be requested.
   - Interrupts track heartbeats and look out for hard resets in the background

Known issues and current mitigations:
   1.  Hypothetically, if this board goes out of operation, the default behavior is for all the other boards to continue working
          However, the main board will not be able to hardware reset itself anymore. The mitigation for this is a programmed software
          that will activate in the case a failed hardware restart.

*/

volatile long last_heartbeat = 0; // time in milliseonds of last heartbeat
volatile bool hard_rst = false; // flag to check for hard reset

void setup() {

  // General note: when a board's reset pin is pulled down to ground (LOW) is when it resets
  
  digitalWrite(SELF_RESET, HIGH);
  
  pinMode(TIMER, INPUT_PULLUP);       // interrupt pin to track heartbeats
  pinMode(HARD_RESET, INPUT_PULLUP);  // interrupt pin to note a reset request from main board
  pinMode(SELF_RESET, OUTPUT);        // pin to hardware reset the watchdog itself
  pinMode(OTHER_RESET, OUTPUT);       // pin to hardware reset the main board
  
  digitalWrite(OTHER_RESET, HIGH);    

  // attaching interrupts
  attachInterrupt(TIMER, heartbeat, CHANGE);
  attachInterrupt(HARD_RESET, hard_reset, LOW);

}

void loop() {

  // reset if heartbeat is missed
  if (millis() - last_heartbeat > CYCLE_FAIL_TIME) {
    digitalWrite(OTHER_RESET, LOW);
    delay(1000);
    digitalWrite(OTHER_RESET, HIGH);
    last_heartbeat = millis();
  }

  // reset if hard reset is requested
  if (hard_rst) {
    digitalWrite(OTHER_RESET, LOW);
    delay(1000);
    digitalWrite(OTHER_RESET, HIGH);
    hard_rst = false;
  }

  // reset self and main board after some amount of time
  if (millis() > RESET_ALL_TIME) {
    digitalWrite(OTHER_RESET, LOW);
    delay(1000);
    digitalWrite(SELF_RESET, LOW);
    delay(1000);
    digitalWrite(SELF_RESET, HIGH);
    digitalWrite(OTHER_RESET, HIGH);
    last_heartbeat = millis();
  }

  delay(100);
}

// flag heartbeat
void heartbeat() {
  last_heartbeat = millis();
  digitalWrite(13, !digitalRead(13)); // indicator light
}

// flag hard reset
void hard_reset() {
  hard_rst = true;
}

