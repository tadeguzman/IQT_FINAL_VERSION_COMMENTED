#include "Arduino.h"
#include "IQT_vehicle.h"

// Theo DeGuzman
// Johns Hopkins University
// IQT24 Senior Design Project
// V2 Prototype Code
// Not for production use -- end user assumes responsibility for usage and testing of this code

/*
    IQT_Vechicle

    This class holds all the vehicle information, determines vehicle states, and runs the guidance algorithms
*/


IQT_VEHICLE::IQT_VEHICLE(){
    return;
}

void IQT_VEHICLE::begin(Mission *mission, unsigned long exit_brown_out, int rc_pin, int reset_pin){
        
        this->mission = mission;
        this->mode = cruise;
        this->mission->sensor_force_enable[0] = automatic;
        this->mission->sensor_force_enable[1] = automatic;
        this->mission->motor_enabled[0] = true;
        this->mission->motor_enabled[1] = false;
        this->mission->motor_enabled[2] = true;
        

        this->update_interval = mission->sat_update;
        this->rough_sea_thresh = mission->rough_sea;
        this->brown_out_thresh = mission->brownout_thresh;
        this->exit_brown_out_thresh = exit_brown_out;
        this->exit_safe_mode_time = mission->exit_safemode_time;
        this->time_mode_change = 0;
        this->last_waypoint_check = 0;

        this->rc_mode = rc_pin;
        this->last_satellite_broadcast = 0;
        this->reset_pin = reset_pin;

        this->last_periph_reset = 0;

}

void IQT_VEHICLE::record_state(tm t){
    char date[12];
    char time[8];
    sprintf(time, "%02d:%02d:%02d", t.tm_hour, t.tm_min, t.tm_sec);
    sprintf(date, "cl%02d%02d%02d.csv", (t.tm_mon + 1) % 100, (t.tm_mday) % 100, (t.tm_hour) % 100);

    File myFile = SD.open(date, FILE_WRITE);     
    // if the file opened okay, write to it:
    if (myFile) 
    {   
        myFile.print(time);
        myFile.print(","); 
        myFile.print(millis()); 
        myFile.print(","); 
        myFile.print(this->mode);
        myFile.print(",");

        myFile.print(this->est_heading);
        myFile.print(",");
        myFile.print(this->last_lat);
        myFile.print(",");
        myFile.print(this->last_long);
        myFile.print(",");

        for (int i = 0; i < 8; i++){
            myFile.print(this->observed_state[i]); 
            myFile.print(","); 
        }
        for (int i = 0; i < 8; i++){
            myFile.print(this->battery_state[i]); 
            myFile.print(","); 
        }
        
        myFile.print(this->mission->current_waypoint);
        myFile.print(",");
        myFile.print(this->mission->time_at_current);
        myFile.print(",");
        myFile.print(this->mission->motor_enabled[0]);
        myFile.print(",");
        myFile.print(this->mission->motor_enabled[1]);
        myFile.print(",");
        myFile.print(this->mission->motor_enabled[2]);
        myFile.print(",");
        myFile.print(this->mission->sensor_force_enable[0]);
        myFile.print(",");
        myFile.print(this->mission->sensor_force_enable[1]);
        myFile.print(",");

        myFile.println();
        myFile.close();
    } 
    else 
    {
        Serial.println("error opening csv.txt");
    }
}

void IQT_VEHICLE::determine_state(long s1[], long s2[], int g1, int g2) {
    bool s1_on = true;
    bool s2_on = true;

    // error checking on peripheral 1
    if ((s1[7] > 9000 || s1[7] < 1 || (s1[0] <= 5 && s1[0] >= -5) || (s1[1] <= 5 && s1[1] >= -5) || this->mission->sensor_force_enable[0] == force_disable) && this->mission->sensor_force_enable[0] != force_enable ) {
        s1_on = false;
        
        if (millis() - this->last_periph_reset > 10000) {
            digitalWrite(g1, LOW);
            delay(500);
            digitalWrite(g1, HIGH);
            this->last_periph_reset = millis();
        }
        
    } 

    // error checking on peripheral 2
    if ((s2[7] > 9000 || s2[7] < 1 || (s2[0] <= 5 && s2[0] >= -5) || (s2[1] <= 5 && s2[1] >= -5) || this->mission->sensor_force_enable[1] == force_disable) && this->mission->sensor_force_enable[1] != force_enable ) {
        s2_on = false;

        if (millis() - this->last_periph_reset > 10000) {
            digitalWrite(g2, LOW);
            delay(500);
            digitalWrite(g2, HIGH);
            this->last_periph_reset = millis();

        }
    } 

    // if neither periperal seems healthy, take info from both
    if (!s1_on && !s2_on) {
        s1_on = true;
        s2_on = true;
    }

    // if both peripherals healthy
    if (s1_on && s2_on) {
        for (int i = 0; i < 4; i++) {
            double total = 20000.0 - s1[7] -s2[7];
            double coeff_1 = (10000.0 - s1[7])/total;
            double coeff_2 = (10000.0 - s2[7])/total;
            this->observed_state[i] = s1[i] * coeff_1 + s2[i] * coeff_2;
        }
        for (int i = 4; i < 7; i++) {
            this->observed_state[i] = (long) ((s1[i] + s2[i]) / 2.0 );
        }
        this->observed_state[7] = s1[7] * (10000.0 - s1[7])/(20000.0 - s1[7] -s2[7]) + s2[7] * (10000.0 - s2[7])/(20000.0 - s1[7] -s2[7]);
        return;

    // if only peripheral 1 is healthy
    } else if (s1_on) {
        for (int i = 0; i < 8; i++) {
            this->observed_state[i] = s1[i];
        }

    // if only peripheral 2 is healthy
    } else {
        for (int i = 0; i < 8; i++) {
            this->observed_state[i] = s2[i];
        }
    }

    return;
}

void IQT_VEHICLE::determine_heading() {
    if (this->observed_state[0] != this->last_lat || this->observed_state[1] != this->last_long) {
        if (this->observed_state[2] > 750) {
            this->est_heading = determine_bearing(this->last_lat, this->last_long, this->observed_state[0], this->observed_state[1]);            
        } else {
            this->est_heading = this->observed_state[3];
        }
        this->last_lat = this->observed_state[0];
        this->last_long = this->observed_state[1];
    }
    return;
}


void IQT_VEHICLE::read_battery(){ // TODO: fix this
    for (int i = 0; i < 8; i++){
        this->battery_state[i] = 0;
    }
    this->battery_state[0] = long(analogRead(A0)*(3.3/1023)*((6.6+36.66)/6.6)* 100); // overall batt in Volts 
    this->battery_state[1] = long(((analogRead(A1)*(3.3/1023))*-13.365 + 19.761) * 100); // overall solar amps in Amps E2
    this->battery_state[2] = long(((analogRead(A2)*(3.3/1023))*-66.825 + 98.807) * 100); // overall m1 amps in Amps E2
    this->battery_state[3] = long(((analogRead(A3)*(3.3/1023))*-66.825 + 98.807) * 100); // overall m2 amps in Amps E2
    this->battery_state[4] = long(analogRead(A4));
    this->battery_state[5] = long(analogRead(A5));
    this->battery_state[6] = long(analogRead(A6));
    this->battery_state[7] = long(analogRead(A7));
    
}

void IQT_VEHICLE::determine_mode(){
    Mode current_mode = this->mode;

    // Manual mode activation
    if (manSwitch(this->rc_mode, false)) {
        this->mode = manual;
        return;
    }

    // If in a safemode:
    //    Provides criteria to exit
    //    If the boat exits safe mode, defines which mode to enter
    if ((current_mode == turbulence_safe || current_mode == power_safe) && (millis() - this->time_mode_change) <  this->exit_safe_mode_time) {
        if (this->observed_state[6] > this->rough_sea_thresh || this->battery_state[0] < this->brown_out_thresh ) {
            this->time_mode_change = millis();
        }
        return;
    }
    else if (current_mode == turbulence_safe) {
        if (this->observed_state[6] <  this->rough_sea_thresh && this->mission->waypoints[this->mission->current_waypoint].type == navigation) {
            this->mode = cruise;
            this->last_satellite_broadcast = 0; // force satellite update
        } else if (this->observed_state[6] <  this->rough_sea_thresh && this->mission->waypoints[this->mission->current_waypoint].type == loitering) {
            this->mode = loiter;
            this->last_satellite_broadcast = 0; // force satellite update

        }
        return;
    } else if (current_mode == power_safe) {
        if (this->battery_state[0] >  this->brown_out_thresh + this->exit_brown_out_thresh && this->mission->waypoints[this->mission->current_waypoint].type == navigation) {
            this->mode = cruise;
            this->last_satellite_broadcast = 0; // force satellite update

        } else if (this->battery_state[0] > this->brown_out_thresh + this->exit_brown_out_thresh && this->mission->waypoints[this->mission->current_waypoint].type == loitering) {
            this->mode = loiter;
            this->last_satellite_broadcast = 0; // force satellite update

        }
        return;
    } 

    // If not in a safemode:
    //    Checks it does not need to be in a safe mode
    //    Decides what mode the boat should be in
    if (this->battery_state[0] <  this->brown_out_thresh) {
        this->mode = power_safe;
        this->last_satellite_broadcast = 0; // force satellite update
        this->time_mode_change = millis();
    } else if (this->observed_state[6] >  this->rough_sea_thresh) {
        this->mode = turbulence_safe;
        this->last_satellite_broadcast = 0; // force satellite update
        this->time_mode_change = millis();
    } else if (this->mission->waypoints[this->mission->current_waypoint].type == navigation) {
        this->mode = cruise;
    } else if (this->mission->waypoints[this->mission->current_waypoint].type == loitering) {
        if (determine_error_distance(this->observed_state, this->mission->waypoints[this->mission->current_waypoint]) < 0.33 * this->mission->waypoints[this->mission->current_waypoint].radius ) {
            this->mode = loiter;
        } else if (determine_error_distance(this->observed_state, this->mission->waypoints[this->mission->current_waypoint]) > 0.67 * this->mission->waypoints[this->mission->current_waypoint].radius ) {
            this->mode = loiter_cruise;
        }
    }
}

long IQT_VEHICLE::dist_from_wp() {
    return determine_error_distance(this->observed_state, this->mission->waypoints[this->mission->current_waypoint]);
}

void IQT_VEHICLE::get_new_mission(Mission *mis) {
    this->mission = mis;
}


Mission* IQT_VEHICLE::check_waypoint_progression() {
    
    // if the acceptable radius around a navigation waypoint has been reached, advance to next waypoint
    if (this->mission->waypoints[this->mission->current_waypoint].type == navigation && determine_error_distance(this->observed_state, this->mission->waypoints[this->mission->current_waypoint]) < this->mission->waypoints[this->mission->current_waypoint].radius) {
        this->mission->current_waypoint = (this->mission->current_waypoint + 1) % this->mission->number_waypoints;
        
        // Debug
        Serial.println(this->mission->current_waypoint);
        Serial.println("NAV WP PROGRESSION");
        
        
        saveMission(this->mission); // save back mission to SD card after progression
        this->last_satellite_broadcast = 0; // force satellite update


    } else if (this->mission->waypoints[this->mission->current_waypoint].type == loitering) {
        
        // if at a loitering waypoint, within radius, but station-keeping time has not been exceeded, add to station-keep time.
        if (this->mission->time_at_current < this->mission->waypoints[this->mission->current_waypoint].time_on_station && determine_error_distance(this->observed_state, this->mission->waypoints[this->mission->current_waypoint]) < this->mission->waypoints[this->mission->current_waypoint].radius) {
            this->mission->time_at_current += millis()-this->last_waypoint_check;
        
        // if at loitering waypoint and the station-keeping time has been exceeded, advance to next waypoint
        } else if (this->mission->time_at_current >= this->mission->waypoints[this->mission->current_waypoint].time_on_station) {

            // debug
            Serial.println(this->mission->time_at_current);
            Serial.println(this->mission->waypoints[this->mission->current_waypoint].time_on_station);
            Serial.println("LOITER WP PROGRESSION");

            this->mission->current_waypoint = (this->mission->current_waypoint + 1) % this->mission->number_waypoints;
            this->mission->time_at_current = 0;
            
            saveMission(this->mission); // save back mission to SD after progression
            this->last_satellite_broadcast = 0; // force satellite update
        }
    }
    this->last_waypoint_check = millis();
    return this->mission;

}

void IQT_VEHICLE::broadcast_satellite(IridiumSBD modem, long state1[], long state2[], int steer, int thrust, int trim) {

    long timer = millis();
    uint8_t buffer[256];
    size_t bufferSize = sizeof(buffer);
    bool ring = modem.hasRingAsserted();
    int signalQuality = -1;


    // if there is a notification of a new message burst (ring), a known message waiting, or sufficient time has elapsed (including a timer reset by a broadcast-triggering event),
    //  AND there is good signal quality
    if ((millis() - this->last_satellite_broadcast > this->update_interval || ring || modem.getWaitingMessageCount() > 0) && modem.getSignalQuality(signalQuality) > 0) {
        
        // Debug
        Serial.println(millis() - this->last_satellite_broadcast);
        Serial.println(ring);
        Serial.println(modem.getWaitingMessageCount());
        Serial.println("attempt statelite update");
        
        // Pack all the types of messages into the satellite write buffer
        pack_vehicle_state_vector(this->observed_state, buffer);
        pack_vehicle_battery_vector(this->battery_state, buffer+34);
        pack_controller_state_vector(steer, thrust, trim, determine_bearing(this->observed_state[0], this->observed_state[1], this->mission->waypoints[this->mission->current_waypoint].latitude, this->mission->waypoints[this->mission->current_waypoint].longitude), buffer+34+34);
        pack_gps_peripheral(state1, buffer+34+34+19);
        pack_gps_peripheral(state2, buffer+34+34+19+34);
        pack_waypoint(this->mission->waypoints[this->mission->current_waypoint], buffer+34+34+19+34+34);

        // Initalize short-burst data session and communicate information
        modem.sendReceiveSBDBinary(buffer, 179, buffer, bufferSize);
        Serial.println("sent statelite update");
        modem.markRingFalse();

        this->last_satellite_broadcast = millis();

        // If there were any messages recieved, respond appropriately
        if (bufferSize > 2) {
            Serial.println("Sat MESSAGE Available");
            
            // chosse an option based on the message tag
            if (buffer[0] == START_BYTE) {
                switch (buffer[1]){
                    case REQUEST_WAYPOINT_LIST:
                        this->unpack_request_waypoint_list(modem);
                        break;
                    case REQUEST_WAYPOINT:
                        this->unpack_request_waypoint(modem, buffer);
                        break;
                    case CHANGE_WAYPOINT:
                        this->unpack_change_waypoint(modem, buffer);
                        break;
                    case GET_MISSION_PARAMS:
                        this->unpack_get_mission_params(modem);
                        break;
                    case CHANGE_MISSIONS_PARAMS:
                        this->unpack_change_mission_params(modem, buffer);
                        break;
                    case FORCE_MODE_CHANGE:
                        this->unpack_force_mode_change(buffer);
                        break;
                }
            }
        }
    }
}


// TODO
void IQT_VEHICLE::check_for_reset() {
    if (millis() > 604800000) { // reset after 1 week 
        // 1. broadcast satellite update reset message (NOT YET IMPLEMENTED)
        // 2. write back mission parameters 
        // 3. power cycle self -> (will also power cycle relays)

        saveMission(this->mission);
        digitalWrite(this->reset_pin, LOW);
        exit();
    }
}



Mode IQT_VEHICLE::get_mode() {
    return this->mode;
}

unsigned long IQT_VEHICLE::get_target_speed() {
    return this->mission->cruise_speed;
}
long IQT_VEHICLE::get_est_heading() {
    return this->est_heading;
}

Waypoint IQT_VEHICLE::get_waypoint() {
    return this->mission->waypoints[this->mission->current_waypoint];
}

Waypoint IQT_VEHICLE::get_target() {
    if (this->mission->current_waypoint == 0) {
        return this->mission->waypoints[this->mission->current_waypoint];
    } 
    
    long boatToFinal = determine_error_distance(this->observed_state, this->mission->waypoints[this->mission->current_waypoint]);
    long initialToBoat = determine_error_distance(this->observed_state, this->mission->waypoints[this->mission->current_waypoint - 1]);
    long initialToFinal = determine_error_distance(this->mission->waypoints[this->mission->current_waypoint - 1].latitude, this->mission->waypoints[this->mission->current_waypoint - 1].longitude, this->mission->waypoints[this->mission->current_waypoint].latitude, this->mission->waypoints[this->mission->current_waypoint].longitude);

    if(boatToFinal < this->mission->lead_distance || initialToBoat > initialToFinal) {
        return this->mission->waypoints[this->mission->current_waypoint];
    } else {
        Waypoint wp;
        wp.radius = this->mission->waypoints[this->mission->current_waypoint].radius;
        wp.type = this->mission->waypoints[this->mission->current_waypoint].type;
        wp.time_on_station = this->mission->waypoints[this->mission->current_waypoint].time_on_station;

        determine_resultant_position(&wp, this->mission->waypoints[this->mission->current_waypoint - 1], determine_bearing(this->mission->waypoints[this->mission->current_waypoint - 1].latitude, this->mission->waypoints[this->mission->current_waypoint - 1].longitude, this->mission->waypoints[this->mission->current_waypoint].latitude, this->mission->waypoints[this->mission->current_waypoint].longitude), initialToBoat + this->mission->lead_distance);
        return wp;
    }

    
}

void IQT_VEHICLE::get_state(long state[]) {
    for (int i = 0; i < 8; i++) {
        state[i] = this->observed_state[i];
    }
}
void IQT_VEHICLE::get_battery(long bat[]) {
    for (int i = 0; i < 8; i++) {
        bat[i] = this->battery_state[i];
    }
}

void IQT_VEHICLE::get_motor_enabled(bool enb[]) {
    for (int i = 0; i < 3; i++) {
        enb[i] = this->mission->motor_enabled[i];
    }
}

void IQT_VEHICLE::get_sensor_enabled(Sensor_enable enb[]) {
    for (int i = 0; i < 3; i++) {
        enb[i] = this->mission->sensor_force_enable[i];
    }
}


void IQT_VEHICLE::unpack_request_waypoint_list(IridiumSBD modem) {
    byte buffer[256];
    for (int i = 0; i < 256; i++) {
        buffer[i] = 0;
    }
    pack_waypoint_list(this->mission->waypoints, buffer, this->mission->number_waypoints);
    modem.sendSBDBinary(buffer, 256);
    Serial.println("sent statelite wps");
    this->last_satellite_broadcast = millis();
    return;
}

void IQT_VEHICLE::unpack_request_waypoint(IridiumSBD modem, byte buf[]) {
    byte buffer[WAYPOINT_LENGTH];
    pack_waypoint(this->mission->waypoints[buf[2]], buffer);
    modem.sendSBDBinary(buffer, WAYPOINT_LENGTH);
    Serial.println("sent statelite wp");
    this->last_satellite_broadcast = millis();
    return;

}

void IQT_VEHICLE::unpack_change_waypoint(IridiumSBD modem, byte buf[]) {
    
    int idx = buf[2];
    this->mission->waypoints[idx].latitude = pull_long_out_of_buffer(3, buf);
    this->mission->waypoints[idx].longitude = pull_long_out_of_buffer(7, buf);
    this->mission->waypoints[idx].radius = pull_long_out_of_buffer(11, buf);
    this->mission->waypoints[idx].type = WaypointType(pull_long_out_of_buffer(15, buf));
    this->mission->waypoints[idx].time_on_station = pull_long_out_of_buffer(19, buf);
    
    this->unpack_request_waypoint(modem, buf);

    saveWaypoint(idx, mission);

    return;

}

void IQT_VEHICLE::unpack_get_mission_params(IridiumSBD modem) {
    byte buffer[MISSION_PARAMS_LEN];
    pack_mission_params(this->mission, buffer);
    modem.sendSBDBinary(buffer, MISSION_PARAMS_LEN);
    Serial.println("sent statelite mission params");
    this->last_satellite_broadcast = millis();
    return;

}

void IQT_VEHICLE::unpack_change_mission_params(IridiumSBD modem, byte buf[]) {
    this->mission->current_waypoint = pull_long_out_of_buffer(2, buf);
    this->mission->number_waypoints = pull_long_out_of_buffer(6, buf);
    this->mission->time_at_current = pull_long_out_of_buffer(10, buf);
    this->mission->cruise_speed = pull_long_out_of_buffer(14, buf);
    this->mission->steer_big_p_gain = pull_long_out_of_buffer(18, buf)/1.0E6;
    this->mission->steer_small_p_gain = pull_long_out_of_buffer(22, buf)/1.0E6;
    this->mission->steer_i_gain = pull_long_out_of_buffer(26, buf)/1.0E7;
    this->mission->thrust_p_gain = pull_long_out_of_buffer(30, buf)/1.0E3;
    this->mission->transition_threshold = pull_long_out_of_buffer(34, buf);
    this->mission->rough_sea = pull_long_out_of_buffer(38, buf);
    this->mission->brownout_thresh = pull_long_out_of_buffer(42, buf);
    this->mission->sat_update = pull_long_out_of_buffer(46, buf);
    this->mission->exit_safemode_time = pull_long_out_of_buffer(50, buf);
    this->mission->lead_distance = pull_long_out_of_buffer(54, buf);

    this->unpack_get_mission_params(modem);

    // Force restart if mission save successful
    if (!saveMission(this->mission)) {
        digitalWrite(this->reset_pin, LOW);
    }
    return;

}

void IQT_VEHICLE::unpack_force_mode_change(byte buf[]) {
    this->mode = Mode(buf[2]);
    return;
}



long determine_bearing(long curr_lat, long curr_lon, long tar_lat, long tar_lon) {

    double lat1 = (curr_lat/1.0E7)*(PI/180);
    double lon1 = (curr_lon/1.0E7)*(PI/180);
    double lat2 = (tar_lat/1.0E7)*(PI/180);
    double lon2 = (tar_lon/1.0E7)*(PI/180);

    double y = sin(lon2-lon1) * cos(lat2);
    double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon2-lon1);
    double theta = atan2(y, x);
    long bng =  ((long)((theta*180/PI + 360)* 1E5) % (long) 360E5) ;

    return bng;
}

double determine_error_distance(long state[], Waypoint wp) {
    double lat1 = state[0]/1.0E7* (PI/180);
    double lon1 = state[1]/1.0E7* (PI/180);
    double lat2 = wp.latitude/1.0E7* (PI/180);
    double lon2 = wp.longitude/1.0E7* (PI/180);

    double R = 6371E3; // metres
    double d = (R * acos(sin(lat1)*sin(lat2)+cos(lat1)*cos(lat2)*cos(lon1-lon2))); // in m

    return d; // in meters

}

double determine_error_distance(long startLat, long startLon, long endLat, long endLon) {
    double lat1 = startLat/1.0E7* (PI/180);
    double lon1 = startLon/1.0E7* (PI/180);
    double lat2 = endLat/1.0E7* (PI/180);
    double lon2 = endLon/1.0E7* (PI/180);

    double R = 6371E3; // metres
    double d = (R * acos(sin(lat1)*sin(lat2)+cos(lat1)*cos(lat2)*cos(lon1-lon2))); // in m

    return d; // in meters

}

void determine_resultant_position(Waypoint *wp, Waypoint start, long bearing, long distance) {
    double lat1 = start.latitude/1.0E7 * (PI/180);
    double lon1 = start.longitude/1.0E7 * (PI/180);
    double bng = bearing/1.0E5 * (PI/180);
    
    double ang_d = distance/6371E3; 
    double lat2 = asin(sin(lat1) *cos(ang_d) + cos(lat1) * sin(ang_d) * cos(bng));
    double lon2 = lon1 + atan2(sin(bng)*sin(ang_d)*cos(lat1), cos(ang_d)-sin(lat1)*sin(lat2));

    wp->latitude = long ((lat2*(180/PI))*1.0E7);
    wp->longitude = long ((lon2*(180/PI))*1.0E7);

    return;
}



int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  delay(15);
  unsigned long timeout = 300;
  bool running = true;
  bool pinHigh = false;
  unsigned long start = 0;
  unsigned long inital = millis();
  unsigned long ch = 0;
  unsigned long debounce_count = 0;
  while(running) {
    if (!pinHigh) {
        if (digitalRead(channelInput) == HIGH && debounce_count < 10) {
            debounce_count++;
        } else if (digitalRead(channelInput) == LOW) {
            debounce_count = 0;
        } else if (digitalRead(channelInput) == HIGH) {
            start = micros();
            pinHigh = true;
            debounce_count = 0;
        }
    } else if (start > 0) {
        if (digitalRead(channelInput) == LOW && debounce_count < 10) {
            debounce_count++;
        } else if (digitalRead(channelInput) == HIGH) {
            debounce_count = 0;
        } else if (digitalRead(channelInput) == LOW) {
            ch = micros() - start;
            if (ch > 999) {
                running = false;
                break;
            }
        } 
    }
    if (millis() - inital > timeout) {
        running = false;
        break;
    }
  }
  Serial.print(ch);
  Serial.print(", ");

  if (ch < 100) return defaultValue;
  
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

bool manSwitch(byte channelInput, bool defaultValue){
  delay(100); // loop cycles faster than the normal 3htz in manual mode, this is needed to eliminate noise 
  int intDefaultValue = (defaultValue)? 100: 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 75);
}