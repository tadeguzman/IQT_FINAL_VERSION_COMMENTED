#include "Arduino.h"
#include "IQT_COMMUNICATIONS.h"

// Theo DeGuzman
// Johns Hopkins University
// IQT24 Senior Design Project
// V2 Prototype Code
// Not for production use -- end user assumes responsibility for usage and testing of this code


void push_long_into_buffer(long val, int start_index, byte buf[]) {
    for (int i = 0; i < 4; i++) {
        buf[start_index+i] = (byte) ((val >> (8 * i)));
    }
}

long pull_long_out_of_buffer(int start_index, byte buf[]) {
    long recoveredValue = 0;
    for (int i = 0; i < 4; i++) {
        long byteVal = (((long) buf[start_index+i]) << (8 * i));
        recoveredValue += byteVal;
    }
    return recoveredValue;

}

void pack_gps_peripheral(long state[], byte buf[]){
    buf[0] = START_BYTE;
    buf[1] = PERIPHERAL_STATE_VECTOR;
    push_long_into_buffer(state[0], 2, buf);
    push_long_into_buffer(state[1], 6, buf);
    push_long_into_buffer(state[2], 10, buf);
    push_long_into_buffer(state[3], 14, buf);
    push_long_into_buffer(state[4], 18, buf);
    push_long_into_buffer(state[5], 22, buf);
    push_long_into_buffer(state[6], 26, buf);
    push_long_into_buffer(state[7], 30, buf);
    buf[34] = END_BYTE;
}

void unpack_gps_peripheral(long val[], byte buf[]) {

    if (buf[0] != START_BYTE && buf[1] != PERIPHERAL_STATE_VECTOR && buf[34 != END_BYTE]) {
        return;
    }

    val[0] = pull_long_out_of_buffer(2, buf);
    val[1] = pull_long_out_of_buffer(6, buf);
    val[2] = pull_long_out_of_buffer(10, buf);
    val[3] = pull_long_out_of_buffer(14, buf);
    val[4] = pull_long_out_of_buffer(18, buf);
    val[5] = pull_long_out_of_buffer(22, buf);
    val[6] = pull_long_out_of_buffer(26, buf);
    val[7] = pull_long_out_of_buffer(30, buf);

}



void pack_vehicle_state_vector(long state[], byte buf[]) {
    buf[0] = START_BYTE;
    buf[1] = VEHICLE_STATE_VECTOR;
    push_long_into_buffer(state[0], 2, buf);
    push_long_into_buffer(state[1], 6, buf);
    push_long_into_buffer(state[2], 10, buf);
    push_long_into_buffer(state[3], 14, buf);
    push_long_into_buffer(state[4], 18, buf);
    push_long_into_buffer(state[5], 22, buf);
    push_long_into_buffer(state[6], 26, buf);
    push_long_into_buffer(state[7], 30, buf);
    buf[34] = END_BYTE;

}
void pack_vehicle_battery_vector(long state[], byte buf[]) {
    buf[0] = START_BYTE;
    buf[1] = VEHICLE_BATTERY_VECTOR;
    push_long_into_buffer(state[0], 2, buf);
    push_long_into_buffer(state[1], 6, buf);
    push_long_into_buffer(state[2], 10, buf);
    push_long_into_buffer(state[3], 14, buf);
    push_long_into_buffer(state[4], 18, buf);
    push_long_into_buffer(state[5], 22, buf);
    push_long_into_buffer(state[6], 26, buf);
    push_long_into_buffer(state[7], 30, buf);
    buf[34] = END_BYTE;
}


void pack_controller_state_vector(int steer, int thrust, int trim, int error, byte buf[]) {
    buf[0] = START_BYTE;
    buf[1] = CONTROLLER_STATE;
    push_long_into_buffer(steer, 2, buf);
    push_long_into_buffer(thrust, 6, buf);
    push_long_into_buffer(trim, 10, buf);
    push_long_into_buffer(error, 14, buf);
    buf[18] = END_BYTE;
}


void pack_waypoint_list(Waypoint waypoints[], byte buf[], unsigned long num_wps) {
    
    buf[0] = START_BYTE;
    buf[1] = WAYPOINT_LIST;
    buf[2] = min(28, num_wps);   

    int i = 0;
    for (i; i < buf[2]; i++) {
        push_long_into_buffer(waypoints[i].latitude, i*9+3, buf);
        push_long_into_buffer(waypoints[i].longitude, i*9+7, buf);
        buf[i*9+11] = (byte) waypoints[i].type;
    }
    buf[i*9+3] = END_BYTE;
}

void pack_waypoint(Waypoint wp, byte buf[]) {
    buf[0] = START_BYTE;
    buf[1] = WAYPOINT;
    push_long_into_buffer(wp.latitude, 2, buf);
    push_long_into_buffer(wp.longitude, 6, buf);
    push_long_into_buffer(wp.radius, 10, buf);
    push_long_into_buffer(wp.type, 14, buf);
    push_long_into_buffer(wp.time_on_station, 18, buf);
    buf[22] = END_BYTE;

}

void pack_mode_change(int last_mode, int mode, byte buf[]) {
    buf[0] = START_BYTE;
    buf[1] = MODE_CHANGE;
    push_long_into_buffer(last_mode, 2, buf);
    push_long_into_buffer(mode, 6, buf);
    buf[10] = END_BYTE;

}

void pack_waypoint_reached(long last_lat, long last_lon, int last_type, long lat, long lon, int type, byte buf[]) {
    buf[0] = START_BYTE;
    buf[1] = WAYPOINT_REACHED;
    push_long_into_buffer(last_lat, 2, buf);
    push_long_into_buffer(last_lon, 6, buf);
    push_long_into_buffer(last_type, 10, buf);
    push_long_into_buffer(lat, 14, buf);
    push_long_into_buffer(lon, 18, buf);
    push_long_into_buffer(type, 22, buf);
    buf[26] = END_BYTE;

}

void pack_mission_params(Mission *mis, byte buf[]) {
    buf[0] = START_BYTE;
    buf[1] = MISSION_PARAMS;
    push_long_into_buffer(mis->current_waypoint, 2, buf);
    push_long_into_buffer(mis->number_waypoints, 6, buf);
    push_long_into_buffer(mis->time_at_current, 10, buf);
    push_long_into_buffer(mis->cruise_speed, 14, buf);
    push_long_into_buffer((int) mis->steer_big_p_gain*1.0E6, 18, buf);
    push_long_into_buffer((int) mis->steer_small_p_gain*1.0E6, 22, buf);
    push_long_into_buffer((int) mis->steer_i_gain*1.0E7, 26, buf);
    push_long_into_buffer((int) mis->thrust_p_gain*1.0E5, 30, buf);
    push_long_into_buffer(mis->transition_threshold, 34, buf);
    push_long_into_buffer(mis->rough_sea, 38, buf);
    push_long_into_buffer(mis->brownout_thresh, 42, buf);
    push_long_into_buffer(mis->sat_update, 46, buf);
    push_long_into_buffer(mis->exit_safemode_time, 50, buf);
    push_long_into_buffer(mis->lead_distance, 54, buf);
    buf[58] = END_BYTE;
}




void gps_poll(int int_pin, Stream &port, long state[]) {

    // Set the interrupt pin to low temporarily
    digitalWrite(int_pin, LOW);
    delay(1);
    digitalWrite(int_pin, HIGH);
    delay(10);


    // if the message has been recieved, update state vector
    if (port.available() > 0) {
        unsigned long timeout = millis();
        while(port.available() && port.read() != START_BYTE && millis() -timeout < 10);
        if (millis() - timeout > 10) {
            return;
        }


        // Read the incoming data and print it to the serial monitor
        byte buf[PERIPHERAL_STATE_VECTOR_LENGTH];
        buf[0] = START_BYTE;
        for (int i = 1; i < PERIPHERAL_STATE_VECTOR_LENGTH; i++) {
            unsigned long timeout2 = millis();
            while(!port.available() && millis() - timeout2 < 10);
            if (millis() - timeout2 > 10) {
                return;
            }
            buf[i] = port.read();
        }   
        unpack_gps_peripheral(state, buf);
    }
}


int loadMission(Mission *mis) {
  // Open file for reading
  File file = SD.open("mp.txt");

  // Allocate the memory pool on the stack.
  // Don't forget to change the capacity to match your JSON document.
  // Use arduinojson.org/assistant to compute the capacity.
  JsonDocument root;
  
  if (deserializeJson(root, file)) return 1;
  // Copy values from the JsonObject to the Config
  mis->current_waypoint = root["current_waypoint"];
  mis->number_waypoints = root["number_waypoints"];
  mis->time_at_current = root["time_at_current"];
  mis->cruise_speed = root["cruise_speed"];
  mis->steer_big_p_gain = root["steer_big_p_gain"];
  mis->steer_small_p_gain = root["steer_small_p_gain"];
  mis->steer_i_gain = root["steer_i_gain"];
  mis->thrust_p_gain = root["thrust_p_gain"];
  mis->transition_threshold = root["transition_threshold"];
  mis->rough_sea = root["rough_sea"];
  mis->brownout_thresh = root["brownout_thresh"];
  mis->sat_update = root["sat_update"];
  mis->exit_safemode_time = root["exit_safemode_time"];
  mis->lead_distance = root["lead_distance"];

//   // TODO: add sensor force enable and motor enable
//   mis->motor_enabled = new bool[3];
//   mis->motor_enabled[0] = root["motor_enabled"][0];
//   mis->motor_enabled[1] = root["motor_enabled"][1];
//   mis->motor_enabled[2] = root["motor_enabled"][2];

//   mis->sensor_force_enable = new Sensor_enable[2];
//   mis->sensor_force_enable[0] = root["sensor_force_enable"][0];
//   mis->sensor_force_enable[1] = root["sensor_force_enable"][1];


  // Close the file (File's destructor doesn't close the file)
  file.close();
  return 0;
}



int loadWaypoints(int wp_num, Mission *mis) {
  
  char fn[8];
  sprintf(fn, "wp%02d.txt", wp_num);

  // Open file for reading
  File file = SD.open(fn);

  // Allocate the memory pool on the stack.
  // Don't forget to change the capacity to match your JSON document.
  // Use arduinojson.org/assistant to compute the capacity.
  JsonDocument root;

  if (deserializeJson(root, file)) return 1;

  mis->waypoints[wp_num].latitude = root["latitude"];
  mis->waypoints[wp_num].longitude = root["longitude"];
  mis->waypoints[wp_num].radius = root["radius"];
  mis->waypoints[wp_num].type = root["type"];
  mis->waypoints[wp_num].time_on_station = root["time_on_station"];

  // Close the file (File's destructor doesn't close the file)
  file.close();

  return 0;
}





int saveMission(Mission *mis) {
  // Delete existing file, otherwise the configuration is appended to the file
  SD.remove("mp.txt");

  Serial.println("start saving mission");
  // Open file for writing
  File file = SD.open("mp.txt", FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to create file"));
    return 1;
  }

  // Allocate the memory pool on the stack
  // Don't forget to change the capacity to match your JSON document.
  // Use https://arduinojson.org/assistant/ to compute the capacity.
  JsonDocument root;

  // Set the values
  root["current_waypoint"] = mis->current_waypoint;
  root["number_waypoints"] = mis->number_waypoints;
  root["time_at_current"] = mis->time_at_current;
  root["cruise_speed"] = mis->cruise_speed;
  root["steer_big_p_gain"] = mis->steer_big_p_gain;
  root["steer_small_p_gain"] = mis->steer_small_p_gain;
  root["steer_i_gain"] = mis->steer_i_gain;
  root["thrust_p_gain"] = mis->thrust_p_gain;
  root["transition_threshold"] = mis->transition_threshold;
  root["rough_sea"] = mis->rough_sea;
  root["brownout_thresh"] = mis->brownout_thresh;
  root["sat_update"] = mis->sat_update;
  root["exit_safemode_time"] = mis->exit_safemode_time;
  root["lead_distance"] = mis->lead_distance;


  // Serialize JSON to file
  if (serializeJson(root, file) == 0) {
    Serial.println(F("Failed to write to file"));
    return 1;
  }

  Serial.println("Wrote back mission!");

  // Close the file (File's destructor doesn't close the file)
  file.close();
  return 0;
}



int saveWaypoint(int wp_num, Mission *mis) {
    // Delete existing file, otherwise the configuration is appended to the file
  
  char fn[8];
  sprintf(fn, "wp%02d.txt", wp_num);
  SD.remove(fn);

  // Open file for writing
  File file = SD.open(fn, FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to create file"));
    return 1;
  }

  // Allocate the memory pool on the stack
  // Don't forget to change the capacity to match your JSON document.
  // Use https://arduinojson.org/assistant/ to compute the capacity.
  JsonDocument root;

  // Set the values
  root["latitude"] = mis->waypoints[wp_num].latitude;
  root["longitude"] = mis->waypoints[wp_num].longitude;
  root["radius"] = mis->waypoints[wp_num].radius;
  root["type"] = mis->waypoints[wp_num].type;
  root["time_on_station"] = mis->waypoints[wp_num].time_on_station;

  // Serialize JSON to file
  if (serializeJson(root, file) == 0) {
    Serial.println(F("Failed to write to file"));
    return 1;
  }

  // Close the file (File's destructor doesn't close the file)
  file.close();
  return 0;

}




