/*

*/

/* TODO
  Assign units to controller parameters
  Develop conversions from controller units to actually control the inputs
  Controller safety - if state or input is about to overflow
  Design experiments to determine process gain and time constants, then to assign controller gain and time constants
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <LiquidCrystal.h>

// Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// SD Card Variables
// Use a FAT16 formatted card
const byte chip_sel = 4; // The SPI selection pin for the SD card
File data_file;
File prgm_file;
File prgm_dir;
//byte prgm_num;
unsigned short sample_num; // can store up to 65536 samples (to keep with 8.3 convention, files are named samXXXXX.csv)

// Timer Variables
const unsigned int timer_period = 10; // Update once you decide on the hardware

// State Machine Variables
/*
Bit 0 - controlling internal temp?
Bit 1 - controlling heatsink temp?
Bit 2 - controlling capillary flow?
Bit 3 - reading other sensors?
Bit 4 - recording data?
Bit 5 - within a run?

Possible values within this code:
B000000: default state
B001011: controlling temperatures
B101011: controlling temperatures during a run
B111111: controlling temperatures and flow and recording data during a run
*/
byte states = 0;
byte UI_loc[] = {1, 0, 0, 0}; // Keeps track of where the user is currently in the interface
volatile byte flags = 0; // Bit 0 - select flag, Bit 1 - cycle flag

// Controller Variables
const byte ctrlr_num = 3;
volatile byte ctrlr_sel = 0;
int duty_cyc;
float gain[] = {0.0, 0.0, 0.0}; // Configure these with lab measurements, assign units
float time_const[] = {0.0, 0.0, 0.0}; // Configure these with lab measurements, assign units
float err_factor[3];
volatile float CV[3];
volatile float err[3];
volatile float MV[3];
volatile float flow_tot;
volatile float log_data[3]; // ambient temp, ambient humidity, ambient pressure
float log_once_data[3]; // latitude, longitude, time
float monitor_data[1]; // battery charge
float setpoint[] = {-10.0, 60.0, 100.0, 500.0}; // *C, *C, mL/min, mL; configured in settings. internal temp, heatsink temp, capillary flow, and total flow setpoints
// If you change the size of CV, log_data, log_once_data, or setpoint, you need to change the bounds of the for loops that iterate over them below!

// Program Control ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup() {
  
  // Controller setup calculations
  duty_cyc = ctrlr_num*timer_period; // Controller duty cycle - the interval between reading data points
  for (byte i = 0; i < ctrlr_num; i++) {
    err_factor[i] = 1 + duty_cyc/time_const[i];
  }

  // Attempts to initialize the SD card, and if it fails, stops the program.
  if (!SD.begin(chip_sel)) {
    while (1);
  }
  
  //prgm_dir = SD.open(F("/programs")); // Opens the programs directory on the SD card.
  //prgm_num = num_files(prgm_dir); // Counts the number of files in the programs directory, excluding subdirectories, to determine how many program options to display
  //prgm_dir.close();
  
  File data_dir = SD.open(F("/data"));
  sample_num = num_files(data_dir); // Counts the number of files in the data directory, excluding subdirectories, to determine where to start numbering new data files
  data_dir.close();

  // Initialize I2C bus
  Wire.begin();
  Wire.setClock(100000); // 100 kHz I2C clock speed
  while (start_flow_sensor()); // Wait for flow sensor to start up and receive its serial number
}

void loop() {

  // TODO call flush() on the data file occasionally
  
  // TODO screen updates based on current state
  // for program list, display the name of the program file using prgm_file.name()
  // This massive conditional tree takes the program to a unique place in the code dependent on the current state, which is determined by the UI_loc, states, and prgm_file variables.
  // It continuously updates the screen based on the current state.
  // It executes an action specific to that state if the cycle or the select button was just pressed.
  switch (UI_loc[0]) {
  case 1:
    if (UI_loc[1] == 0) { // "Programs" is selected in top list
      if (bitRead(flags, 0)) { // Move into program list and open the first program file
        UI_loc[1] = 1;
        prgm_dir = SD.open(F("/programs")); // Opens the programs directory on the SD card.
        prgm_file = prgm_dir.openNextFile();
        flags = 0; // Clears both button flags so that no weird behavior arises from simultaneous button presses
      }
      else if (bitRead(flags, 1)) { // Cycle to "Status" in top list
        UI_loc[0] = 2;
        flags = 0;
      }
      // Display "Programs" highlighted and "Status" not.
    }
    else if (!prgm_file) { // "Return" is selected in program list
      if (bitRead(flags, 0)) { // Move back to top list
        UI_loc[1] = 0;
        prgm_file.close();
        prgm_dir.close();
        flags = 0;
      }
      else if (bitRead(flags, 1)) { // Cycle to first program in program list
        UI_loc[1] = 1;
        prgm_dir.rewindDirectory();
        prgm_file = prgm_dir.openNextFile();
        flags = 0;
      }
    }
    else {
      if (UI_loc[2] == 0) { // A program is selected in program list
        if (bitRead(flags, 0)) { // Read the program, set the run state bit, and move into run status list
          // read program file
          String setting_str;
          byte setting_sel = 0;
          while (prgm_file.available() > 0) { // While there are more characters to be read:
            char read_char = prgm_file.read();
            if (read_char != '\n') {
              setting_str += read_char;
            }
            else {
              setpoint[setting_sel] = setting_str.toFloat();
              setting_str = "";
              if (setting_sel < 3) { // CHANGE IF YOU CHANGE THE SIZE OF setpoint
                setting_sel++;
              }
              else {
                break;
              }
            }
          }
          prgm_file.close();
          prgm_dir.close();
          states = B101011;
          UI_loc[2] = 1;
          flags = 0;
        }
        else if (bitRead(flags, 1)) { // Cycle to next program in program list
          UI_loc[1]++;
          prgm_file.close();
          prgm_file = prgm_dir.openNextFile();
          flags = 0;
        }
      }
      else {
        switch (UI_loc[3]) {
        case 0: // A run status display is selected in run status list
          if (bitRead(flags, 0)) { // Clear the flow control and data log bits, stop flow, and move into paused run list
            states = B101011;
            MV[3] = 0;
            UI_loc[3] = 1;
            flags = 0;
          }
          else if (bitRead(flags, 1)) { // Cycle to next run status display in run status list
            if (UI_loc[2] < 3) {
              UI_loc[2]++;
            }
            else {
              UI_loc[2] = 1;
            }
            flags = 0;
          }
          else {
            switch (UI_loc[2]) {
            case 1: // Run status display 1 is selected in run status list
              
              break;
            case 2: // Run status display 2 is selected in run status list
              
              break;
            case 3: // The final run status display is selected in run status list
              
              break;
            }
            if (states == B111111 && flow_tot >= setpoint[ctrlr_num]) { // If the system is currently sampling, is not paused, and has reached the total flow setpoint, stop run once total flow reaches configured value
              states = B001011;
              MV[3] = 0;
              data_file.close();
              sample_num++; // Increment the collection counter
              flow_tot = 0; // Reset the flow rate integration
              UI_loc[3] = 0;
              UI_loc[2] = 0;
              UI_loc[1] = 0;
            }
            else if (states == B101011 && abs(CV[0] - setpoint[0]) < 1) { // If the system is cooling to start flow, is not paused, and has reached temp setpoint (to within one degree), switch flow on once temp hits setpoint
              // Read all the sensor data to log once
              read_GPS();
              if (!data_file) { // If the data file hasn't been opened
                data_file = SD.open(String(F("/data/sam")) + String(sample_num) + String(F(".csv")), FILE_WRITE); // Open a new data file for writing
              }
              // Write log_once_data to data file
              // Construct a comma-separated string containing the sensor data
              String data_str = "*"; // Starred data lines indicate they contain log_once_data
              for (byte data_sel = 0; data_sel < 3; data_sel++) { // CHANGE IF YOU CHANGE THE SIZE OF log_once_data
                data_str += String(log_once_data[data_sel]);
                if (data_sel < 2) { // CHANGE IF YOU CHANGE THE SIZE OF log_once_data
                  data_str += ",";
                }
              }
              data_file.println(data_str + "\n"); // Write sensor data to current data file
              states = B111111; // Start flow and recording data
            }
          }
          break;
        case 1: // "Finish" is selected in paused run list
          if (bitRead(flags, 0)) { // Clear the run state bit, increment the sample counter, close the data file, and move back to top list
            states = B001011;
            data_file.close();
            sample_num++; // Increment the collection counter
            flow_tot = 0; // Reset the flow rate integration
            UI_loc[3] = 0;
            UI_loc[2] = 0;
            UI_loc[1] = 0;
            flags = 0;
          }
          else if (bitRead(flags, 1)) { // Cycle to "Resume" in paused run list
            UI_loc[3] = 2;
            flags = 0;
          }
          break;
        case 2: // "Resume" is selected in paused run list
          if (bitRead(flags, 0)) { // Move back to run status list to continue flow. The flow control and data log bits will be set once the temp setpoint is reached (may be on the next loop if paused during flow), and the log_once_data will be written again.
            UI_loc[3] = 0;
            flags = 0;
          }
          if (bitRead(flags, 1)) { // Cycle to "Finish" in paused run list
            UI_loc[3] = 1;
            flags = 0;
          }
          break;
        }
      }
    }
    break;
  case 2:
    if (UI_loc[1] == 0) { // "Status" is selected in top list
      if (bitRead(flags, 0)) { // Move into status list
        UI_loc[1] = 1;
        flags = 0;
      }
      if (bitRead(flags, 1)) { // Cycle to "Programs" in top list
        UI_loc[0] = 1;
        flags = 0;
      }
    }
    else { // A status display is selected in status list
      if (bitRead(flags, 0)) { // Move back to top list
        UI_loc[1] = 0;
        UI_loc[0] = 1;
        flags = 0;
      }
      switch (UI_loc[1]) {
      case 1: // Status display 1 is selected in status list
        if (bitRead(flags, 1)) { // Cycle to next status display in status list
          UI_loc[1]++;
          flags = 0;
        }
        break;
      case 2: // Status display 2 is selected in status list
        if (bitRead(flags, 1)) { // Cycle to next status display in status list
          UI_loc[1]++;
          flags = 0;
        }
        break;
      case 3: // The final status display is selected in status list
        if (bitRead(flags, 1)) { // Cycle to first status display in status list
          UI_loc[1] = 1;
          flags = 0;
        }
        break;
      }
    }
    break;
  }
}

// Interrupts ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Interrupt on a timer
void timer_ISR() {
  
  // If selected controller is active
  if (bitRead(states, ctrlr_sel)) {
    
    // Read appropriate sensor to obtain current state
    float CV_new = read_sensor(ctrlr_sel);
    
    // Velocity-mode PI control calculation
    // Calculates the proper manipulation to bring the state into control with the setpoint
    float err_new = setpoint[ctrlr_sel] - CV_new;
    float MV_new = gain[ctrlr_sel]*(err_factor[ctrlr_sel]*err_new - err[ctrlr_sel]) + MV[ctrlr_sel]; // should perhaps cast this to a byte for PWM control, selecting the gain value such that the range of the float fits

    if (ctrlr_sel == 2) { // If the selected controller is for flow
      // Update the total flow counter while two consecutive flow data points are stored, using trapezoidal integration
      flow_tot += duty_cyc*(CV_new + CV[2])/2;
    }
    
    CV[ctrlr_sel] = CV_new;
    err[ctrlr_sel] = err_new;
    MV[ctrlr_sel] = MV_new;
  }

  ctrlr_sel++; // Advances through the controllers each time this ISR is called

  // Distributes reading/writing tasks across the controller cycle
  // On a controller cycle, read the other sensors not involved in the control loops, if the reading other sensors state is active
  if (bitRead(states, 3)) {
    if (ctrlr_sel == 1) {
      read_temphumidity();
    }
    else if (ctrlr_sel == 2) {
      read_pressure();
    }
  }
  
  if (ctrlr_sel == ctrlr_num) {
    ctrlr_sel = 0;
    
    if (bitRead(states, 4)) { // If the data logging state is active, write sensor data to the SD card

      // Construct a comma-separated string containing the sensor data, both that involved in the control loops and that to log only
      String data_str = "";
      for (byte data_sel = 0; data_sel < 3; data_sel++) { // CHANGE IF YOU CHANGE THE SIZE OF CV
        data_str += String(CV[data_sel]) + ",";
      }
      for (byte data_sel = 0; data_sel < 3; data_sel++) { // CHANGE IF YOU CHANGE THE SIZE OF log_data
        data_str += String(log_data[data_sel]);
        if (data_sel < 2) { // CHANGE IF YOU CHANGE THE SIZE OF log_data
          data_str += ",";
        }
      }
      data_file.println(data_str + "\n"); // Write sensor data to current data file
    }
  }
}

// Interrupt for "Select" button
void select_ISR() {
  bitSet(flags, 0);
}

// Interrupt for "Cycle" button
void cycle_ISR() {
  bitSet(flags, 1);
}

// Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Reads the sensor corresponding to the selected control loop and returns a value with the specified units: sccm currently
float read_sensor(byte selector) {
  
  float value;
  switch (selector) {
    case 0: // Internal temp control
      // TODO
      break;
    case 1: // Heatsink temp control
       // TODO
       // read temp difference between ambient air and heatsink base
       //value = (heatsink temp) - log_data[0]
      break;
    case 2: // Flow control
      Wire.requestFrom(0x49, 2);    // request 2 bytes from airflow sensor (address 0x49)

      // Read the two bytes into a 16-bit datatype
      short dout_code; // Okay to use a signed datatype here since the data is only 14-bit
      dout_code = ((short)Wire.read()) << 8;
      dout_code += (short)Wire.read();
      value = (((float)dout_code/16383.0 - 0.5)/0.4)*200.0; // Multiply at end by full-scale flow (200 sccm I think for this sensor)
      break;
  }
  return value;
}

void read_temphumidity() {
  // read into log_data[0:1]
}

void read_pressure() {
  // read into log_data[2]
}

void read_GPS() {
  // read into log_once_data
}

void read_battery() {
  // read into monitor_data
}

byte start_flow_sensor() {
  
  // Upon data requests, the airflow sensor will reply with zeros until it has initialized. Then it sends its serial number on the first two data requests afterward.
  Wire.requestFrom(0x49, 2);    // request 2 bytes from airflow sensor (address 0x49)
  byte c[2];
  c[0] = Wire.read();
  c[1] = Wire.read();

  // If the serial number was received on the last request, print it and read/print the rest of the serial number to prepare the device for receiving real data
  if (c[0] != 0) {
    Wire.requestFrom(0x49, 2);
    Serial.print(F("Flow Sensor Serial Number: "));
    Serial.print(c[0], HEX);
    Serial.print(c[1], HEX);
    c[0] = Wire.read();
    c[1] = Wire.read();
    Serial.print(c[0], HEX);
    Serial.println(c[1], HEX);
    return 0;
  }
  return 1;
}

byte num_files(File dir) {

  byte num = 0;
  File dir_file = dir.openNextFile();
  while (dir_file) {
    if (!dir_file.isDirectory()) {
      num++;
    }
    dir_file.close();
    dir_file = dir.openNextFile();
  }
  return num;
}

// Loop state switch
/*
switch (states) {
  case B000000: // default state
    break;
  case B001011: // controlling temperatures
    break;
  case B101011: // controlling tempuratures during a run
    break;
  case B111011: // controlling temperatures and recording data during a run
    break;
  case B111111: // controlling temperatures and flow and recording data during a run
    break;
  default: // Undefined state; move back to default state
    states = B000000;
    break;
}
*/

// Alternate program reading functions
/*for (byte setting_sel = 0; setting_sel < sizeof(setpoint); setting_sel++) {
  String setting_str;
  char read_char = prgm_file.read();
  while (read_char != '\n' && prgm_file.available > 0) {
    setting_str += read_char;
    read_char = prgm_file.read();
  }
  setpoint[setting_sel] = setting_str.toFloat();
}*/
/*for (byte set_sel = 0; set_sel < 4; set_sel++) { // for each setpoint in the setpoints variable, read the corresponding float from the program file
  for (byte bit_sel = 0; bit_sel < 32; bit_sel += 8) {
    *(setpoint[set_sel] + bit_sel) = prgm_file.read(); // do the pointer math here to write a byte at consecutive positions pointed to by the float 
  }
}*/

// Old button interrupts
/*
// Interrupt for "Cycle" button
void cycle_ISR() {
  switch (UI_loc[0]) {
    case 1:
      if (UI_loc[1] == 0) { // "Programs" is selected in top list; cycle to "Status" in top list
        UI_loc[0] = 2;
      }
      else if (!prgm_file) { // "Return" is selected in program list; cycle to first program in program list
        UI_loc[1] = 1;
        prgm_dir.rewindDirectory();
        prgm_file = prgm_dir.openNextFile();
      }
      else {
        if (UI_loc[2] == 0) { // A program is selected in program list; cycle to next program in program list
          UI_loc[1]++;
          prgm_file = prgm_dir.openNextFile();
        }
        else {
          switch (UI_loc[3]) {
            case 0:
              if (UI_loc[2] == 3) { // The final run status display is selected in run status list; cycle to first run status display in run status list
                UI_loc[2] = 1;
              }
              else { // A run status display is selected in run status list; cycle to next run status display in run status list
                UI_loc[2]++;
              }
              break;
            case 1: // "Finish" is selected in paused run list; cycle to "Resume" in paused run list
              UI_loc[3] = 2;
              break;
            case 2: // "Resume" is selected in paused run list; cycle to "Finish" in paused run list
              UI_loc[3] = 1;
              break;
          }
        }
      }
      break;
    case 2:
      if (UI_loc[1] == 0) { // "Status" is selected in top list; cycle to "Programs" in top list
        UI_loc[0] = 1;
      }
      else {
        if (UI_loc[1] == 3) { // The final status display is selected in status list; cycle to first status display in status list
          UI_loc[1] = 1;
        }
        else { // A status display is selected in status list; cycle to next status display in status list
          UI_loc[1]++;
        }
      }
      break;
  }
}

// Interrupt for "Select" button
void select_ISR() {
  switch (UI_loc[0]) {
    case 1:
      if (UI_loc[1] == 0) { // "Programs" is selected in top list; move into program list and open the first program file
        UI_loc[1] = 1;
        prgm_dir = SD.open(F("/programs")); // Opens the programs directory on the SD card.
        prgm_file = prgm_dir.openNextFile();
      }
      else if (!prgm_file) { // "Return" is selected in program list; move back to top list
        UI_loc[1] = 0;
        prgm_file.close();
        prgm_dir.close();
      }
      else {
        if (UI_loc[2] == 0) { // A program is selected in program list; read the program, set the run state bit and move into run status list
          // read program file
          String setting_str;
          byte setting_sel = 0;
          while (prgm_file.available() > 0) { // While there are more characters to be read:
            char read_char = prgm_file.read();
            if (read_char != '\n') {
              setting_str += read_char;
            }
            else {
              setpoint[setting_sel] = setting_str.toFloat();
              setting_str = "";
              if (setting_sel < sizeof(setpoint)) {
                setting_sel++;
              }
              else {
                break;
              }
            }
          }
          prgm_file.close();
          prgm_dir.close();
          states = B101011;
          UI_loc[2] = 1;
        }
        else {
          switch (UI_loc[3]) {
            case 0: // A run status display is selected in run status list; reset the flow control and data log bits, stop flow, and move into paused run list
              states = B101011;
              MV[3] = 0;
              UI_loc[3] = 1;
              break;
            case 1: // "Finish run" is selected in paused run list; reset the run state bit, increment the sample counter, close the data file, and move back to top list
              states = B001011;
              sample_num++;
              data_file.close();
              UI_loc[3] = 0;
              UI_loc[2] = 0;
              UI_loc[1] = 0;
              break;
            case 2: // "Resume run" is selected in paused run list; set the flow control and data log bits and move back to run status list
              states = B111111;
              UI_loc[3] = 0;
              break;
          }
        }
      }
      break;
    case 2:
      if (UI_loc[1] == 0) { // "Status" is selected in top list; move into status list
        UI_loc[1] = 1;
      }
      else { // A status display is selected in status list; move back to top list
        UI_loc[1] = 0;
        UI_loc[0] = 1;
      }
      break;
  }
}
*/
// Old state change monitors
/*
// switch flow on once temp hits setpoint
if (states == B101011 && UI_loc[3] == 0 && UI_loc[2] > 0 && UI_loc[0] == 1 && abs(CV[0] - setpoint[0]) < 1) { // If the system is cooling to start a run, is not paused, and the system has reached temp setpoint
  // Read all the sensor data to log once
  read_GPS();
  
  data_file = SD.open(String(F("/data/sample")) + String(sample_num) + String(F(".csv")), FILE_WRITE); // Open a new data file for writing
  
  // Write log_once_data to data file
  // Construct a comma-separated string containing the sensor data
  String sensor_data = "";
  for (byte data_sel = 0; data_sel < sizeof(log_once_data); data_sel++) {
    data_string += String(log_once_data[data_sel]);
    if (data_sel < sizeof(log_once_data)-1) {
      data_string += ",";
    }
  }
  data_file.println(data_string + "\n"); // Write sensor data to current data file
  
  states = B111111; // Start flow and recording data
}

// stop run once total flow reaches configured value
if (states == B111111 && UI_loc[3] == 0 && UI_loc[2] > 0 && UI_loc[0] == 1 && flow_tot >= flow_tot_setpoint) { // If the system is currently sampling, is not paused, and the system has reached the total flow setpoint
  states = B001011;
  data_file.close();
  sample_num++; // Increment the collection counter
  flow_tot = 0; // Reset the flow rate integration
  UI_loc[3] = 0;
  UI_loc[2] = 0;
  UI_loc[1] = 0;
}
*/
