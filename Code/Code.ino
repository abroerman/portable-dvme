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

// Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// SD Card Variables
const byte chip_sel = 4; // The SPI selection pin for the SD card
File data_file;
byte prgm_num;
unsigned int sample_num;

// Timer Variables
const unsigned int timer_period = 10; // Update once you decide on the hardware

// State Machine Variables
/*
Bit 0 - controlling internal temp?
Bit 1 - controlling heatsink temp?
Bit 2 - controlling capillary flow?
Bit 3 - recording data?
Bit 4 - within a run?
*/
byte states = 0;
byte UI_loc[] = {1, 0, 0, 0}; // Keeps track of where the user is currently in the interface

// Controller Variables
const byte ctrlr_num = 3;
volatile byte ctrlr_sel = 0;
int duty_cyc;
int gain[] = {0, 0, 0}; // Configure these with lab measurements, assign units
unsigned int time_const[] = {0, 0, 0}; // Configure these with lab measurements, assign units
float err_factor[3];
unsigned int setpoint[] = {0, 0, 0}; // Configured in settings
volatile float CV[3];
volatile float err[3];
volatile float MV[3];
volatile float flow_tot;

// Program Control ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup() {
  
  // Controller setup calculations
  duty_cyc = ctrlr_num*timer_period; // Controller duty cycle - the interval between reading data points
  for (byte i = 0; i < ctrlr_num; i++) {
    err_factor[i] = 1 + duty_cyc/(float)time_const[i];
  }

  // Attempts to initialize the SD card, and if it fails, stops the program.
  if (!SD.begin(chip_sel)) {
    while (1);
  }
  
  File prgm_dir = SD.open(F("/programs")); // Opens the programs directory on the SD card.
  prgm_num = num_files(prgm_dir); // Counts the number of files in the programs directory, excluding subdirectories, to determine how many program options to display
  prgm_dir.close();
  
  File data_dir = SD.open(F("/data"));
  sample_num = num_files(data_dir); // Counts the number of files in the data directory, excluding subdirectories, to determine where to start numbering new data files
  data_dir.close();

  // Initialize I2C bus
  Wire.begin();
  Wire.setClock(100000); // 100 kHz I2C clock speed
  while (start_flow_sensor()); // Wait for flow sensor to start up and receive its serial number
}

void loop() {

  // switch this to depend on UI_loc

  if (states == B10011 && UI_loc[2] == 1 && UI_loc[0] == 1) { // If the system is cooling to start a run
    if (abs(CV[0] - setpoint[0]) < 1) { // If the system has reached temp setpoint
      data_file = SD.open(String(F("/data/sample")) + String(sample_num) + String(F(".csv")), FILE_WRITE); // Open a new data file for writing
      states = B11111; // Start flow and recording data
    }
  }

  // stop flow once total flow reaches configured value

  /*
  // screen updates based on current state
  switch (states) {
    case B00000: // default state
      break;
    case B00011: // controlling temperatures
      break;
    case B10011: // controlling tempuratures during a run
      break;
    case B11011: // controlling temperatures and recording data during a run
      break;
    case B11111: // controlling temperatures and flow and recording data during a run
      break;
    default: // Undefined state; move back to default state
      states = B00000;
      break;
  }
  */
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
  
  // Cycles through the controllers each time this ISR is called
  ctrlr_sel++;
  if (ctrlr_sel == ctrlr_num) {
    ctrlr_sel = 0;
    
    if (bitRead(states, 3)) { // If the data logging state is active, write sensor data to the SD card

      // Construct a comma-separated string containing the sensor data
      String sensor_data = "";
      for (byte sensor_sel = 0; sensor_sel < sizeof(CV); sensor_sel++) {
        sensor_data += String(CV[sensor_sel]);
        if (sensor_sel < sizeof(CV)-1) {
          sensor_data += ",";
        }
      }

      data_file.println(sensor_data); // Write sensor data to current data file
    }
  }
}

// Interrupt for "Cycle" button
void cycle_ISR() {
  
}

// Interrupt for "Select" button
void select_ISR() {
  switch (UI_loc[0]) {
    case 1:
      if (UI_loc[1] == 0) { // "Programs" is selected in top list; move into program list
        UI_loc[1] = 1;
      }
      else if (UI_loc[1] == prgm_num+1) { // "Return" is selected in program list; move back to top list
        UI_loc[1] = 0;
      }
      else {
        if (UI_loc[2] == 0) { // A program is selected in program list; read the program, set the run state bit and move into run status list
          read_program(UI_loc[1]);
          states = B10011;
          UI_loc[2] = 1;
        }
        else {
          switch (UI_loc[3]) {
            case 0: // A run status display is selected in run status list; reset the flow control and data log bits, stop flow, and move into paused run list
              states = B10011;
              MV[3] = 0;
              UI_loc[3] = 1;
              break;
            case 1: // "Finish run" is selected in paused run list; reset the run state bit, increment the sample counter, close the data file, and move back to top list
              states = B00011;
              sample_num++;
              data_file.close();
              UI_loc[3] = 0;
              UI_loc[2] = 0;
              UI_loc[1] = 0;
              break;
            case 2: // "Resume run" is selected in paused run list; set the flow control and data log bits and move back to run status list
              states = B11111;
              UI_loc[3] = 0;
              break;
          }
        }
      }
      break;
    case 2:
      //UI_loc[1] = !UI_loc[1]; // If 0, "Status" is selected in top list; move into status list. If nonzero, a status display is selected in status list; move back to top list
      
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

// Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Reads the sensor corresponding to the selected control loop and returns a value with the specified units: sccm currently
float read_sensor(byte selector) {
  
  float value;
  switch (selector) {
    case 0: // Internal temp control
      
      break;
    case 1: // Heatsink temp control
      
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

void read_program(byte prgm_sel) {
  
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
  while (true) {
    File program = dir.openNextFile();
    if (!program) {
      break;
    }
    if (!program.isDirectory()) {
      num++;
    }
    program.close();
  }
  return num;
}
