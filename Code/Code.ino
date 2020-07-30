/*
 * Author: Adam Broerman
 * Portable DVME firmware written for Teensy 3.2
 * Version: 0.15
 * National Institute of Standards and Technology
 * Fluid Characterization Group | Boulder, CO
*/

/* Notes
 *  Does not compile with the Teensy SD library. It must be deleted from hardware/teensy/avr/libraries.
 */

/* Resources
 *  https://www.arduino.cc/reference/
 *  http://gammon.com.au/interrupts
 *  https://www.pjrc.com/teensy/teensy31.html
 *  https://www.pjrc.com/teensy/pinout.html
 *  https://www.pjrc.com/teensy/teensyduino.html
 *  https://www.pjrc.com/teensy/td_libs.html
 */

/* TODO
 *  Can potentially set CPU at a lower frequency to decrease power consumption
 *  Can potentially switch to i2c_t3 library if more control over the sensor i2c communications is necessary
 *  Can optimize by converting to integer math in the control loops
 *  Potentially switch to char* instead of String() for stability concerns
 *  Safety for when outputs are dangerously close to saturated
 *  Design experiments to determine process gain and time constants, then to assign controller gain and time constants
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <LiquidCrystal.h>
#include <TinyGPS.h>

// Pinout ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

const byte sd_cs = 10; // The SPI selection pin for the SD card
//const byte SD_MISO = 12; // Don't need to declare these
//const byte SD_MOSI = 11;
//const byte SD_SCK = 13;

const byte lcd_rs = 2;
const byte lcd_en = 3;
const byte lcd_d4 = 4;
const byte lcd_d5 = 5;
const byte lcd_d6 = 6;
const byte lcd_d7 = 7;

const byte i2c_sda = 18;
const byte i2c_scl = 19;

const byte select_pin = 8;
const byte cycle_pin = 9;

const byte pwm_pin[] = {20, 21, 22};

//const byte gps_rx = 0; // Don't need to declare these
//const byte gps_tx = 1;

// Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// SD Card Variables
// Use a FAT16 formatted card
File data_file;
File prgm_file;
File prgm_dir;
//byte prgm_num;
unsigned short sample_num; // can store up to 65536 samples (to keep with 8.3 convention, files are named samXXXXX.csv)

// Timer Variables
const unsigned int timer_period = 100000; // microseconds
volatile byte timer_flag;
IntervalTimer ctrlr_timer;

// LCD Display
LiquidCrystal lcd(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7);
const byte lcd_col_num = 20;
const byte lcd_row_num = 4;
const byte degree_sym[8] = {B01111, B01001, B01001, B01111, B00000, B00000, B00000, B00000};

// GPS
TinyGPS gps;

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
byte states = B001011;
byte UI_loc[] = {0, 0}; // {state, window} - Keeps track of where the user is currently in the interface
volatile byte flags = B0100; // Bit 0 - select flag, Bit 1 - cycle flag, Bit 2 - new state flag, Bit 3 - new window flag

// Controller Variables
const byte ctrlr_num = 3;
byte ctrlr_sel = 0;
int duty_cyc;
float ctrlr_gain[] = {0.0, 0.0, 0.0}; // Configure these with lab measurements, assign units
float time_const[] = {0.0, 0.0, 0.0}; // Configure these with lab measurements, assign units
float err_factor[3];
float CV[3];
float err[3];
float MV[3];
float flow_tot;
float log_data[3]; // ambient temp, ambient humidity, ambient pressure
float log_once_data[3]; // latitude, longitude, UTC time
float monitor_data[1]; // battery charge
float setpoint[] = {-10.0, 60.0, 100.0, 500.0}; // *C, *C, mL/min, mL; configured in settings. internal temp, heatsink temp, capillary flow, and total flow setpoints
// If you change the size of CV, log_data, log_once_data, or setpoint, you need to change these variables containing their sizes - they ensure the bounds of the for loops that iterate over them below are correct!
const byte log_data_num = 3;
const byte log_once_data_num = 3;
const byte setpoint_num = 4;

// Program Control ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup() {

  //Serial.begin(115200); // Debug interface
  Serial1.begin(9600); // Serial interface for GPS

  // The internal pullup resistors keep the interrupt pins high when they're not connected to anything.
  // When the button connects them to ground, they're pulled low, and that's detected by the falling ISR
  pinMode(select_pin, INPUT_PULLUP);
  pinMode(cycle_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(select_pin), select_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(cycle_pin), cycle_ISR, FALLING);
  
  // Controller setup calculations
  duty_cyc = ctrlr_num*timer_period; // Controller duty cycle - the interval between reading data points
  for (byte i = 0; i < ctrlr_num; i++) {
    err_factor[i] = 1 + duty_cyc/time_const[i];
    pinMode(pwm_pin[i], OUTPUT);
  }
  ctrlr_timer.begin(timer_ISR, timer_period);

  lcd.begin(lcd_col_num, lcd_row_num);
  lcd.noAutoscroll();
  lcd.print(F("Initializing..."));
  lcd.createChar(0, degree_sym);

  // Attempts to initialize the SD card, and if it fails, stops the program.
  if (!SD.begin(sd_cs)) {
    while (1);
  }
  
  File data_dir = SD.open(F("/data"));
  sample_num = num_files(data_dir); // Counts the number of files in the data directory, excluding subdirectories, to determine where to start numbering new data files
  data_dir.close();

  // Initialize I2C bus
  Wire.setSDA(i2c_sda);
  Wire.setSCL(i2c_scl);
  Wire.begin();
  Wire.setClock(100000); // 100 kHz I2C clock speed
  while (start_flow_sensor()); // Wait for flow sensor to start up and receive its serial number
}

void loop() {

  if (timer_flag) { // If the timer has activated
    
    // If selected controller is active
    if (bitRead(states, ctrlr_sel)) {
      float CV_new = read_sensor(ctrlr_sel); // Read appropriate sensor to obtain current state
      
      // Velocity-mode PI control calculation
      // Calculates the proper manipulation to bring the state into control with the setpoint
      float err_new = setpoint[ctrlr_sel] - CV_new;
      float MV_new = ctrlr_gain[ctrlr_sel]*(err_factor[ctrlr_sel]*err_new - err[ctrlr_sel]) + MV[ctrlr_sel];
      
      if (ctrlr_sel == 2) { // If the selected controller is for flow
        flow_tot += duty_cyc*(CV_new + CV[2])/2; // Update the total flow counter while two consecutive flow data points are stored, using trapezoidal integration
      }

      // Update stored values for next calculation
      CV[ctrlr_sel] = CV_new;
      err[ctrlr_sel] = err_new;
      MV[ctrlr_sel] = MV_new;

      // Send the intervention calculated by the control loop as a PWM output to control power to each device.
      // If you need a more complicated output here tailored for each control loop, switch to a function to which you pass ctrlr_sel, kinda like how read_sensors works.
      // Select the gain value such that the range of the float is from 0 to 255.
      analogWrite(pwm_pin[ctrlr_sel], round(MV_new));
    }
  
    ctrlr_sel++; // Advances through the controllers each time this ISR is called
  
    // Distributes reading/writing tasks across the controller cycle
    if (ctrlr_sel == 1 && bitRead(states, 4)) { // Write data to the SD card, if the recording data state is active
      data_file.flush();
    }
    else if (ctrlr_sel == 2 && bitRead(states, 3)) { // Read the other sensors not involved in the control loops, if the reading other sensors state is active
      read_temphumidity();
      read_pressure();
    }
    else if (ctrlr_sel == ctrlr_num) {
      ctrlr_sel = 0;
      
      if (bitRead(states, 4)) { // If the data logging state is active, write sensor data to the SD card
  
        // Construct a comma-separated string containing the sensor data, both that involved in the control loops and that to log only
        String data_str = "";
        for (byte data_sel = 0; data_sel < ctrlr_num; data_sel++) { // CHANGE if the size of CV is no longer ctrlr_num (which shouldn't happen)
          data_str += String(CV[data_sel]) + ",";
        }
        for (byte data_sel = 0; data_sel < log_data_num; data_sel++) {
          data_str += String(log_data[data_sel]);
          if (data_sel < log_data_num-1) {
            data_str += ",";
          }
        }
        data_file.println(data_str + "\n"); // Write sensor data to current data file
      }
    }
    
    timer_flag = 0;
  }
  
  // This massive conditional tree takes the program to a unique place in the code dependent on the current state, which is determined by the UI_loc, states, flags, and prgm_file variables.
  // If values can change, it continuously updates the screen based on the current state.
  // It executes an action specific to that state if the cycle or the select button was just pressed.
  switch(UI_loc[0]) {
    case 0: // Program list
      if (bitRead(flags, 0)) { // Move to pre-run setpoint display, reading the contents of the selected program file into the setpoint array
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
            if (setting_sel < setpoint_num-1) {
              setting_sel++;
            }
            else {
              break;
            }
          }
        }
        UI_loc[0] = 1;
        flags = B0100;
      }
      else if (bitRead(flags, 1)) { // Cycle to the next program file, returning to the top of the list if at the end
        prgm_file.close();
        prgm_file = prgm_dir.openNextFile();
        //UI_loc[1]++; // This counts as the user cycles through the program file list. It's not used for anything at the moment, but could be useful at some point.
        if (!prgm_file) {
          prgm_dir.rewindDirectory();
          prgm_file = prgm_dir.openNextFile();
          //UI_loc[1] = 0;
        }
        flags = B1000;
      }
      else if (bitRead(flags, 2)) { // Open the program directory and the first program file
        prgm_dir = SD.open(F("/programs")); // Opens the programs directory on the SD card.
        prgm_file = prgm_dir.openNextFile();
        UI_loc[1] = 0;
        flags = B1000;
      }
      else if (bitRead(flags, 3)) { // Display the name of the currently selected program file
        lcd.clear();
        lcd.print(F("Programs:"));
        lcd.setCursor(0, 1);
        lcd.print(prgm_file.name());
        flags = 0;
      }
      
    case 1: // Pre-run setpoint display - allows the user to preview the contents of the program before running it
      if (bitRead(flags, 0)) { // Move to the run status display and start the run. Set state to within run, awaiting internal temp to reach setpoint. Open the data file.
        String prgm_name = prgm_file.name();
        prgm_file.close();
        prgm_dir.close();
        if (!data_file) { // If the data file hasn't been opened
          data_file = SD.open(String(F("/data/sam")) + String(sample_num) + String(F(".csv")), FILE_WRITE); // Open a new data file for writing
          data_file.println("#" + prgm_name + "\n"); // Write program name to current data file. Hashed data lines indicate they contain the program name used in the collection.
          sample_num++; // Increment the collection counter
        }
        states = B101011;
        UI_loc[0] = 2;
        flags = B0100;
      }
      else if (bitRead(flags, 1)) { // Return to program list with the next program selected
        UI_loc[0] = 0;
        // flags is currently B0010; don't change it. That way, the cycle action will be performed within UI_loc[0] = 0, selecting the next program.
      }
      else if (bitRead(flags, 2)) { // Display the currently loaded setpoints
        lcd.clear();
        lcd.print(F("Internal Spt       C"));
        lcd.setCursor(13, 0);
        lcd.print(String(setpoint[0], 1)); // Write the internal temp setpoint with one decimal place - up to 5 characters
        lcd.setCursor(18, 0);
        lcd.write(byte(0)); // Write the custom degree character
        lcd.setCursor(0, 1);
        lcd.print(F("External Spt       C"));
        lcd.setCursor(13, 0);
        lcd.print(String(setpoint[1], 1));
        lcd.setCursor(18, 0);
        lcd.write(byte(0)); // Write the custom degree character
        lcd.setCursor(0, 2);
        lcd.print(F("Flow Spt        sccm"));
        lcd.setCursor(10, 2);
        lcd.print(String(setpoint[2], 2));
        lcd.setCursor(0, 3);
        lcd.print(F("Stop Flow At      mL"));
        lcd.setCursor(13, 3);
        lcd.print(String(setpoint[3], 0));
        flags = 0;
      }
      
    case 2: // Run status display
      if (bitRead(flags, 0)) { // Pause run and move to paused run list.
        UI_loc[0] = 3;
        flags = B0100;
      }
      else if (bitRead(flags, 1)) { // Switch run status window.
        UI_loc[1] = !UI_loc[1];
        flags = B1000;
      }
      else if (bitRead(flags, 2)) { // Ensure the first run status window is selected.
        UI_loc[1] = 0;
        flags = B1000;
      }
      else if (bitRead(flags, 3)) { // Write names, setpoints (if applicable), and units for the current window
        if (UI_loc[1]) {
          lcd.clear();
          lcd.print(F("Ambient Temp       C"));
          lcd.setCursor(18, 0);
          lcd.write(byte(0)); // Write the custom degree character
          lcd.setCursor(0, 1);
          lcd.print(F("Humidity           %")); // TODO units
          lcd.setCursor(0, 2);
          lcd.print(F("Pressure         kPa"));
          lcd.setCursor(0, 3);
          lcd.print(F("Battery Charge     %"));
        }
        else {
          lcd.clear();
          lcd.print(F("Internal      ,    C"));
          lcd.setCursor(15, 0);
          lcd.print(String(setpoint[0], 0));
          lcd.setCursor(18, 0);
          lcd.write(byte(0)); // Write the custom degree character
          lcd.setCursor(0, 1);
          lcd.print(F("External      ,    C"));
          lcd.setCursor(15, 0);
          lcd.print(String(setpoint[1], 0));
          lcd.setCursor(18, 0);
          lcd.write(byte(0)); // Write the custom degree character
          lcd.setCursor(0, 2);
          lcd.print(F("Flow        ,   sccm"));
          lcd.setCursor(13, 2);
          lcd.print(String(setpoint[2], 0));
          lcd.setCursor(0, 3);
          lcd.print(F("Total       ,     mL"));
          lcd.print(String(setpoint[3], 0));
        }
        flags = 0;
      }
      else { // Monitor whether flow limit or temp setpoint has been reached and update values on LCD
        if (states == B111111 && flow_tot >= setpoint[ctrlr_num]) { // If the system is currently sampling, is not paused, and has reached the total flow setpoint, stop run once total flow reaches configured value. Move to program list.
          states = B001011;
          MV[2] = 0; // stop flow AFTER turning flow control off
          data_file.close();
          flow_tot = 0; // Reset the flow rate integration
          UI_loc[0] = 0;
          flags = B0100;
        }
        else if (states == B101011 && abs(CV[0] - setpoint[0]) < 1) { // If the system is cooling to start flow, is not paused, and has reached temp setpoint (to within one degree), switch flow on once temp hits setpoint
          // Read all the sensor data to log once
          read_GPS();
          // Write log_once_data to data file
          // Construct a comma-separated string containing the sensor data
          String data_str = "*"; // Starred data lines indicate they contain log_once_data
          for (byte data_sel = 0; data_sel < log_once_data_num; data_sel++) {
            data_str += String(log_once_data[data_sel]);
            if (data_sel < log_once_data_num-1) {
              data_str += ",";
            }
          }
          data_file.println(data_str + "\n"); // Write sensor data to current data file
          states = B111111; // Start flow and recording data
        }
        if (UI_loc[1]) {
          lcd.setCursor(13, 0);
          lcd.print(String(log_data[0], 1)); // Ambient temperature
          lcd.setCursor(14, 1); // TODO Adjust if units are not %
          lcd.print(String(log_data[1], 1)); // Humidity
          lcd.setCursor(10, 2);
          lcd.print(String(log_data[2], 3)); // Pressure
          lcd.setCursor(16, 3);
          lcd.print(String(monitor_data[0], 0)); // Battery charge
        }
        else {
          lcd.setCursor(9, 0);
          lcd.print(String(CV[0], 1)); // Internal temperature
          lcd.setCursor(9, 1);
          lcd.print(String(CV[1], 1)); // External temperature
          lcd.setCursor(6, 2);
          lcd.print(String(CV[2], 2)); // Flow rate
          lcd.setCursor(7, 3);
          lcd.print(String(flow_tot, 1)); // Total flow
        }
      }
      
    case 3: // Paused run list
      if (bitRead(flags, 0)) { // Select option: either end the run or continue
        if (UI_loc[1]) { // "Resume" is selected; move back to run status display to continue flow. The flow control and data log bits will be set once the temp setpoint is reached (may be on the next loop if paused during flow), and the log_once_data will be written again.
          UI_loc[0] = 2;
        }
        else { // "Finish" is selected; stop the run. Move to program list.
          states = B001011;
          MV[2] = 0; // Just in case flags(0) gets set in between flags(2) getting set and performing the flags(2) actions for UI_loc[0]==3. It shouldn't happen due to the if statements within the ISRs, but it's good to be careful.
          data_file.close();
          flow_tot = 0; // Reset the flow rate integration
          UI_loc[0] = 0;
        }
        flags = B0100;
      }
      else if (bitRead(flags, 1)) { // Switch selected option (window)
        UI_loc[1] = !UI_loc[1];
        flags = B1000;
      }
      else if (bitRead(flags, 2)) { // Set state to within run, but flow not controlled and not recording data. Stop flow.
        states = B101011;
        MV[2] = 0; // stop flow AFTER turning flow control off
        UI_loc[1] = 0;
        lcd.clear();
        lcd.print(F(" Finish"));
        lcd.setCursor(0, 1);
        lcd.print(F(" Resume"));
        flags = B1000;
      }
      else if (bitRead(flags, 3)) { // Set arrow at selected option corresponding to UI_loc[1]
        lcd.setCursor(0, UI_loc[1]);
        lcd.print(F(">"));
        lcd.setCursor(0, !UI_loc[1]);
        lcd.print(F(" "));
        flags = 0;
      }
  }
}

// Interrupts ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Interrupt on a timer
void timer_ISR() {
  timer_flag = 1;
}

// Interrupt for "Select" button
void select_ISR() {
  if (flags == 0) { // Prevents required run-once actions (Bit 2) from not occurring due to an untimely select button press
    flags = B0001;
  }
}

// Interrupt for "Cycle" button
void cycle_ISR() {
  if (flags == 0) { // Prevents required run-once actions (Bit 2) from not occurring due to an untimely cycle button press
    flags = B0010;
  }
}

// Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Reads the sensor corresponding to the selected control loop and returns a value with the specified units: sccm currently
float read_sensor(byte selector) {
  
  float value = 0.0;
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
  // TODO read into monitor_data
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
    //Serial.print(F("Flow Sensor Serial Number: "));
    //Serial.print(c[0], HEX);
    //Serial.print(c[1], HEX);
    c[0] = Wire.read();
    c[1] = Wire.read();
    //Serial.print(c[0], HEX);
    //Serial.println(c[1], HEX);
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

/*void lcd_toplist() {
  lcd.clear();
  lcd.print(F("*Programs"));
  lcd.setCursor(0, 1);
  lcd.print(F(" Status"));
}

void lcd_cyclelist(byte current, byte next) {
  lcd.setCursor(0, current);
  lcd.print(F(" "));
  lcd.setCursor(0, next);
  lcd.print(F("*"));
}

void lcd_status(byte screen) { // screen is 1-indexed so that it corresponds to UI_loc[2]
  switch (screen) {
  case 1: // Temps and flows
    lcd.clear();
    lcd.print(F("Internal Temp"));
    lcd.setCursor(0, 1);
    lcd.print(F("External Temp"));
    lcd.setCursor(0, 2);
    lcd.print(F("Flow"));
    lcd.setCursor(0, 3);
    lcd.print(F("Total Flow"));
    break;
  case 2: // Setpoints
    lcd.clear();
    lcd.print(F("Int Temp Stpt"));
    lcd.setCursor(0, 1);
    lcd.print(F("Ext Temp Stpt"));
    lcd.setCursor(0, 2);
    lcd.print(F("Flow Stpt"));
    lcd.setCursor(0, 3);
    lcd.print(F("Tot Flow Stpt"));
    break;
  case 3: // Ambient conditions and data file number
    lcd.clear();
    lcd.print(F("Ambient Temp"));
    lcd.setCursor(0, 1);
    lcd.print(F("Pressure"));
    lcd.setCursor(0, 1);
    lcd.print(F("Humidity"));
    lcd.setCursor(0, 1);
    lcd.print(F("Sample Num"));
    break;
  }
}
void lcd_status_vals(byte screen) { // screen is 1-indexed so that it corresponds to UI_loc[2]
  switch (screen) {
  case 1: // Temps and setpoints
    
    break;
  case 2: // Flows and setpoints
    
    break;
  case 3: // Ambient conditions and data file number
    
    break;
  }
}*/

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

// Old interface conditional tower
/*switch (UI_loc[0]) {
  case 1:
    if (UI_loc[1] == 0) { // "Programs" is selected in top list
      if (bitRead(flags, 0)) { // Move into program list and open the first program file
        UI_loc[1] = 1;
        prgm_dir = SD.open(F("/programs")); // Opens the programs directory on the SD card.
        prgm_file = prgm_dir.openNextFile();
        flags = 0; // Clears both button flags so that no weird behavior arises from simultaneous button presses
        lcd.clear();
        lcd.print(F("Programs:"));
        lcd.setCursor(0, 1);
        lcd.print(prgm_file.name());
      }
      else if (bitRead(flags, 1)) { // Cycle to "Status" in top list
        UI_loc[0] = 2;
        flags = 0;
        lcd_cyclelist(0, 1);
      }
    }
    else if (!prgm_file) { // "Return" is selected in program list
      if (bitRead(flags, 0)) { // Move back to top list
        UI_loc[1] = 0;
        prgm_dir.close();
        flags = 0;
        lcd_toplist();
      }
      else if (bitRead(flags, 1)) { // Cycle to first program in program list
        UI_loc[1] = 1;
        prgm_dir.rewindDirectory();
        prgm_file = prgm_dir.openNextFile();
        flags = 0;
        lcd.clear();
        lcd.print(F("Programs:"));
        lcd.setCursor(0, 1);
        lcd.print(prgm_file.name());
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
              if (setting_sel < setpoint_num-1) {
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
          lcd_status(UI_loc[2]); // Print the names for the first run status display
        }
        else if (bitRead(flags, 1)) { // Cycle to next program in program list
          UI_loc[1]++;
          prgm_file.close();
          prgm_file = prgm_dir.openNextFile();
          flags = 0;
          if (prgm_file) {
            lcd.setCursor(0, 1);
            String write_str = prgm_file.name();
            lcd.print(write_str);
            byte name_len = write_str.length();
            if (name_len < lcd_col_num) { // Clear any residual characters past the printed name
              write_str = " ";
              for (byte i = 0; i < lcd_col_num-name_len-1; i++) {
                write_str += " ";
              }
              lcd.setCursor(name_len, 1)
              lcd.print(write_str);
            }
          }
          else { // The last program file was selected; print the option to return to the top list
            lcd.clear();
            lcd.print(F("Return"));
          }
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
            lcd.clear();
            lcd.print(F("*Finish"));
            lcd.setCursor(0, 1);
            lcd.print(F(" Resume"));
          }
          else if (bitRead(flags, 1)) { // Cycle to next run status display in run status list
            if (UI_loc[2] < 3) {
              UI_loc[2]++;
            }
            else {
              UI_loc[2] = 1;
            }
            flags = 0;
            lcd_status(UI_loc[2]); // Print the names for the next run status display
          }
          else {
            lcd_status_vals(UI_loc[2]); // Update status values on the LCD
            if (states == B111111 && flow_tot >= setpoint[ctrlr_num]) { // If the system is currently sampling, is not paused, and has reached the total flow setpoint, stop run once total flow reaches configured value
              states = B001011;
              MV[3] = 0;
              data_file.close();
              flow_tot = 0; // Reset the flow rate integration
              UI_loc[3] = 0;
              UI_loc[2] = 0;
              UI_loc[1] = 0;
              lcd_toplist();
            }
            else if (states == B101011 && abs(CV[0] - setpoint[0]) < 1) { // If the system is cooling to start flow, is not paused, and has reached temp setpoint (to within one degree), switch flow on once temp hits setpoint
              // Read all the sensor data to log once
              read_GPS();
              if (!data_file) { // If the data file hasn't been opened
                data_file = SD.open(String(F("/data/sam")) + String(sample_num) + String(F(".csv")), FILE_WRITE); // Open a new data file for writing
                sample_num++; // Increment the collection counter
              }
              // Write log_once_data to data file
              // Construct a comma-separated string containing the sensor data
              String data_str = "*"; // Starred data lines indicate they contain log_once_data
              for (byte data_sel = 0; data_sel < log_once_data_num; data_sel++) {
                data_str += String(log_once_data[data_sel]);
                if (data_sel < log_once_data_num-1) {
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
            flow_tot = 0; // Reset the flow rate integration
            UI_loc[3] = 0;
            UI_loc[2] = 0;
            UI_loc[1] = 0;
            flags = 0;
            lcd_toplist();
          }
          else if (bitRead(flags, 1)) { // Cycle to "Resume" in paused run list
            UI_loc[3] = 2;
            flags = 0;
            lcd_cyclelist(0, 1);
          }
          break;
        case 2: // "Resume" is selected in paused run list
          if (bitRead(flags, 0)) { // Move back to run status list to continue flow. The flow control and data log bits will be set once the temp setpoint is reached (may be on the next loop if paused during flow), and the log_once_data will be written again.
            UI_loc[3] = 0;
            flags = 0;
            lcd_status(UI_loc[2]);
          }
          if (bitRead(flags, 1)) { // Cycle to "Finish" in paused run list
            UI_loc[3] = 1;
            flags = 0;
            lcd_cyclelist(1, 0);
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
        lcd_status(UI_loc[1]);
      }
      if (bitRead(flags, 1)) { // Cycle to "Programs" in top list
        UI_loc[0] = 1;
        flags = 0;
        lcd_cyclelist(1, 0);
      }
    }
    else { // A status display is selected in status list
      if (bitRead(flags, 0)) { // Move back to top list
        UI_loc[1] = 0;
        UI_loc[0] = 1;
        flags = 0;
        lcd_toplist();
      }
      else if (bitRead(flags, 1)) { // Cycle to next status display in status list
        if (UI_loc[1] < 3) {
          UI_loc[1]++;
        }
        else {
          UI_loc[1] = 1;
        }
        flags = 0;
        lcd_status(UI_loc[1]);
      }
      else {
        lcd_status_vals(UI_loc[1]);
      }
    }
    break;
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
