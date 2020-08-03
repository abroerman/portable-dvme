/*
 * Author: Adam Broerman
 * Portable DVME firmware written for Teensy 3.2
 * Version: 0.17
 * National Institute of Standards and Technology
 * Fluid Characterization Group | Boulder, CO
*/

/* Notes
 *  Does not compile with the Teensy SD library. It must be deleted from hardware/teensy/avr/libraries.
 *  This code has not yet been tested on a physical device, since it has yet to be built.
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
 *  A control loop for the heatsink fan may be overkill. It may be most efficient to always run the fan at full to maximize heat dumped from the TEC hot side to maximize TEC efficiency, rather than trying to keep hot side and ambient temp difference at a setpoint. An easy way to hack this code to achieve this would be to set that temp difference setpoint to zero or even a negative value. But, for the most efficient code, that control loop should be removed entirely if it is deemed unnecessary. You can use the built-in alert system in the MCP9808 with an interrupt to detect if the fan needs to come on.
 *  Potentially switch to char* instead of String() for stability concerns - use a 100 character buffer or something
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
//const byte SD_MOSI = 11; // Don't need to declare these
//const byte SD_MISO = 12;
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
uint16_t sample_num; // can store up to 65536 samples (to keep with 8.3 convention, files are named samXXXXX.csv)

// Timer Variables
const unsigned int timer_period = 1000; // milliseconds
volatile byte timer_flag;
IntervalTimer ctrlr_timer;

// LCD Display
LiquidCrystal lcd(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7);
const byte lcd_col_num = 20;
const byte lcd_row_num = 4;
const byte degree_sym[8] = {B01111, B01001, B01001, B01111, B00000, B00000, B00000, B00000};

// GPS
TinyGPS gps;

// Sensors
const byte flow_addr = 0x49;
const byte temphumidity_addr = 0x44;
const byte pressure_addr = 0x28;
const byte ext_temp_addr = 0x19;
const byte int_temp_addr = 0x18;

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
float log_data[3]; // ambient temp - *C, ambient humidity - %, ambient pressure - kPa
float pos_data[2]; // latitude, longitude
int year_data;
byte time_data[5]; // month, day, hour, minute, second
float monitor_data[1]; // battery charge
float setpoint[] = {-10.0, 10.0, 100.0, 500.0}; // *C, *C, mL/min, mL; configured in settings. internal temp, heatsink temp, capillary flow, and total flow setpoints
// If you change the size of CV, log_data, log_once_data, or setpoint, you need to change these variables containing their sizes - they ensure the bounds of the for loops that iterate over them below are correct!
const byte log_data_num = 3;
const byte time_data_num = 5;
const byte setpoint_num = 4;

// Program Control ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup() {

  //Serial.begin(115200); // Debug interface

  lcd.begin(lcd_col_num, lcd_row_num);
  lcd.noAutoscroll();
  lcd.print(F("Initializing..."));
  lcd.createChar(0, degree_sym);
  
  Serial1.begin(9600); // Serial interface for GPS

  // The internal pullup resistors keep the interrupt pins high when they're not connected to anything.
  // When the button connects them to ground, they're pulled low, and that's detected by the falling ISR
  pinMode(select_pin, INPUT_PULLUP);
  pinMode(cycle_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(select_pin), select_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(cycle_pin), cycle_ISR, FALLING);

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
  if (!start_temp_sensor(int_temp_addr)) { // start internal temp sensor
    while(1);
  }
  if (!start_temp_sensor(ext_temp_addr)) { // start external temp sensor
    while(1);
  }
  while (start_flow_sensor()); // Wait for flow sensor to start up and receive its serial number
  start_temphumidity_sensor(); // start ambient temp and humidity sensor
  
  // Controller setup calculations
  duty_cyc = ctrlr_num*timer_period; // milliseconds - controller duty cycle - the interval between reading data points
  for (byte i = 0; i < ctrlr_num; i++) {
    err_factor[i] = 1 + duty_cyc/time_const[i];
    pinMode(pwm_pin[i], OUTPUT);
  }
  ctrlr_timer.begin(timer_ISR, timer_period*1000);
}

void loop() {

  if (timer_flag) { // If the timer has activated
    
    // If selected controller is active
    if (bitRead(states, ctrlr_sel)) {
      float CV_new = 0.0;
      switch (ctrlr_sel) { // Read appropriate sensor to obtain current state
        case 0:
          CV_new = read_temp(int_temp_addr);
          break;
        case 1:
          CV_new = read_temp(ext_temp_addr) - log_data[0]; // controlled variable is the temperature DIFFERENCE between TEC hot side and ambient air, since ambient air is what's cooling the TEC hot side.
          break;
        case 2:
          CV_new = read_flow();
          break;
      }
      
      // Velocity-mode PI control calculation
      // Calculates the proper manipulation to bring the state into control with the setpoint
      float err_new = setpoint[ctrlr_sel] - CV_new;
      float MV_new = ctrlr_gain[ctrlr_sel]*(err_factor[ctrlr_sel]*err_new - err[ctrlr_sel]) + MV[ctrlr_sel]; // Select the gain value such that the range of the float is from 0 to 255.
      
      if (ctrlr_sel == 2) { // If the selected controller is for flow
        flow_tot += duty_cyc*(CV_new + CV[2])/120000; // mL - Update the total flow counter while two consecutive flow data points are stored, using trapezoidal integration. duty_cyc*((CV_new + CV[2])/2)/((1000000 us/s)*(60 s/min))
      }

      // Update stored values for next calculation
      CV[ctrlr_sel] = CV_new;
      err[ctrlr_sel] = err_new;
      MV[ctrlr_sel] = MV_new;

      // Send the intervention calculated by the control loop as a PWM output to control power to each device.
      // If you need a more complicated output here tailored for each control loop, switch to a function to which you pass ctrlr_sel, kinda like how read_sensors works.
      analogWrite(pwm_pin[ctrlr_sel], round(MV_new));
    }
  
    ctrlr_sel++; // Advances through the controllers each time this ISR is called
  
    // Distributes reading/writing tasks across the controller cycle
    if (ctrlr_sel == 1 && bitRead(states, 3)) { // Read the other sensors not involved in the control loops, if the reading other sensors state is active
      read_temphumidity(); // Read ambient temp right before the ext temp control loop for the most up-to-date value
      read_pressure();
    }
    else if (ctrlr_sel == ctrlr_num) {
      ctrlr_sel = 0;
      
      if (bitRead(states, 4)) { // If the data logging state is active, write sensor data to the SD card
  
        // Construct a comma-separated string containing the sensor data, both that involved in the control loops and that to log only
        String data_str = "";
        for (byte data_sel = 0; data_sel < ctrlr_num; data_sel++) { // CHANGE if the size of CV is no longer ctrlr_num (which shouldn't happen)
          data_str += String(CV[data_sel], 3) + ",";
        }
        for (byte data_sel = 0; data_sel < log_data_num; data_sel++) {
          data_str += String(log_data[data_sel], 3);
          if (data_sel < log_data_num-1) {
            data_str += ",";
          }
        }
        data_file.println(data_str + "\n"); // Write sensor data to current data file
      }
      data_file.flush();
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
          lcd.print(F("Humidity           %"));
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
          // Write position and time data to data file
          data_file.println("*" + String(pos_data[0], 5) + "," + String(pos_data[1], 5) + "\n"); // Write position data to current data file. // Starred data lines indicate they contain position data
          // Construct a comma-separated string containing the sensor data
          String data_str = "$" + String(year_data); // Dollared data lines indicate they contain time data (time is money, friend!)
          for (byte data_sel = 0; data_sel < time_data_num; data_sel++) {
            data_str += "," + String(time_data[data_sel]);
          }
          data_file.println(data_str + "\n"); // Write sensor data to current data file
          states = B111111; // Start flow and recording data
        }
        if (UI_loc[1]) {
          lcd.setCursor(13, 0);
          lcd.print(String(log_data[0], 1)); // Ambient temperature
          lcd.setCursor(14, 1);
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

// Adapted from the Adafruit MCP9808 library
float read_temp(byte addr) {
  float temp = 0.0;
  uint16_t dout_code = read16(0x05, addr); // Read the temperature register

  if (dout_code != 0xFFFF) {
    temp = dout_code & 0x0FFF;
    temp /= 16.0;
    if (dout_code & 0x1000)
      temp -= 256;
  }

  return temp;
}

float read_flow() {
  Wire.requestFrom(flow_addr, (byte)2);    // request 2 bytes from airflow sensor (address 0x49)
  // Read the two bytes into a 16-bit datatype
  uint16_t dout_code;
  dout_code = Wire.read();
  dout_code <<= 8;
  dout_code += Wire.read();
  return (((float)dout_code/16383.0 - 0.5)/0.4)*200.0; // Multiply at end by full-scale flow (200 sccm I think for this sensor)
}

// Adapted from the Adafruit SHT31 library
bool read_temphumidity() {
  uint8_t readbuffer[6];
  uint8_t len = sizeof(readbuffer);

  Wire.beginTransmission(temphumidity_addr); // Write the command to read with high reproducibility and clock stretch disabled
  Wire.write(0x24);
  Wire.write(0x00);
  if (Wire.endTransmission()) { // return false if there was an error
    return false;
  }

  delay(20); // wait for sensor reading

  size_t recv_num = Wire.requestFrom(temphumidity_addr, len);
  if (recv_num != len) {
    // Not enough data available to fulfill our obligation!
    return false;
  }
  for (byte i = 0; i < len; i++) { // Read the returned data
    readbuffer[i] = Wire.read();
  }

  // Not performing the CRC check here - refer to the Adafruit library for the code

  // read into log_data[0:1], converting from the values in readbuffer
  int32_t stemp = (int32_t)(((uint32_t)readbuffer[0] << 8) | readbuffer[1]);
  // simplified (65536 instead of 65535) integer version of:
  // temp = (stemp * 175.0f) / 65535.0f - 45.0f;
  stemp = ((4375 * stemp) >> 14) - 4500;
  log_data[0] = (float)stemp / 100.0f; // actual temp

  uint32_t shum = ((uint32_t)readbuffer[3] << 8) | readbuffer[4];
  // simplified (65536 instead of 65535) integer version of:
  // humidity = (shum * 100.0f) / 65535.0f;
  shum = (625 * shum) >> 12;
  log_data[1] = (float)shum / 100.0f; // actual humidity

  return true;
}

void read_pressure() {
  // read into log_data[2]
  Wire.requestFrom(pressure_addr, (byte)2);    // request 2 bytes from pressure sensor (address 0x28)
  // Read the two bytes into a 16-bit datatype
  uint32_t dout_code;
  dout_code = Wire.read();
  dout_code <<= 8;
  dout_code += Wire.read();
  if (!bitRead(dout_code, 15)) { // If the data is not stale and the pressure sensor is not in a diagnostic condition
    // integer math instead of (dout_code - 0.1f * 16384) * 206.843f / (0.8 * 16384)
    dout_code = ((dout_code * 258553) >> 14) - 25855; // According to the i2c communication technical note, this may instead need to be dout_code = (dout_code * 206843) >> 14;
    log_data[2] = (float)dout_code / 1000.0f; // actual pressure in kPa
  }
}

void read_GPS() {
  // read into log_once_data
  // use TinyGPS library and examples
  while (Serial1.available()) {
    if (gps.encode(Serial1.read())) { // process gps messages
      unsigned long fix_age;
      gps.f_get_position(&pos_data[0], &pos_data[1], &fix_age);
      gps.crack_datetime(&year_data, &time_data[0], &time_data[1], &time_data[2], &time_data[3], &time_data[4], NULL, &fix_age);
    }
  }
}

void read_battery() {
  // TODO read into monitor_data
}

// Adapted from the Adafruit MCP9808 library
bool start_temp_sensor(byte addr) {
  if (read16(0x06, addr) != 0x0054) { // Checks the manufacturer ID
    return false;
  }
  if (read16(0x07, addr) != 0x0400) { // Checks the devide ID
    return false;
  }
  // Default power-up resolution is 0.0625*C. If you want a lower resolution for faster read times, change here.
  //write16(0x01, 0x0, addr); // Write 0 to the config register - unnecessary since it contains zero on startup.
  return true;
}

bool start_flow_sensor() {
  // Upon data requests, the airflow sensor will reply with zeros until it has initialized. Then it sends its serial number on the first two data requests afterward.
  Wire.requestFrom(flow_addr, (byte)2);    // request 2 bytes from airflow sensor (address 0x49)
  byte c[2];
  c[0] = Wire.read();
  c[1] = Wire.read();
  
  // If the serial number was received on the last request, print it and read/print the rest of the serial number to prepare the device for receiving real data
  if (c[0] != 0) {
    delay(10);
    Wire.requestFrom(flow_addr, (byte)2);
    //Serial.print(F("Flow Sensor Serial Number: "));
    //Serial.print(c[0], HEX);
    //Serial.print(c[1], HEX);
    c[0] = Wire.read();
    c[1] = Wire.read();
    //Serial.print(c[0], HEX);
    //Serial.println(c[1], HEX);
    return false;
  }
  return true;
}

// Adapted from the Adafruit SHT31 library
uint16_t start_temphumidity_sensor() {
  Wire.beginTransmission(temphumidity_addr); // Write the command to soft reset
  Wire.write(0x30);
  Wire.write(0xA2);
  if (Wire.endTransmission()) { // return false if there was an error
    return false;
  }
  delay(10);
  
  uint8_t readbuffer[3];
  uint8_t len = sizeof(readbuffer);
  
  Wire.beginTransmission(temphumidity_addr); // Write the command to read device status
  Wire.write(0xF3);
  Wire.write(0x2D);
  if (Wire.endTransmission()) { // return false if there was an error
    return false;
  }

  size_t recv_num = Wire.requestFrom(temphumidity_addr, len);
  if (recv_num != len) {
    // Not enough data available to fulfill our obligation!
    return false;
  }
  for (byte i = 0; i < len; i++) { // Read the returned data
    readbuffer[i] = Wire.read();
  }

  uint16_t stat = readbuffer[0];
  stat <<= 8;
  stat |= readbuffer[1];
  return stat;
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

// Adapted from the Adafruit MCP9808 library
uint16_t read16(uint8_t reg, uint8_t addr) {
  uint16_t val = 0xFFFF;

  Wire.beginTransmission(addr);
  Wire.write(reg);

  if (!Wire.endTransmission()) {
    Wire.requestFrom(addr, (uint8_t)2);
    val = Wire.read();
    val <<= 8;
    val |= Wire.read();
  }

  return val;
}
