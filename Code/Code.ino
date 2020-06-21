/*

*/

/* TODO
  Assign units to controller parameters
  Develop conversions from controller units to actually control the inputs
  Controller safety - if state or input is about to overflow
  Design experiments to determine process gain and time constants, then to assign controller gain and time constants
*/

#include <Wire.h>

// Timer Variables
const unsigned int timer_period = 10; // Update once you decide on the hardware

/* State Machine Variables
Bit 1 - controlling internal temp?
Bit 2 - controlling heatsink temp?
Bit 3 - controlling capillary flow?
Bit 4 - recording data?
*/
byte states = 0;

// Controller Variables
const byte ctrlr_num = 3;
volatile byte sel = 0;
int duty_cyc;
int gain[] = {0, 0, 0}; // Configure these with lab measurements, assign units
unsigned int time_const[] = {0, 0, 0}; // Configure these with lab measurements, assign units
float err_factor[3];
unsigned int setpoint[] = {0, 0, 0}; // Configure these with lab measurements, also enable them to be changed in settings
volatile float CV[3];
volatile float err[3];
volatile float MV[3];

void setup() {
  
  // Controller setup calculations
  duty_cyc = ctrlr_num*timer_period; // Controller duty cycle - the interval between reading data points
  for (byte i = 0; i < ctrlr_num; i++) {
    err_factor[i] = 1 + duty_cyc/(float)time_const[i];
  }

  Wire.begin();
  Wire.setClock(100000);
  while (start_flow_sensor()); // Wait for flow sensor to start up and receive its serial number
}

void loop() {
  
}

// Interrupt on a timer

void timer_ISR() {
  
  // If selected controller is active
  if (bitRead(states, sel)) {
    
    // Read appropriate sensor to obtain current state
    float CV_new = read_sensor(sel);
    
    // Velocity-mode PI control calculation
    // Calculates the proper manipulation to bring the state into control with the setpoint
    float err_new = setpoint[sel] - CV_new;
    float MV_new = gain[sel]*(err_factor[sel]*err_new - err[sel]) + MV[sel];
    
    CV[sel] = CV_new;
    err[sel] = err_new;
    MV[sel] = MV_new;
  }
  
  // Cycles through the controllers each time this ISR is called
  sel++;
  if (sel == ctrlr_num) {
    sel = 0;
    
    if (bitRead(states, 4)) {
      // Write to SD
    }
  }
}

// Reads the sensor corresponding to the selected control loop and returns a value with the specified units
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
