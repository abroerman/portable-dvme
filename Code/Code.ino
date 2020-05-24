/*

*/

/* TODO
  Assign units to controller parameters
  Develop conversions from controller units to actually control the inputs
  Controller safety - if state or input is about to overflow
  Design experiments to determine process gain and time constants, then to assign controller gain and time constants
*/

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
int gain[] = {0, 0, 0}; // Configure these, assign units
unsigned int time_const[] = {0, 0, 0}; // Configure these, assign units
float err_factor[3];
unsigned int setpoint[] = {0, 0, 0}; // Configure these, also enable them to be changed by setting
volatile float CV[3];
volatile float err[3];
volatile float MV[3];

void setup() {
  
  // Controller setup calculations
  duty_cyc = ctrlr_num*timer_period;
  for (byte i = 0; i < ctrlr_num; i++) {
    err_factor[i] = 1 + duty_cyc/(float)time_const[i];
  }
  
}

void loop() {
  
}

// Interrupt on a timer

void timer_ISR() {
  
  // If selected controller is active
  if (bitRead(states, sel)) {
    
    // Read appropriate sensor to obtain current state
    CV_new = read_sensor(sel);
    
    // Velocity-mode PI control calculation
    // Calculates the proper manipulation to bring the state into control with the setpoint
    err_new = setpoint[sel] - CV_new;
    MV_new = gain[sel]*(err_factor[sel]*err_new - err[sel]) + MV_sel;
    
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
float read_sensor(selector) {
  
  float value;
  switch (selector) {
    case 0:
      
      break;
    case 1:
      
      break;
    case 2:
      
      break;
  }
  return value;
}
