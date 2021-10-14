/* Basic software to run the Lexus GS450H hybrid transmission and inverter using the open source V1 controller
 * Take an analog throttle signal and converts to a torque command to MG1 and MG2
 * Feedback provided over USB serial
 * V3 - Reverse added
 * V4 - CAN specific to BMW E65 735i project
 * V5 - WiFi connection on USART2 at 19200 baud
 * V6 - add ISA CAN shunt connectivity. Note V2 hardware only. 
 * V7 - add HV precharge and control- oil pump relay=midpack and precharge contactor, out1= main contactor.
 * V8 - stripped down rewrite by CS. basic functionality.
 * 
 * Copyright 2019 T.Darby , D.Maguire, C.Smurthwaite
 * openinverter.org
 * evbmw.com
 */

#include <Metro.h>
#include <due_wire.h>

// Two timers are used, one to drive the inverter and a second to print useful information to a USB port.
Metro inverter_timer = Metro(2);
Metro debug_timer    = Metro(1000);

// There numbers don't mhave any special meaning, they're just labels for convenience.
#define REVERSE 1
#define NEUTRAL 2
#define DRIVE   3

// Define pin mappings
#define pin_inv_req  22 // Requests status data from inverter
#define Out1         50 // This pin is used to close the main contactor
#define OilPumpPWM    2 // Oil pump speed control, not yet implemented
#define OilPumpPower 33 // This pin closes the negative and precharge contactors. Always on, could probably be put to better use.
#define InvPower     34 // Powers up the inverter. Always on, could probably be put to better use.
#define TransSL1     47 // Transmission Solenoid 1, not yet implemented
#define TransSL2     44 // Transmission Solenoid 2, not yet implemented
#define TransSP      45 // Please can someone explain to me what this is for? Not yet implemented.

#define IN1          6  // High when gear level is in FWD position (D/B)
#define IN2          7  // High when gear level is in REV position (R)
#define Brake_In    62  // Not currently used. I will likely use for gear lever "B" position.

#define TransPB1    40  // Oil pressure sensor? Not yet implemented.
#define TransPB2    43  // Oil pressure sensor? Not yet implemented.
#define TransPB3    42  // Oil pressure sensor? Not yet implemented.

#define Throt1Pin   A0  // Throttle 1 input.
#define Throt2Pin   A1  // Throttle 2 input. Not yet implemented.

#define OilpumpTemp A7  // Oil pump temperature. Not yet implemented.
#define TransTemp   A4  // Oil pump temperature. Not yet implemented.
#define MG1Temp     A5  // Oil pump temperature. Not yet implemented.
#define MG2Temp     A6  // Oil pump temperature. Not yet implemented.


// Hard limits
#define MG2MAXSPEED 10000
#define MAXCURRENT 27500
#define MAXREGEN -5000

// Tuneable settings
#define MAXTORQUEFWD 500 // Max forward torque (up to 3500)
#define MAXTORQUEREV 250 // Max reverse torque (up to 3500)
#define PEDAL_MIN 125    // Min position of throttle 1
#define PEDAL_MAX 533    // Max position of throttle 1
#define REGEN_FACTOR 10  // Regen torque as a % of engine speed
#define REGEN_LIMIT 200  // Maximum regen (up to 3500)

#define HVPreset    40   // Voltage at which to enable main contactor. Don't leave this set to 40.

// Read IN1 and IN2 and decide whether we're in FWD, REV, or neutral
uint8_t get_gear() {
  if(digitalRead(IN1))
      return(DRIVE);
  if(digitalRead(IN2))
      return(REVERSE);
  return(NEUTRAL);
}

// Memory for data packets and default contents. MTH data comes from the inverter. HTM messages (setup and normal) are sent to it.
uint8_t mth_data[100];
uint8_t htm_data_setup[80]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,25,0,0,0,0,0,0,0,128,0,0,0,128,0,0,0,37,1};
uint8_t htm_data[80]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,255,0,0,0,0,0,0,0,0,0}; 

// Only used for diagnostic output
float dc_bus_voltage=0,temp_inv_water=0,temp_inv_inductor=0;

// Useful to have these globally available
int16_t mg1_torque=0,
        mg2_torque=0,
        mg1_speed=-1,
        mg2_speed=-1;

// This value switched to 1 when the inverter has been configured and is ready to use.
uint8_t inv_initialized = 0;
// This value switches to 1 then the main contactor closes.
uint8_t precharge_complete = 0;

// Based on gear and throttle position, calculate desired torque.
uint16_t get_torque(uint8_t gear)
{
  // Normalize throttle 1 input to 0-1000
  int throttle = analogRead(Throt1Pin);
  throttle = constrain(throttle, PEDAL_MIN, PEDAL_MAX);
  throttle = map(throttle, PEDAL_MIN, PEDAL_MAX, 0, 1000);

  // Uncomment to create an exponential curve that gived more control of low sleep / regen.
  //throttle = throttle * throttle / 1000;

  // Calculate regen amount based on % of mg1 speed and maximum limit
  int regen = mg1_speed * REGEN_FACTOR / 100;
  regen = constrain(regen, -REGEN_LIMIT, REGEN_LIMIT);

  // Map throttle pedal curve so that minimum is full regen and max is full torque
  if(gear==DRIVE)   return map(throttle, 0, 1000, regen, MAXTORQUEFWD);
  if(gear==REVERSE) return map(throttle, 0, 1000, regen, -MAXTORQUEREV);
  return 0;  // If we're not in FWD or REV default to no torque.
}

void setup() {
  // Set up OUTPUT pins
  pinMode(13, OUTPUT);              // LED
  pinMode(pin_inv_req, OUTPUT);
  pinMode(OilPumpPower, OUTPUT);
  pinMode(InvPower, OUTPUT);
  pinMode(Out1, OUTPUT);
  pinMode(TransSL1, OUTPUT);
  pinMode(TransSL2, OUTPUT);
  pinMode(TransSP, OUTPUT);

  // Set defalt pin states
  digitalWrite(pin_inv_req, LOW);    // This initial state is unimportant
  digitalWrite(InvPower, HIGH);      // Turn on inverter
  digitalWrite(OilPumpPower, HIGH);  // Begin HV precharge
  digitalWrite(Out1, LOW);           // Turn off main contactor
  digitalWrite(TransSL1, LOW);       // Turn off at startup, not used yet.
  digitalWrite(TransSL2, LOW);       // Turn off at startup, not used yet.
  digitalWrite(TransSP, LOW);        // Turn off at startup, not used yet.

  // Set up INPUT pins
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);
  pinMode(Brake_In, INPUT);

  pinMode(TransPB1, INPUT);
  pinMode(TransPB2, INPUT);
  pinMode(TransPB3, INPUT);

  Serial1.begin(250000);
  Serial2.begin(19200); //setup serial 2 for wifi access
  SerialUSB.begin(115200);

  // I don't know what these magic constants do, but I assume they are necessary
  // for the sync serial port to work.
  PIOA->PIO_ABSR |= 1<<17;
  PIOA->PIO_PDR |= 1<<17;
  USART0->US_MR |= 1<<4 | 1<<8 | 1<<18;

  // Black magic from previous gurus
  htm_data[63]=(MAXREGEN)&0xFF;   // Max regen
  htm_data[64]=((MAXREGEN)>>8);
  htm_data[65]=(MAXCURRENT)&0xFF; // Max discharge
  htm_data[66]=((MAXCURRENT)>>8);
}

// Close the main contactor once inverter DC bus voltage reaches defined value.
void precharge() {
    if (dc_bus_voltage > HVPreset) {
        digitalWrite(Out1,HIGH);
        precharge_complete = 1;
    }
}

// Send a control packet to the inverter to set the torque of MG1
// and MG2 based on desired torque.
void send_inverter_control() {
  // We need to know if we're in FWD, REV or neutral
  uint8_t gear=get_gear();
  
  // Call get_torque for main torque calculation
  mg2_torque=get_torque(gear); // -3500 (reverse) to 3500 (forward)

  // MG1 torque is set to MG2 * 1.25. Please can someone tell me where this ratio came from?
  mg1_torque=((mg2_torque*5)/4);

  // Hard cut torque for overspeed
  if((mg2_speed>MG2MAXSPEED)||(mg2_speed<-MG2MAXSPEED))mg2_torque=0;

  // We don't use mg1 in reverse. Why not?
  if(gear==REVERSE)mg1_torque=0;
  
  // Black magic from previous gurus. Thanks! I'd love some more info on these data structures.
  int speedSum = mg2_speed + mg1_speed;
  speedSum /= 113;
  htm_data[0]=(byte)speedSum;
  htm_data[75] = (mg1_torque * 4) & 0xFF;
  htm_data[76] = ((mg1_torque * 4) >> 8);
  
  //mg1
  htm_data[5] = (mg1_torque * -1) & 0xFF; // Negative!
  htm_data[6] = ((mg1_torque * -1) >> 8);
  htm_data[11] = htm_data[5];
  htm_data[12] = htm_data[6];
  
  //mg2
  htm_data[26] = (mg2_torque) & 0xFF; // Positive
  htm_data[27] = ((mg2_torque) >> 8);
  htm_data[32] = htm_data[26];
  htm_data[33] = htm_data[27];
  
  // Calculate simple checksum
  uint16_t htm_checksum = 0;
  for(byte i=0; i<78; i++) htm_checksum += htm_data[i];
  htm_data[78] = htm_checksum & 0xFF;
  htm_data[79] = htm_checksum >> 8;
  
  // Send control frame (init frames until inverter responds, then normal control frames)
  if(inv_initialized) Serial1.write(htm_data, 80);
  else Serial1.write(htm_data_setup, 80);
}

void monitor_inverter() {
  // Every 2ms toggle request line to request inverter status
  if(inverter_timer.check()) {
    digitalWrite(pin_inv_req,!digitalRead(pin_inv_req));
  }

  // Wait for 100 bytes of status data
  if(Serial1.available() >= 100) {
    Serial1.readBytes(mth_data, 100);
    // Discard any unexpected extra bytes to ensure we stay in sync
    while(Serial1.available()) Serial1.read();
    // Check message checksum
    uint16_t mth_checksum = 0;
    for(int i=0;i<98;i++) mth_checksum += mth_data[i];
    uint8_t mth_good = (mth_checksum == (mth_data[98] | (mth_data[99]<<8)));

    // Calculate status information from received data
    if(mth_good) {
      dc_bus_voltage = (((mth_data[82] | mth_data[83] << 8) - 5) / 2);
      temp_inv_water = (mth_data[42] | mth_data[43] << 8);
      temp_inv_inductor = (mth_data[86] | mth_data[87] << 8);
      mg1_speed = mth_data[6] | mth_data[7] << 8;
      mg2_speed = mth_data[31] | mth_data[32] << 8;
      if(mth_data[1])
        inv_initialized = 1; // Inverter now initialized
    }
    // Every time we receive a status frame, send a control frame.
    send_inverter_control();
  }
}

// Dump some useful data to the USB port for debugging.
void print_debug_data() {
  SerialUSB.print("DC Bus:        ");
  SerialUSB.println(dc_bus_voltage);
  SerialUSB.print("MG1 Speed:     ");
  SerialUSB.println(mg1_speed);
  SerialUSB.print("MG2 Speed:     ");
  SerialUSB.println(mg2_speed);
  SerialUSB.print("Water Temp:    ");
  SerialUSB.println(temp_inv_water);
  SerialUSB.print("Inductor Temp: " );
  SerialUSB.println(temp_inv_inductor);
  SerialUSB.println("");
  SerialUSB.println("");
}

void loop() {
  // Print debug data from time to time.
  if(debug_timer.check()) print_debug_data();
  // If we're not precharged yet, prepare to close contactor.
  if(!precharge_complete) precharge();
  // Call main inverter code. Monitor and control.
  monitor_inverter();
}
