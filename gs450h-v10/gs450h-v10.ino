/* Basic software to run the Lexus GS450H hybrid transmission and inverter using the open source V1 controller
 * Take an analog throttle signal and converts to a torque command to MG1 and MG2
 * Feedback provided over USB serial
 * V3  - Reverse added
 * V4  - CAN specific to BMW E65 735i project
 * V5  - WiFi connection on USART2 at 19200 baud
 * V6  - add ISA CAN shunt connectivity. Note V2 hardware only. 
 * V7  - add HV precharge and control- oil pump relay=midpack and precharge contactor, out1= main contactor.
 * V8  - stripped down rewrite by CS. basic functionality.
 * V9  - add serial control for settings (CS)
 * V10 - gear shifting (icomplete / untested) (CS)
 * 
 * Copyright 2019 T.Darby , D.Maguire, C.Smurthwaite
 * openinverter.org
 * evbmw.com
 */

#include <Metro.h>
#include <due_wire.h>
#include <Wire_EEPROM.h>

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
#define TransSL1     47 // Transmission Solenoid 1
#define TransSL2     44 // Transmission Solenoid 2
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
#define CONFIG_VERSION 1
struct
{
  uint16_t version;
  uint16_t precharge_voltage;
  uint16_t max_torque_fwd;
  uint16_t max_torque_rev;
  uint16_t pedal_min;
  uint16_t pedal_max;
  uint16_t regen_factor;
  uint16_t regen_limit;
  uint16_t throttle_exp;
} config;

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
uint16_t dc_bus_voltage=0,temp_inv_water=0,temp_inv_inductor=0;

// Useful to have these globally available
int16_t mg1_torque = 0;
int16_t mg2_torque = 0;
int16_t mg1_speed = 0;
int16_t mg2_speed = 0;

// This value switched to 1 when the inverter has been configured and is ready to use.
uint8_t inv_initialized = 0;
// This value switches to 1 then the main contactor closes.
uint8_t precharge_complete = 0;

// Current ratio, set when shifting completes
#define RATIO_HIGH 0
#define RATIO_LOW 1
uint8_t ratio_current;
// Is a shift currently in progress?
uint8_t ratio_state;
// If we're shifting, when did the process begin?
uint32_t ratio_change_started;

// Based on gear and throttle position, calculate desired torque.
uint16_t get_torque(uint8_t gear)
{
  // Normalize throttle 1 input to 0-1000
  int throttle = analogRead(Throt1Pin);
  throttle = map(throttle, config.pedal_min, config.pedal_max, 0, 1000);
  throttle = constrain(throttle, 0, 1000);

  // If enabled, create an exponential curve that gives more control of low speed / regen.
  if(config.throttle_exp) throttle = throttle * throttle / 1000;

  // Calculate regen amount based on % of mg1 speed and maximum limit
  int regen = mg1_speed * config.regen_factor / 100;
  regen = constrain(regen, -config.regen_limit, config.regen_limit);

  // Map throttle pedal curve so that minimum is full regen and max is full torque
  uint16_t torque;
  if(gear==DRIVE)   torque = map(throttle, 0, 1000, regen,  config.max_torque_fwd);
  if(gear==REVERSE) torque = map(throttle, 0, 1000, regen, -config.max_torque_rev);
  torque = 0;  // If we're not in FWD or REV default to no torque.

  // If a gear shift is in progress, additional shenanigans are in order
  // here to ensure parts of the transmission dont end up in the road.
  if(ratio_state) {
    uint32_t in_progress_time = millis() - ratio_change_started;
    if(in_progress_time < 200) {
      // For the first 200ms we just reduce torque to zero
      torque = 0;
    } else if(in_progress_time < 400) {
      // The next 200ms is dedicated to engaging the gears
      digitalWrite(TransSL1, !ratio_current);
      digitalWrite(TransSL2, !ratio_current);
      torque = 0;
    } else if(in_progress_time < 650) {
      // During the final 250ms, torque is gradually increased back to full
      torque = constrain(torque, (in_progress_time - 400) * -14, (in_progress_time - 400) * 14);
    } else {
      // Shifting complete
      ratio_current = !ratio_current;
      ratio_state = 0;
    }
  }
  return(torque);
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

  // Oil pump speed
  AnalogOut(OilPumpPWM, 180);        // Hardcoded to 70% for now

  Serial1.begin(250000);
  Serial2.begin(19200); // Wifi Module
  SerialUSB.begin(115200);

  // I don't know what these magic constants do, but I assume
  // they are necessary for the sync serial port to work.
  PIOA->PIO_ABSR |= 1<<17;
  PIOA->PIO_PDR |= 1<<17;
  USART0->US_MR |= 1<<4 | 1<<8 | 1<<18;

  // Black magic from previous gurus
  htm_data[63]=(MAXREGEN)&0xFF;   // Max regen
  htm_data[64]=((MAXREGEN)>>8);
  htm_data[65]=(MAXCURRENT)&0xFF; // Max discharge
  htm_data[66]=((MAXCURRENT)>>8);

  // Read configuration
  EEPROM.read(0, config);
  if (config.version != CONFIG_VERSION)
  {
    config.version = CONFIG_VERSION;
    config.precharge_voltage = 2000;
    config.max_torque_fwd = 0;
    config.max_torque_rev = 0;
    config.pedal_min = 2000;
    config.pedal_max = 10000;
    config.regen_factor = 0;
    config.regen_limit = 0;
    config.throttle_exp = 0;
    EEPROM.write(0, config);
  }
}

// Close the main contactor once inverter DC bus voltage reaches defined value.
void precharge() {
    if (dc_bus_voltage > config.precharge_voltage) {
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
    while(Serial1.available()) { Serial1.read(); delay(1); }
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

void print_menu() {
  SerialUSB.println("");
  SerialUSB.println("************ Available Commands ************");
  SerialUSB.println(" ? - Print this menu");
  SerialUSB.println(" q - Print inverter status");
  SerialUSB.println(" w - Print configuration data");
  SerialUSB.println(" e - Calibrate minimum throttle.");
  SerialUSB.println(" r - Calibrate maximum throttle.");
  SerialUSB.println(" t - Set max forward torque (0-3500)");
  SerialUSB.println(" y - Set max reverse torque (0-3500)");
  SerialUSB.println(" u - Set regen factor (0-100+)");
  SerialUSB.println(" i - Set max regen (0-3500)");
  SerialUSB.println(" o - Set normal throttle curve");
  SerialUSB.println(" p - Set exponential throttle curve");
  SerialUSB.println(" v - Set precharge voltage");
  SerialUSB.println(" z - Save configuration data to EEPROM memory");
  SerialUSB.println("********************************************");
}

void print_config() {
  SerialUSB.println("");
  SerialUSB.println("Configuration");
  SerialUSB.println("=============");
  SerialUSB.print("Precharge:     ");
  SerialUSB.println(config.precharge_voltage);
  SerialUSB.print("Max Torque FWD:");
  SerialUSB.println(config.max_torque_fwd);
  SerialUSB.print("Max Torque REV:");
  SerialUSB.println(config.max_torque_rev);
  SerialUSB.print("Pedal min:     ");
  SerialUSB.println(config.pedal_min);
  SerialUSB.print("Pedal max:     ");
  SerialUSB.println(config.pedal_max);
  SerialUSB.print("Regen factor:  ");
  SerialUSB.println(config.regen_factor);
  SerialUSB.print("Regen limit:   ");
  SerialUSB.println(config.regen_limit);
  SerialUSB.print("Exp Throttle   ");
  SerialUSB.println(config.throttle_exp);
}

void print_status() {
  SerialUSB.println("");
  SerialUSB.println("Status");
  SerialUSB.println("======");
  SerialUSB.print("DC Bus:        ");
  SerialUSB.println(dc_bus_voltage);
  SerialUSB.print("Throttle Pos:  ");
  SerialUSB.println(analogRead(Throt1Pin));
  SerialUSB.print("MG1 Speed:     ");
  SerialUSB.println(mg1_speed);
  SerialUSB.print("MG2 Speed:     ");
  SerialUSB.println(mg2_speed);
  SerialUSB.print("Water Temp:    ");
  SerialUSB.println(temp_inv_water);
  SerialUSB.print("Inductor Temp: ");
  SerialUSB.println(temp_inv_inductor);
  SerialUSB.print("Raw MG1 Temp:  ");
  SerialUSB.println(analogRead(MG1Temp));
  SerialUSB.print("Raw MG2 Temp:  ");
  SerialUSB.println(analogRead(MG2Temp));
}

char rx_buffer[16];
uint8_t rx_buffer_offset = 0;

void read_serial() {
  uint8_t rx_byte;
  while(SerialUSB.available()) {
    rx_byte = SerialUSB.read();
    if(rx_byte == 0x0a) {
      // Received a newline, process the buffer
      switch(rx_buffer[0]) {
        case 0: // If nothing received, print the menu just in case
        case '?':
          print_menu();
          break;
        case 'q':
          print_status();
          break;
        case 'w':
          print_config();
          break;
        case 'e':
          config.pedal_min = analogRead(Throt1Pin) + 10;
          SerialUSB.println("Throttle low position calibrated.");
          break;
        case 'r':
          config.pedal_max = analogRead(Throt1Pin);
          SerialUSB.println("Throttle high position calibrated.");
          break;
        case 't':
          config.max_torque_fwd = atoi(rx_buffer+1);
          SerialUSB.println("Max forward torque set.");
          break;
        case 'y':
          config.max_torque_rev = atoi(rx_buffer+1);
          SerialUSB.println("Max reverse torque set.");
          break;
        case 'u':
          config.regen_factor = atoi(rx_buffer+1);
          SerialUSB.println("Regen factor set.");
          break;
        case 'i':
          config.regen_limit = atoi(rx_buffer+1);
          SerialUSB.println("Regen limit set.");
          break;
        case 'o':
          config.throttle_exp = 0;
          SerialUSB.println("Throttle curve set to regular.");
          break;
        case 'p':
          config.throttle_exp = 1;
          SerialUSB.println("Throttle curve set to exponential.");
          break;
        case 'v':
          config.precharge_voltage = atoi(rx_buffer+1);
          SerialUSB.println("Precharge voltage set.");
          break;
        case 'z':
          EEPROM.write(0, config);
          SerialUSB.println("Configuration saved.");
          break;
      }
      memset(rx_buffer, 0, 16);
      rx_buffer_offset = 0;
    } else {
      // Received a non-newline, append it to the buffer
      if(rx_buffer_offset < 16)
        rx_buffer[rx_buffer_offset++] = rx_byte;
    }
  }
}

// Highly speculative gear ratio shifting function
void shift_gear() {
  if(ratio_state) return; // Ignore shifting requests if already in progress.
  if(ratio_current == RATIO_LOW) {
    // It's always safe to switch to high ratio
    ratio_state = 1;
    ratio_change_started = millis();
  } else {
    // It's only safe to switch to low ratio at low rpm
    if(mg2_speed < -3500 || mg1_speed > 3500) return;
    ratio_state = 1;
    ratio_change_started = millis();
  }
}

void loop() {
  // If we're not precharged yet, prepare to close contactor.
  if(!precharge_complete) precharge();
  // Call main inverter code. Monitor and control.
  monitor_inverter();

  read_serial();
}
