/* Basic software to run the Lexus GS450H hybrid transmission and inverter using the open source V1 controller
 * Take an analog throttle signal and converts to a torque command to MG1 and MG2
 * Feedback provided over USB serial
 * V3  - Reverse added
 * V4  - CAN specific to BMW E65 735i project
 * V5  - WiFi connection on USART2 at 19200 baud
 * V6  - add ISA CAN shunt connectivity. Note V2 hardware only. 
 * V7  - add HV precharge and control- oil pump relay=midpack and precharge contactor, out1= main contactor.
 * V10  - stripped down rewrite by CS. Complete basic functionality and basic web interface.
 * 
 * Copyright 2019 T.Darby , D.Maguire, C.Smurthwaite
 * openinverter.org
 * evbmw.com
 */

#include <Metro.h>
#include <due_wire.h>
#include <Wire_EEPROM.h>

// Two timers are used, one to drive the inverter and a second to send data to the wifi module
Metro inverter_timer = Metro(2);
Metro wifi_timer     = Metro(20);

// There numbers don't mhave any special meaning, they're just labels for convenience.
#define REVERSE 1
#define NEUTRAL 2
#define DRIVE   3

// Define pin mappings
#define LED          13 // Just an LED
#define InvReq       22 // Requests status data from inverter
#define Out1         50 // This pin is used to close the main contactor
#define OilPumpPWM    2 // Oil pump speed control
#define OilPumpPower 33 // This pin closes the negative and precharge contactors. Always on, could probably be put to better use.
#define InvPower     34 // Powers up the inverter. Always on, could probably be put to better use.
#define TransSL1     47 // Transmission Solenoid 1
#define TransSL2     44 // Transmission Solenoid 2
#define TransSP      45 // Oil pressure solenoid, we likely don't need to use this if we control the oil pump

#define IN1          6  // High when gear level is in FWD position (D/B)
#define IN2          7  // High when gear level is in REV position (R)
#define Brake_In    62  // Not currently used. I will likely use for gear lever "B" position.

#define TransPB1    40  // Oil pressure sensor
#define TransPB2    43  // Oil pressure sensor
#define TransPB3    42  // Oil pressure sensor

#define Throt1Pin   A0  // Throttle 1 input.
#define Throt2Pin   A1  // Throttle 2 input. Not yet implemented.

#define OilpumpTemp A7  // Temperature sensor
#define TransTemp   A4  // Temperature sensor
#define MG1Temp     A5  // Temperature sensor
#define MG2Temp     A6  // Temperature sensor

// Hard limits
#define MG1MAXSPEED 12000
#define MAXCURRENT 27500
#define MAXREGEN -5000

// Tuneable settings
#define CONFIG_VERSION 1
struct {
  uint16_t version;
  uint16_t precharge_voltage;
  uint16_t max_torque_fwd;
  uint16_t max_torque_rev;
  uint16_t pedal_min;
  uint16_t pedal_max;
  uint16_t regen_factor;
  uint16_t regen_limit;
  uint16_t throttle_exp;
  uint16_t oil_pump_pwm;
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
int16_t dc_bus_voltage=0,temp_inv_water=0,temp_inv_inductor=0;

// Useful to have these globally available
int16_t mg1_torque = 0;
int16_t mg2_torque = 0;
int16_t mg1_speed = 0;
int16_t mg2_speed = 0;

// Gear ratio solenoid states
uint8_t trans_sl1 = 0;
uint8_t trans_sl2 = 0;

// This value switched to 1 when the inverter has been configured and is ready to use.
uint8_t inv_initialized = 0;
// This value switches to 1 then the main contactor closes.
uint8_t precharge_complete = 0;

// Based on gear and throttle position, calculate desired torque.
void calculate_torque()
{
  uint8_t gear = get_gear();
  // Normalize throttle 1 input to 0-1000
  int throttle = analogRead(Throt1Pin);
  throttle = map(throttle, config.pedal_min, config.pedal_max, 0, 1000);
  throttle = constrain(throttle, 0, 1000);

  // If enabled, create an exponential curve that gives more control of low speed / regen.
  if(config.throttle_exp) throttle = throttle * throttle / 1000;

  // Calculate regen amount based on % of mg2 speed and maximum limit
  int regen = -mg2_speed * config.regen_factor / 100;
  regen = constrain(regen, -config.regen_limit, config.regen_limit);

  // Map throttle pedal curve so that minimum is full regen and max is full torque
  uint16_t torque;
  if(gear==DRIVE)        torque = map(throttle, 0, 1000, regen,  config.max_torque_fwd);
  else if(gear==REVERSE) torque = map(throttle, 0, 1000, regen, -config.max_torque_rev);
  else                   torque = 0;  // If we're not in FWD or REV default to no torque.

  // Hard cut torque if MG1 is overspeed
  if((mg1_speed>MG1MAXSPEED)||(-mg1_speed>MG1MAXSPEED)) torque=0;

  // MG1 torque is set to MG2 * 1.25.
  mg2_torque = torque;
  mg1_torque=((torque*5)/4);

  // We don't use mg1 in reverse.
  if(gear==REVERSE)mg1_torque=0;
}

void setup() {
  // Set up OUTPUT pins
  pinMode(LED, OUTPUT);
  pinMode(InvReq, OUTPUT);
  pinMode(OilPumpPower, OUTPUT);
  pinMode(InvPower, OUTPUT);
  pinMode(Out1, OUTPUT);
  pinMode(TransSL1, OUTPUT);
  pinMode(TransSL2, OUTPUT);
  pinMode(TransSP, OUTPUT);

  // Set defalt pin states
  digitalWrite(InvReq, LOW);         // This initial state is unimportant
  digitalWrite(InvPower, HIGH);      // Turn on inverter
  digitalWrite(OilPumpPower, HIGH);  // Begin HV precharge
  digitalWrite(Out1, LOW);           // Turn off main contactor
  digitalWrite(TransSL1, LOW);       // Turn off at startup.
  digitalWrite(TransSL2, LOW);       // Turn off at startup.
  digitalWrite(TransSP, LOW);        // Turn off at startup, not used yet.

  // Set up INPUT pins
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);
  pinMode(Brake_In, INPUT);

  pinMode(TransPB1, INPUT);
  pinMode(TransPB2, INPUT);
  pinMode(TransPB3, INPUT);

  // Oil pump speed
  analogWrite(OilPumpPWM, 0);

  Serial1.begin(250000);
  Serial2.begin(115200); // Wifi Module
  
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
    config.oil_pump_pwm = 0;
    EEPROM.write(0, config);
  }
}

// Close the main contactor once inverter DC bus voltage reaches defined value.
void precharge() {
    if (inv_initialized && (dc_bus_voltage > config.precharge_voltage)) {
        delay(500);
        digitalWrite(Out1,HIGH);
        precharge_complete = 1;
    }
}

// Send a control packet to the inverter to set the torque of MG1
// and MG2 based on desired torque.
void control_inverter() {
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

// Every 2ms toggle request line to request inverter status
void poll_inverter() {
  if(inverter_timer.check()) {
    digitalWrite(InvReq,!digitalRead(InvReq));
  }
}

// Wait for 100 bytes of status data, when a frame has been received, parse it.
// Return true when a frame has been received from the inverter.
uint8_t monitor_inverter() {
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
    return(1);
  }
  return(0);
}

uint8_t config_allowed = 0;

// Lines must begin with \t and end with \n
// Following the \t the first 2 bytes are an ascii number, specifying the parameter / command
void process_serial(char* buffer) {
  if(buffer[0] == '\t' && buffer[1] == '2' && buffer[2] == '0') {
    EEPROM.write(0, config);
  }
  if(buffer[0] == '\t' && buffer[1] == '0') {
    switch(buffer[2]) {
    case '0':
      config.pedal_min = atoi(buffer+4);
      break;
    case '1':
      config.pedal_max = atoi(buffer+4);
      break;
    case '2':
      config.max_torque_fwd = atoi(buffer+4);
      break;
    case '3':
      config.max_torque_rev = atoi(buffer+4);
      break;
    case '4':
      config.regen_factor = atoi(buffer+4);
      break;
    case '5':
      config.regen_limit = atoi(buffer+4);
      break;
    case '6':
      config.throttle_exp = atoi(buffer+4);
      break;
    case '7':
      config.precharge_voltage = atoi(buffer+4);
      break;
    case '8':
      config.oil_pump_pwm = atoi(buffer+4);
      break;
    case '9':
      trans_sl1 = (atoi(buffer+4) << 0) & 1;
      trans_sl2 = (atoi(buffer+4) << 1) & 1;
      break;
    }    
  }
  memset(buffer, 0, 32);
}

char rx_buffer_wifi[32];
uint8_t rx_buffer_offset_wifi = 0;

void read_serial() {
  uint8_t rx_byte;
  while(Serial2.available()) {
    rx_byte = Serial2.read();
    if(rx_byte == 0x0a) {
      // Received a newline, process the buffer
      process_serial(rx_buffer_wifi);
      rx_buffer_offset_wifi = 0;
    } else {
      // Received a non-newline, append it to the buffer
      if(rx_buffer_offset_wifi < 32)
        rx_buffer_wifi[rx_buffer_offset_wifi++] = rx_byte;
    }
  }
}

// Send parameters to wifi on a loop. HTTP page will be  dynamically generated accordingly.
// All lines begin with \t followed by 2-digit ID, comma, type (1=RW,2=RO), comma, descripton, comma, value (integer).
uint8_t wifi_index;
void write_wifi() {
  if(wifi_timer.check()) {
    Serial2.print("\t");
    switch(wifi_index) {
    case 0: // Config and read-write parameters
      Serial2.print("00,1,Pedal Min,");
      Serial2.println(config.pedal_min);
      break;
    case 1:
      Serial2.print("01,1,Pedal Max,");
      Serial2.println(config.pedal_max);
      break;
    case 2:
      Serial2.print("02,1,Max Torque Fwd,");
      Serial2.println(config.max_torque_fwd);
      break;
    case 3:
      Serial2.print("03,1,Max Torque Rev,");
      Serial2.println(config.max_torque_rev);
      break;
    case 4:
      Serial2.print("04,1,Regen Factor,");
      Serial2.println(config.regen_factor);
      break;
    case 5:
      Serial2.print("05,1,Regen Limit,");
      Serial2.println(config.regen_limit);
      break;
    case 6:
      Serial2.print("06,1,Throttle Mode,");
      Serial2.println(config.throttle_exp);
      break;
    case 7:
      Serial2.print("07,1,Precharge Volage,");
      Serial2.println(config.precharge_voltage);
      break;
    case 8:
      Serial2.print("08,1,Oil Pump PWM,");
      Serial2.println(config.oil_pump_pwm);
      break;
    case 9:
      Serial2.print("09,1,Solenoid State,");
      Serial2.println((trans_sl2 << 1) | trans_sl1);
      break;
    case 10: // Read-only status parameters
      Serial2.print("10,2,DC Bus Voltage,");
      Serial2.println(dc_bus_voltage);
      break;
    case 11:
      Serial2.print("11,2,MG1 Speed,");
      Serial2.println(mg1_speed);
      break;
    case 12:
      Serial2.print("12,2,MG2 Speed,");
      Serial2.println(mg2_speed);
      break;
    case 13:
      Serial2.print("13,2,Inverter Water Temp,");
      Serial2.println(temp_inv_water);
      break;
    case 14:
      Serial2.print("14,2,MG1 Temp,");
      Serial2.println(analogRead(MG1Temp));
      break;
    case 15:
      Serial2.print("15,2,MG2 Temp,");
      Serial2.println(analogRead(MG2Temp));
      break;
    case 16:
      Serial2.print("16,2,Oil Pump Temp,");
      Serial2.println(analogRead(OilpumpTemp));
      break;
    case 17:
      Serial2.print("17,2,Transmission Temp,");
      Serial2.println(analogRead(TransTemp));
      break;
    case 18:
      Serial2.print("18,2,Oil Pressure State,");
      Serial2.print(digitalRead(TransPB1));
      Serial2.print(" ");
      Serial2.print(digitalRead(TransPB1));
      Serial2.print(" ");
      Serial2.println(digitalRead(TransPB1));
      break;
    case 19:
      Serial2.print("19,2,Pedal Position,");
      Serial2.println(analogRead(Throt1Pin));
      break;
    case 20:
      Serial2.println("20,1,Save to EEPROM,SAVE");
      break;
    case 21:
      Serial2.println("21,0,,");
      break;
    }
    wifi_index = (wifi_index + 1) % 22;
  }
}

void loop() {
  // If we're not precharged yet, prepare to close contactor.
  precharge();

  // Poll the motor at regular intervals to request status frames
  poll_inverter();

  // Wait for a status frame from the inverter
  if(monitor_inverter()) {
    // Every time we receive a status frame, calculate torque demand, and send a control frame.
    calculate_torque();
    control_inverter();
  }

  // Set the speed of the electric oil pump
  analogWrite(OilPumpPWM, config.oil_pump_pwm);

  // Set gear ratio solenoids
  digitalWrite(TransSL1, trans_sl1);
  digitalWrite(TransSL2, trans_sl2);

  // Wait for control data from USB or wifi
  read_serial();
  // Write status and configuration data to wifi
  write_wifi();
}
