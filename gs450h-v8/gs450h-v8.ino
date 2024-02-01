/* Basic software to run the Lexus GS450H hybrid transmission and inverter using the open source V1 controller
   Take an analog throttle signal and converts to a torque command to MG1 and MG2
   Feedback provided over USB serial
   V3  - Reverse added
   V4  - CAN specific to BMW E65 735i project
   V5  - WiFi connection on USART2 at 19200 baud
   V6  - add ISA CAN shunt connectivity. Note V2 hardware only.
   V7  - add HV precharge and control- oil pump relay=midpack and precharge contactor, out1= main contactor.
   V8  - Refactor and simplity by CS. Support for standard openinverter serial interface and wifi

   Copyright 2019 T.Darby , D.Maguire, C.Smurthwaite
   openinverter.org
   evbmw.com
*/

#include <Metro.h>
#include <due_wire.h>
#include <Wire_EEPROM.h>

// A timer to periodically poll the inverter for status data
Metro inverter_timer = Metro(2);

// These numbers don't mhave any special meaning, they're just labels for convenience.
#define REVERSE 1
#define NEUTRAL 2
#define DRIVE   3

// Define pin mappings
#define PIN_LED            13 // Just an LED
#define PIN_REQ            22 // Requests status data from inverter
#define PIN_OUT1           50 // This pin is used to close the main contactor
#define PIN_OIL_PUMP_PWM    2 // Oil pump speed control
#define PIN_OIL_PUMP_POWER 33 // This pin closes the negative and precharge contactors. Always on, could probably be put to better use.
#define PIN_INV_POWER      34 // Powers up the inverter. Always on, could probably be put to better use.
#define PIN_TRANS_SL1      47 // Transmission Solenoid 1
#define PIN_TRANS_SL2      44 // Transmission Solenoid 2
#define PIN_TRANS_SP       45 // Oil pressure solenoid, not used

#define PIN_IN1            6  // High when gear level is in FWD position (D/B)
#define PIN_IN2            7  // High when gear level is in REV position (R)
#define PIN_BRAKE_IN      62  // High level selects low gear ratio

#define PIN_TRANS_PB1     40  // Oil pressure sensor, displayed in web interface, otherwise unused
#define PIN_TRANS_PB2     43  // Oil pressure sensor, displayed in web interface, otherwise unused
#define PIN_TRANS_PB3     42  // Oil pressure sensor, displayed in web interface, otherwise unused

#define PIN_THROTTLE1     A0  // Throttle 1 input.
#define PIN_THROTTLE2     A1  // Throttle 2 input, not yet used

#define PIN_OIL_PUMP_TEMP A7  // Temperature sensor, not yet used
#define PIN_TRANS_TEMP    A4  // Temperature sensor, not yet used
#define PIN_MG1_TEMP      A5  // Temperature sensor, not yet used
#define PIN_MG2_TEMP      A6  // Temperature sensor, not yet used

// Hard limits
#define MAX_SPEED   6000
#define MAX_CURRENT 27500
#define MAX_REGEN   -5000

// Other definitions
#define UART_BUFFER_SIZE 128

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

// Read PIN_IN1 and PIN_IN2 and decide whether we're in FWD, REV, or neutral
uint8_t get_gear() {
  if (digitalRead(PIN_IN1))
    return (DRIVE);
  if (digitalRead(PIN_IN2))
    return (REVERSE);
  return (NEUTRAL);
}

// Memory for data packets and default contents. MTH data comes from the inverter. HTM messages (setup and normal) are sent to it.
uint8_t mth_data[100];
uint8_t htm_data_setup[80] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 25, 0, 0, 0, 0, 0, 0, 0, 128, 0, 0, 0, 128, 0, 0, 0, 37, 1};
uint8_t htm_data[80] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Only used for diagnostic output
int16_t dc_bus_voltage = 0, temp_inv_water = 0, temp_inv_inductor = 0;

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

// Threshold for detecting discrepancy between throttle sensors
const float THRESHOLD = 0.2;

// Function to check the health of throttle pedal sensors
bool checkThrottleSensors() {
  int Throt1Val = analogRead(PIN_THROTTLE1);
  int Throt2Val = analogRead(PIN_THROTTLE2);

  // Check if the two sensors are within a specified tolerance
  if (abs(Throt2Val - (Throt1Val / 2)) < THRESHOLD) {
    return true;  // Sensors are within tolerance
  } else {
    return false; // Sensors are not within tolerance, potential fault
  }
}

// Based on gear and throttle position, calculate desired torque.
void calculate_torque()
{
  // Check if throttle sensors are healthy
  if (!checkThrottleSensors()) {
    // If sensors are not within tolerance, set torque to zero and exit function
    mg1_torque = 0;
    mg2_torque = 0;
    return; // Exit the function early
  }

  uint8_t gear = get_gear();
  // Force neutral if the main contactor hasn't closed yet
  if (!precharge_complete) gear = NEUTRAL;
  // Normalize throttle 1 input to 0-1000
  int throttle = analogRead(PIN_THROTTLE1);
  throttle = map(throttle, config.pedal_min, config.pedal_max, 0, 1000);
  throttle = constrain(throttle, 0, 1000);

  // If enabled, create an exponential curve that gives more control of low speed / regen.
  if (config.throttle_exp) throttle = throttle * throttle / 1000;

  // Calculate regen amount based on % of mg1 speed and maximum limit
  int32_t regen_speed = mg1_speed;
  int32_t regen = regen_speed * config.regen_factor / 100;
  regen = constrain(regen, -config.regen_limit, config.regen_limit);

  // Map throttle pedal curve so that minimum is full regen and max is full torque
  uint32_t torque;
  if (gear == DRIVE)        torque = map(throttle, 0, 1000, regen,  config.max_torque_fwd);
  else if (gear == REVERSE) torque = map(throttle, 0, 1000, regen, -config.max_torque_rev);
  else                   torque = 0;  // If we're not in FWD or REV default to no torque.

  // Hard cut torque if MG1 is overspeed
  if ((mg1_speed > MAX_SPEED) || (-mg1_speed > MAX_SPEED)) torque = 0;

  // MG1 torque is set to MG2 * 1.25.
  mg2_torque = torque;
  mg1_torque = ((torque * 5) / 4);

  // We don't use mg1 in reverse.
  if (gear == REVERSE)mg1_torque = 0;
}

void setup() {
  // Set up OUTPUT pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_REQ, OUTPUT);
  pinMode(PIN_OIL_PUMP_POWER, OUTPUT);
  pinMode(PIN_INV_POWER, OUTPUT);
  pinMode(PIN_OUT1, OUTPUT);
  pinMode(PIN_TRANS_SL1, OUTPUT);
  pinMode(PIN_TRANS_SL2, OUTPUT);
  pinMode(PIN_TRANS_SP, OUTPUT);

  // Set defalt pin states
  digitalWrite(PIN_REQ, LOW);             // This initial state is unimportant
  digitalWrite(PIN_INV_POWER, HIGH);      // Turn on inverter
  digitalWrite(PIN_OIL_PUMP_POWER, HIGH); // Begin HV precharge
  digitalWrite(PIN_OUT1, LOW);            // Turn off main contactor
  digitalWrite(PIN_TRANS_SL1, LOW);       // Turn off at startup.
  digitalWrite(PIN_TRANS_SL2, LOW);       // Turn off at startup.
  digitalWrite(PIN_TRANS_SP, LOW);        // Turn off at startup, not used yet.

  // Set up INPUT pins
  pinMode(PIN_IN1, INPUT);
  pinMode(PIN_IN2, INPUT);
  pinMode(PIN_BRAKE_IN, INPUT);

  pinMode(PIN_TRANS_PB1, INPUT);
  pinMode(PIN_TRANS_PB2, INPUT);
  pinMode(PIN_TRANS_PB3, INPUT);

  // Oil pump speed
  analogWrite(PIN_OIL_PUMP_PWM, 0);

  Serial1.begin(250000); // Inverter
  Serial2.begin(115200); // Wifi Module

  // I don't know what these magic constants do, but I assume
  // they are necessary for the sync serial port to work.
  PIOA->PIO_ABSR |= 1 << 17;
  PIOA->PIO_PDR |= 1 << 17;
  USART0->US_MR |= 1 << 4 | 1 << 8 | 1 << 18;

  // Black magic from previous gurus
  htm_data[63] = (MAX_REGEN) & 0xFF; // Max regen
  htm_data[64] = ((MAX_REGEN) >> 8);
  htm_data[65] = (MAX_CURRENT) & 0xFF; // Max discharge
  htm_data[66] = ((MAX_CURRENT) >> 8);

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
    digitalWrite(PIN_OUT1, HIGH);
    precharge_complete = 1;
  }
}

// Send a control packet to the inverter to set the torque of MG1
// and MG2 based on desired torque.
void control_inverter() {
  // Black magic from previous gurus. Thanks! I'd love some more info on these data structures.
  int speedSum = mg2_speed + mg1_speed;
  speedSum /= 113;
  htm_data[0] = (byte)speedSum;
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
  for (byte i = 0; i < 78; i++) htm_checksum += htm_data[i];
  htm_data[78] = htm_checksum & 0xFF;
  htm_data[79] = htm_checksum >> 8;

  // Send control frame (init frames until inverter responds, then normal control frames)
  if (inv_initialized) Serial1.write(htm_data, 80);
  else Serial1.write(htm_data_setup, 80);
}

// Every 2ms toggle request line to request inverter status
void poll_inverter() {
  if (inverter_timer.check()) {
    digitalWrite(PIN_REQ, !digitalRead(PIN_REQ));
  }
}

// Wait for 100 bytes of status data, when a frame has been received, parse it.
// Return true when a frame has been received from the inverter.
uint8_t monitor_inverter() {
  if (Serial1.available() >= 100) {
    Serial1.readBytes(mth_data, 100);
    // Discard any unexpected extra bytes to ensure we stay in sync
    while (Serial1.available()) {
      Serial1.read();
      delay(1);
    }
    // Check message checksum
    uint16_t mth_checksum = 0;
    for (int i = 0; i < 98; i++) mth_checksum += mth_data[i];
    uint8_t mth_good = (mth_checksum == (mth_data[98] | (mth_data[99] << 8)));

    // Calculate status information from received data
    if (mth_good) {
      dc_bus_voltage = (((mth_data[82] | mth_data[83] << 8) - 5) / 2);
      temp_inv_water = (mth_data[42] | mth_data[43] << 8);
      temp_inv_inductor = (mth_data[86] | mth_data[87] << 8);
      mg1_speed = mth_data[6] | mth_data[7] << 8;
      mg2_speed = mth_data[31] | mth_data[32] << 8;
      if (mth_data[1])
        inv_initialized = 1; // Invamitialized
      else
        inv_initialized = 0; // In case inverter is reset
    }
    return (1);
  }
  return (0);
}

void choose_ratio() {
  if ( mg1_speed > 50) return;
  if (-mg1_speed > 50) return;
  if (digitalRead(PIN_BRAKE_IN)) {
    trans_sl1 = 1;
    trans_sl2 = 1;
  } else {
    trans_sl1 = 0;
    trans_sl2 = 0;
  }
  // Set gear ratio solenoids
  digitalWrite(PIN_TRANS_SL1, trans_sl1);
  digitalWrite(PIN_TRANS_SL2, trans_sl2);
}

void loop() {
  // If we're not precharged yet, prepare to close contactor.
  if (!precharge_complete) precharge();

  // Poll the motor at regular intervals to request status frames
  poll_inverter();

  // Wait for a status frame from the inverter
  if (monitor_inverter()) {
    // Every time we receive a status frame, calculate torque demand, and send a control frame.
    calculate_torque();
    choose_ratio(); // Note the ratio switching code may override calculated torque
    control_inverter();
  }

  // Set the speed of the electric oil pump
  analogWrite(PIN_OIL_PUMP_PWM, config.oil_pump_pwm);


  // Wait for control data from USB or wifi
  check_serial();
}
