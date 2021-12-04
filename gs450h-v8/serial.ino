
#define SOFTWARE_VERSION 1

// Send JSON for a parameter value
void send_json_param(Stream &port, char* name, char* category, char* unit, uint32_t id, uint32_t value, uint32_t min, uint32_t max, uint32_t dflt) {
  char buf[1024];
  sprintf(buf, "\"%s\":{\"category\": \"%s\", \"unit\":\"%s\", \"isparam\": true, \"i\": %i, \"value\": %i, \"minimum\": %i, \"maximum\": %i, \"default\": %i}",
   name, category, unit, id, value, min, max, dflt);
  port.print(buf);
}

// Send JSON for a non-parameter value
void send_json_value(Stream &port, char* name, char* unit, uint32_t value) {
  char buf[1024];
  sprintf(buf, "\"%s\":{\"unit\":\"%s\", \"isparam\": false, \"value\": %i}",
   name, unit, value);
  port.print(buf);
}

// Set a parameter by name
uint8_t set_config(char* name, uint32_t value) {
  if(!strcmp(name, "precharge_voltage")) {
    config.precharge_voltage = value; return 1;
  } else if(!strcmp(name, "max_torque_fwd")) {
    config.max_torque_fwd = value; return 1;
  } else if(!strcmp(name, "max_torque_rev")) {
    config.max_torque_rev = value; return 1;
  } else if(!strcmp(name, "pedal_min")) {
    config.pedal_min = value; return 1;
  } else if(!strcmp(name, "pedal_max")) {
    config.pedal_max = value; return 1;
  } else if(!strcmp(name, "regen_factor")) {
    config.regen_factor = value; return 1;
  } else if(!strcmp(name, "regen_limit")) {
    config.regen_limit = value; return 1;
  } else if(!strcmp(name, "throttle_exp")) {
    config.throttle_exp = value; return 1;
  } else if(!strcmp(name, "oil_pump_pwm")) {
    config.oil_pump_pwm = value; return 1;
  }
  return 0;
}

// Get a parameter by name
uint8_t get_config(char* name) {
  if(!strcmp(name, "precharge_voltage")) {
    return config.precharge_voltage;
  } else if(!strcmp(name, "max_torque_fwd")) {
    return config.max_torque_fwd;
  } else if(!strcmp(name, "max_torque_rev")) {
    return config.max_torque_rev;
  } else if(!strcmp(name, "pedal_min")) {
    return config.pedal_min;
  } else if(!strcmp(name, "pedal_max")) {
    return config.pedal_max;
  } else if(!strcmp(name, "regen_factor")) {
    return config.regen_factor;
  } else if(!strcmp(name, "regen_limit")) {
    return config.regen_limit;
  } else if(!strcmp(name, "throttle_exp")) {
    return config.throttle_exp;
  } else if(!strcmp(name, "oil_pump_pwm")) {
    return config.oil_pump_pwm;
  } else if(!strcmp(name, "version")) {
    return SOFTWARE_VERSION;
  } else if(!strcmp(name, "dc_bus_voltage")) {
    return dc_bus_voltage;
  } else if(!strcmp(name, "trans_pb1")) {
    return digitalRead(PIN_TRANS_PB1);
  } else if(!strcmp(name, "trans_pb2")) {
    return digitalRead(PIN_TRANS_PB2);
  } else if(!strcmp(name, "trans_pb3")) {
    return digitalRead(PIN_TRANS_PB3);
  } else if(!strcmp(name, "throttle1")) {
    return analogRead(PIN_THROTTLE1);
  } else if(!strcmp(name, "mg1_torque")) {
    return mg1_torque;
  } else if(!strcmp(name, "mg2_torque")) {
    return mg2_torque;
  } else if(!strcmp(name, "mg1_speed")) {
    return mg1_speed;
  } else if(!strcmp(name, "mg2_speed")) {
    return mg2_speed;
  }
  return 0;
}

// Send JSON for all values
void send_json(Stream &port) {
  port.print("{");
  send_json_param(port, "precharge_voltage", "Inverter",    "V", 0, config.precharge_voltage, 0, 2000, 2000);   port.print(",");
  send_json_param(port, "max_torque_fwd",    "Inverter",    "",  1, config.max_torque_fwd,    0, 3500, 0);      port.print(",");
  send_json_param(port, "max_torque_rev",    "Inverter",    "",  2, config.max_torque_rev,    0, 3500, 0);      port.print(",");
  send_json_param(port, "pedal_min",         "Inverter",    "",  3, config.pedal_min,         0, 10000, 2000);  port.print(",");
  send_json_param(port, "pedal_max",         "Inverter",    "",  4, config.pedal_max,         0, 10000, 10000); port.print(",");
  send_json_param(port, "regen_factor",      "Inverter",    "",  5, config.regen_factor,      0, 200, 0);       port.print(",");
  send_json_param(port, "regen_limit",       "Inverter",    "",  6, config.regen_limit,       0, 3500, 0);      port.print(",");
  send_json_param(port, "throttle_exp",      "Inverter",    "",  7, config.throttle_exp,      0, 1, 0);         port.print(",");
  send_json_param(port, "oil_pump_pwm",      "Trasmission", "",  8, config.oil_pump_pwm,      0, 255, 0);       port.print(",");
  send_json_value(port, "version",                          "" ,    SOFTWARE_VERSION);                          port.print(",");
  send_json_value(port, "dc_bus_voltage",                   "V",    dc_bus_voltage);                            port.print(",");
  send_json_value(port, "trans_pb1",                        "",     digitalRead(PIN_TRANS_PB1));                port.print(",");
  send_json_value(port, "trans_pb3",                        "",     digitalRead(PIN_TRANS_PB2));                port.print(",");
  send_json_value(port, "trans_pb3",                        "",     digitalRead(PIN_TRANS_PB3));                port.print(",");
  send_json_value(port, "throttle1",                        "",     analogRead(PIN_THROTTLE1));                 port.print(",");
  send_json_value(port, "mg1_torque",                       "",     mg1_torque);                                port.print(",");
  send_json_value(port, "mg2_torque",                       "",     mg2_torque);                                port.print(",");
  send_json_value(port, "mg1_speed",                        "rpm",  mg1_speed);                                 port.print(",");
  send_json_value(port, "mg2_speed",                        "rpm",  mg2_speed);
  port.println("}");
}

// Buffers for serial communication
char rx_buffer_wifi[UART_BUFFER_SIZE];
uint8_t rx_buffer_offset_wifi;
char rx_buffer_usb[UART_BUFFER_SIZE];
uint8_t rx_buffer_offset_usb;
char last_command[UART_BUFFER_SIZE];

// Process serial data
void process_serial(Stream &port, char* buffer) {
  strncpy(last_command, buffer, UART_BUFFER_SIZE);
  char* cmd = strtok(buffer, " ");
  if(!strcmp(cmd, "json")) {
    send_json(port);
  } else if(!strcmp(cmd, "save")) {
    EEPROM.write(0, config);
    port.println("OK - config saved.");
  } else if(!strcmp(cmd, "load")) {
    EEPROM.read(0, config);
    port.println("OK - config loaded.");
  } else if(!strcmp(cmd, "set")) {
    char* param_name = strtok(NULL, " ");
    char* param_value_s = strtok(NULL, " ");
    uint32_t param_value = atoi(param_value_s);
    if(set_config(param_name, param_value))
      port.println("Paramater set successfully!");
    else
      port.println("Paramater set failed!");
  } else if(!strcmp(cmd, "get")) {
    char* param_name;
    while(param_name = strtok(NULL, ",")) {
      uint32_t param_value = get_config(param_name);
      port.print(param_value);
      port.print(".0");
      port.print("\r\n");
    }
  } else {
    port.println("Unknown command!");
  }
  memset(buffer, 0, UART_BUFFER_SIZE);
}

// Read data from a serial port
void read_serial(Stream &port, char* buffer, uint8_t* offset) {
  uint8_t rx_byte;
  rx_byte = port.read();
  if(rx_byte == '!') {
    port.print("!");
    strncpy(buffer, last_command, UART_BUFFER_SIZE);
    process_serial(port, buffer);
  } else if(rx_byte == 0x0a) {
    // Received a newline, process the buffer
    process_serial(port, buffer);
    (*offset) = 0;
  } else {
    // Received a non-newline, append it to the buffer
    if(*offset < UART_BUFFER_SIZE)
      buffer[(*offset)++] = rx_byte;
  }
}

// Check serial ports for data
void check_serial() {
  while(Serial2.available())
    read_serial(Serial2, rx_buffer_wifi, &rx_buffer_offset_wifi);
  while(SerialUSB.available())
    read_serial(SerialUSB, rx_buffer_usb, &rx_buffer_offset_usb);
}
