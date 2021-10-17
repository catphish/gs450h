// Import required libraries
#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

struct {
  uint16_t voltage;
  uint16_t water_temp;
  int16_t mg1_speed;
  int16_t mg2_speed;
} status;

struct {
  uint16_t precharge_voltage;
  uint16_t max_torque_fwd;
  uint16_t max_torque_rev;
  uint16_t pedal_min;
  uint16_t pedal_max;
  uint16_t regen_factor;
  uint16_t regen_limit;
  uint16_t throttle_exp;
} config;

char json_data[512];

void setup() {
  Serial.begin(115200);

  // Initialize SPIFFS
  if (!SPIFFS.begin()) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  //Start WiFi AP mode
  WiFi.mode(WIFI_AP);
  WiFi.softAP("GS450H");

  // Route for root / web pages
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html");
  });
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest * request) {
    Serial.println("ENABLE");
    request->send(SPIFFS, "/config.html");
  });
  server.on("/command", HTTP_POST, [](AsyncWebServerRequest * request) {
    if(request->hasParam("command", true)) {
      AsyncWebParameter* p = request->getParam("command", true);
      Serial.println(p->value());
    }
    request->send(200, "text/plain", "OK");
  });
  server.on("/main.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/main.js", "text/javascript");
  });
  server.on("/config.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/config.js", "text/javascript");
  });
  server.on("/highcharts.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/highcharts.js", "text/javascript");
  });
  server.on("/highcharts-more.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/highcharts-more.js", "text/javascript");
  });
  server.on("/solid-gauge.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/solid-gauge.js", "text/javascript");
  });
  server.on("/data.json", HTTP_GET, [](AsyncWebServerRequest * request) {
    sprintf(json_data, "{\"voltage\":%i, \"mg1_speed\":%i, \"mg2_speed\":%i, \"water_temp\":%i}", status.voltage, status.mg1_speed, status.mg2_speed, status.water_temp);
    request->send(200, "application/json", json_data);
  });
  server.on("/config.json", HTTP_GET, [](AsyncWebServerRequest * request) {
    sprintf(json_data, "{\"pedal_min\":%i, \"pedal_max\":%i, \"max_torque_fwd\":%i, \"max_torque_rev\":%i, \"regen_factor\":%i, \"regen_limit\":%i, \"throttle_exp\":%i, \"precharge_voltage\":%i}", config.pedal_min, config.pedal_max, config.max_torque_fwd, config.max_torque_rev, config.regen_factor, config.regen_limit, config.throttle_exp, config.precharge_voltage);
    request->send(200, "application/json", json_data);
  });

  ArduinoOTA.begin();
  server.begin();
}


void process_serial(char* buffer) {
  switch(buffer[0]) {
  case 'b':
    status.voltage = atoi(buffer+1);
    break;
  case 'm':
    status.mg1_speed = atoi(buffer+1);
    break;
  case 'n':
    status.mg2_speed = atoi(buffer+1);
    break;
  case 'g':
    status.water_temp = atoi(buffer+1);
    break;
  case 'e':
    config.pedal_min = atoi(buffer+1);
    break;
  case 'r':
    config.pedal_max = atoi(buffer+1);
    break;
  case 't':
    config.max_torque_fwd = atoi(buffer+1);
    break;
  case 'y':
    config.max_torque_rev = atoi(buffer+1);
    break;
  case 'u':
    config.regen_factor = atoi(buffer+1);
    break;
  case 'i':
    config.regen_limit = atoi(buffer+1);
    break;
  case 'o':
    config.throttle_exp = atoi(buffer+1);
    break;
  case 'v':
    config.precharge_voltage = atoi(buffer+1);
    break;
  }
  memset(buffer, 0, 16);
}

char rx_buffer[16];
uint8_t rx_buffer_offset;
void read_serial_data() {
  uint8_t rx_byte;
  while(Serial.available()) {
    rx_byte = Serial.read();
    if(rx_byte == 0x0a) {
      // Received a newline, process the buffer
      process_serial(rx_buffer);
      rx_buffer_offset = 0;
    } else {
      // Received a non-newline, append it to the buffer
      if(rx_buffer_offset < 16)
        rx_buffer[rx_buffer_offset++] = rx_byte;
    }
  }
}

void loop() {
  ArduinoOTA.handle();
  read_serial_data();
}
