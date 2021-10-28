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
  int16_t voltage;
  int16_t water_temp;
  int16_t mg1_speed;
  int16_t mg2_speed;
  uint16_t mg1_temp;
  uint16_t mg2_temp;
  uint16_t pump_temp;
  uint16_t trans_temp;
  uint16_t trans_sl;
  uint16_t trans_pb1;
  uint16_t trans_pb2;
  uint16_t trans_pb3;
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
  uint16_t oil_pump_pwm;
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
  AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/highcharts.js.gz", "text/javascript");
  response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });
  server.on("/highcharts-more.js", HTTP_GET, [](AsyncWebServerRequest * request) {
  AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/highcharts-more.js.gz", "text/javascript");
  response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });
  server.on("/solid-gauge.js", HTTP_GET, [](AsyncWebServerRequest * request) {
  AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/solid-gauge.js.gz", "text/javascript");
  response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });
  server.on("/data.json", HTTP_GET, [](AsyncWebServerRequest * request) {
    sprintf(json_data, "{\"pedal_min\":%i,"
                        "\"pedal_max\":%i,"
                        "\"max_torque_fwd\":%i,"
                        "\"max_torque_rev\":%i,"
                        "\"regen_factor\":%i,"
                        "\"regen_limit\":%i,"
                        "\"throttle_exp\":%i,"
                        "\"precharge_voltage\":%i,"
                        "\"oil_pump_pwm\":%i,"
                        "\"voltage\":%i,"
                        "\"water_temp\":%i,"
                        "\"mg1_speed\":%i,"
                        "\"mg2_speed\":%i,"
                        "\"mg1_temp\":%i,"
                        "\"mg2_temp\":%i,"
                        "\"pump_temp\":%i,"
                        "\"trans_temp\":%i,"
                        "\"trans_sl\":%i,"
                        "\"trans_pb1\":%i,"
                        "\"trans_pb2\":%i,"
                        "\"trans_pb3\":%i}",
                        config.pedal_min, config.pedal_max,
                        config.max_torque_fwd, config.max_torque_rev,
                        config.regen_factor, config.regen_limit,
                        config.throttle_exp, config.precharge_voltage,
                        config.oil_pump_pwm,
                        status.voltage, status.water_temp,
                        status.mg1_speed, status.mg2_speed,
                        status.mg1_temp, status.mg2_temp,
                        status.pump_temp, status.trans_temp,
                        status.trans_sl, status.trans_pb1,
                        status.trans_pb2, status.trans_pb3);
    request->send(200, "application/json", json_data);
  });

  ArduinoOTA.begin();
  server.begin();
}


void process_serial(char* buffer) {
  switch(buffer[0]) {
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
  case 'a':
    config.oil_pump_pwm = atoi(buffer+1);
    break;
  case 's':
    status.trans_sl = atoi(buffer+1);
    break;
  case 'b':
    status.voltage = atoi(buffer+1);
    break;
  case 'm':
    status.mg1_speed = atoi(buffer+1);
    break;
  case 'n':
    status.mg2_speed = atoi(buffer+1);
    break;
  case 'c':
    status.water_temp = atoi(buffer+1);
    break;
  case 'h':
    status.mg1_temp = atoi(buffer+1);
    break;
  case 'j':
    status.mg2_temp = atoi(buffer+1);
    break;
  case 'k':
    status.pump_temp = atoi(buffer+1);
    break;
  case 'l':
    status.trans_temp = atoi(buffer+1);
    break;
  case 'd':
    status.trans_pb1 = atoi(buffer+1);
    break;
  case 'f':
    status.trans_pb2 = atoi(buffer+1);
    break;
  case 'g':
    status.trans_pb3 = atoi(buffer+1);
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
