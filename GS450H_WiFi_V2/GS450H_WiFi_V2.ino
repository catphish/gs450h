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

struct parameter {
  char name[32];
  char value[15];
  uint8_t type; // 0=NULL, 1=RW, 2=RO
} config;

struct parameter parameters[32];

char json_data[2048];

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
    request->redirect("/config");
  });
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/config.html");
  });
  server.on("/config.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/config.js", "text/javascript");
  });
  server.on("/command", HTTP_POST, [](AsyncWebServerRequest * request) {
    if(request->hasParam("command", true)) {
      AsyncWebParameter* p = request->getParam("command", true);
      Serial.print("\n\t");
      Serial.println(p->value());
    }
    request->send(200, "text/plain", "OK");
  });
  server.on("/data.bin", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "application/octet-stream", (uint8_t*)parameters, sizeof(parameters));
  });
  server.on("/data.json", HTTP_GET, [](AsyncWebServerRequest * request) {
    memset(json_data, 0, sizeof(json_data));
    int idx = 0;
    int json_offset = 0;
    json_data[json_offset++] = '[';
    while(parameters[idx].type) {
      if(idx > 31) break; // Panic, don't overun the array
      if(idx > 0) json_data[json_offset++] = ',';
      json_offset += sprintf(json_data + json_offset, "[%i,\"%s\",%i,\"%s\"]", idx, parameters[idx].name, parameters[idx].type, parameters[idx].value);
      idx++;
    }
    json_data[json_offset++] = ']';
    request->send(200, "application/json", json_data);
  });

  ArduinoOTA.begin();
  server.begin();
}

// Input data will be \t nn , n , DESCRIPTION, nnnn
void process_serial(char* buffer) {
  if(buffer[0] == '\t') {
    int buffer_index=6, name_index=0, value_index=0;
    uint16_t idx = atoi(buffer+1);
    if(idx > 31) return; // Panic, don't overflow the buffer

    memset((char*)(parameters+idx), 0, sizeof(struct parameter));
    parameters[idx].type = atoi(buffer+4);

    while(buffer[buffer_index] != ',') {
      if(name_index > 30) return; // Panic, don't overflow the buffer
      parameters[idx].name[name_index] = buffer[buffer_index];
      name_index++; buffer_index++;
    }
    buffer_index++;
    while(buffer[buffer_index]) {
      if(value_index > 13) return; // Panic, don't overflow the buffer
      parameters[idx].value[value_index] = buffer[buffer_index];
      value_index++; buffer_index++;
    }
  }
  memset(buffer, 0, 64);
}

char rx_buffer[64];
uint8_t rx_buffer_offset;
void read_serial_data() {
  uint8_t rx_byte;
  while(Serial.available()) {
    rx_byte = Serial.read();
    if(rx_byte == 0x0d) {
      // Ignore \r
    } else if(rx_byte == 0x0a) {
      // Received a newline, process the buffer
      process_serial(rx_buffer);
      rx_buffer_offset = 0;
    } else {
      // Received a non-newline, append it to the buffer
      if(rx_buffer_offset < 64)
        rx_buffer[rx_buffer_offset++] = rx_byte;
    }
  }
}

void loop() {
  ArduinoOTA.handle();
  read_serial_data();
}
