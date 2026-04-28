/*
 * Copyright (C) 2026 [SoundHorizons]
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <Wire.h>
#include <MPU6050.h>

const char* ssid     = "YourWiFiRouter";
const char* password = "YourPassword";

MPU6050 sensor;

int16_t  Axis[6];
float    filter[6];
float    Data[6];
const float alpha = 0.20f;

const float ACC_MIN  = -16383.0f; //scaled for +- 1g, suitable for body accelerations
const float ACC_MAX  =  16385.0f;
const float GYRO_MIN = -32768.0f;
const float GYRO_MAX =  32768.0f;

WiFiUDP Udp;
const unsigned int port   = 8000;
const char*        outIp  = "192.168.0.255";

unsigned long lastUpdateTime  = 0;
const unsigned long updateInterval = 10;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
  Udp.begin(port);

  Wire.begin();
  sensor.initialize();
}

//int32_t indice = 0;

void loop() {
  unsigned long now = millis();
  if (now - lastUpdateTime < updateInterval) return;
  lastUpdateTime = now;

  // read raw sensor data
  sensor.getMotion6(&Axis[0], &Axis[1], &Axis[2], &Axis[3], &Axis[4], &Axis[5]);

  // filter & normalize accel
  for (int i = 0; i < 3; i++) {
    filter[i] = alpha * Axis[i] + (1.0f - alpha) * filter[i];
    filter[i] = constrain(filter[i], ACC_MIN, ACC_MAX);
    Data[i]   = (filter[i] - ACC_MIN) / (ACC_MAX - ACC_MIN);
  }

  // filter & normalize gyro
  for (int i = 0; i < 3; i++) {
    filter[i+3] = alpha * Axis[i+3] + (1.0f - alpha) * filter[i+3];
    filter[i+3] = constrain(filter[i+3], GYRO_MIN, GYRO_MAX);
    Data[i+3]   = (filter[i+3] - GYRO_MIN) / (GYRO_MAX - GYRO_MIN);
  }

  // build & send OSC
  OSCMessage msg("/S1");
  //msg.add(indice++); // Debug value to check data received order
  for (int i = 0; i < 6; i++) {
    msg.add(Data[i]);     // float in [0..1]
  }
  //indice %= 16000;

  Udp.beginPacket(outIp, port);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}
