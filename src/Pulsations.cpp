#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "RTClib.h"
#include "SLIPEncodedSerial.h"

#ifndef FRAME_DURATION_MS
#define FRAME_DURATION_MS=40
#endif

SLIPEncodedSerial SLIPSerial(Serial);

RTC_DS3231 rtc;
Adafruit_BNO055 sensor;

sensors_event_t event;
imu::Vector<3> accel;
imu::Vector<3> euler;
imu::Quaternion quat;

DateTime now;
long ms_start;
long sec_start;
long frame_start_ms;
long frame_end_ms;

uint8_t STATUS_PIN = 13;
uint8_t ACTIVITY_PIN = 13;

void setup() {
  digitalWrite(STATUS_PIN, HIGH);
  while (!Serial);

  SLIPSerial.begin(115200);
  digitalWrite(STATUS_PIN, LOW);

  if (!rtc.begin()) {
    digitalWrite(STATUS_PIN, HIGH);
    delay(500);
    digitalWrite(STATUS_PIN, LOW);
    delay(500);
  } else {
    if (rtc.lostPower()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  }
  now = rtc.now();
  ms_start = millis();
  sec_start = now.unixtime();

  sensor = Adafruit_BNO055(0x28);
  while (!sensor.begin()) {
    digitalWrite(STATUS_PIN, HIGH);
    delay(50);
    digitalWrite(STATUS_PIN, LOW);
    delay(50);
  }

  delay(500);
  sensor.setExtCrystalUse(true);

  char statbuffer[6];
  statbuffer[0] = 3;
  #ifdef DEVICE_ID
    statbuffer[1] = DEVICE_ID;
  #else
    statbuffer[1] = 0;
  #endif
  statbuffer[2] = 6;
  sensor.getSystemStatus(&statbuffer[3], &statbuffer[4], &statbuffer[5]);
  SLIPSerial.beginPacket();
  SLIPSerial.write(statbuffer, sizeof(statbuffer));
  SLIPSerial.endPacket();
}

void loop() {
  digitalWrite(ACTIVITY_PIN, HIGH);

  //sensor.getEvent(&event);
  now = rtc.now();
  frame_start_ms = millis();

#ifdef SEND_CALIBRATION
  if (now.second() % 10) {
    char calbuffer[7];
    calbuffer[0] = 2;
    #ifdef DEVICE_ID
      calbuffer[1] = DEVICE_ID;
    #else
      calbuffer[1] = 0;
    #endif
    calbuffer[2] = 7;
    sensor.getCalibration(&calbuffer[3], &calbuffer[4], &calbuffer[5], &calbuffer[6]);
    SLIPSerial.beginPacket();
    SLIPSerial.write(calbuffer, sizeof(calbuffer));
    SLIPSerial.endPacket();
  }
#endif

  char msgbuffer[64];
  msgbuffer[0] = 1;
  #ifdef DEVICE_ID
    msgbuffer[1] = DEVICE_ID;
  #else
    msgbuffer[1] = 0;
  #endif

  uint8_t idx = 3;
  float *fl;

#ifdef SEND_QUATERNION
  quat = sensor.getQuat();
  fl = (float *)&msgbuffer[idx];
  *fl = (float)quat.w();
  idx += sizeof(float);
  fl = (float *)&msgbuffer[idx];
  *fl = (float)quat.x();
  idx += sizeof(float);
  fl = (float *)&msgbuffer[idx];
  *fl = (float)quat.y();
  idx += sizeof(float);
  fl = (float *)&msgbuffer[idx];
  *fl = (float)quat.z();
  idx += sizeof(float);

  fl = (float *)&msgbuffer[idx];
  *fl = (float)quat.magnitude();
  idx += sizeof(float);
#endif

#ifdef SEND_EULER
  euler = sensor.getVector(Adafruit_BNO055::VECTOR_EULER);
  fl = (float *)&msgbuffer[idx];
  *fl = (float)euler.x();
  idx += sizeof(float);
  fl = (float *)&msgbuffer[idx];
  *fl = (float)euler.y();
  idx += sizeof(float);
  fl = (float *)&msgbuffer[idx];
  *fl = (float)euler.z();
  idx += sizeof(float);
#endif

#ifdef SEND_LINEAR_ACCELERATION
  accel = sensor.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  fl = (float *)&msgbuffer[idx];
  *fl = (float)accel.x();
  idx += sizeof(float);
  fl = (float *)&msgbuffer[idx];
  *fl = (float)accel.y();
  idx += sizeof(float);
  fl = (float *)&msgbuffer[idx];
  *fl = (float)accel.z();
  idx += sizeof(float);
#endif

  msgbuffer[2] = idx;

  SLIPSerial.beginPacket();
  SLIPSerial.write(msgbuffer, idx);
  SLIPSerial.endPacket();

  digitalWrite(ACTIVITY_PIN, LOW);

  now = rtc.now();
  frame_end_ms = millis();
  if (frame_end_ms - frame_start_ms < FRAME_DURATION_MS) {
    delay(frame_end_ms - frame_start_ms);
  }
}
