#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "RTClib.h"
#include "SLIPEncodedSerial.h"
#include "OSCMessage.h"
#include "OSCTiming.h"

SLIPEncodedSerial SLIPSerial(Serial);

RTC_DS3231 rtc;

Adafruit_BNO055 sensor;
sensors_event_t event;
imu::Vector<3> accel;
imu::Vector<3> euler;

OSCMessage msg;
char addr[32];

DateTime now;
osctime_t osctime;
long ms_start;
long sec_start;
long frame_start_ms;
long frame_end_ms;

uint8_t STATUS_PIN = 13;
uint8_t ACTIVITY_PIN = 13;

void setup() {
  digitalWrite(STATUS_PIN, HIGH);
  while (!Serial);

#ifdef DEVICE_ID
    sprintf(addr, "/u/%u", DEVICE_ID);
#else
  sprintf(addr, "/u/%u", 0);
#endif

  SLIPSerial.begin(57600);
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
}

void loop() {
  digitalWrite(ACTIVITY_PIN, HIGH);

  now = rtc.now();
  frame_start_ms = millis();

  if (now.second() % 10) {
    char calbuffer[7];
    calbuffer[0] = 2;
    calbuffer[1] = 1;
    calbuffer[2] = 7;
    sensor.getCalibration(&calbuffer[3], &calbuffer[4], &calbuffer[5], &calbuffer[6]);
    SLIPSerial.beginPacket();
    SLIPSerial.write(calbuffer, sizeof(calbuffer));
    SLIPSerial.endPacket();
  }

  euler = sensor.getVector(Adafruit_BNO055::VECTOR_EULER);
  accel = sensor.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  char msgbuffer[64];
  msgbuffer[0] = 1;
  msgbuffer[1] = 1;

  uint8_t idx = 3;
  float *fl;

  fl = (float *)&msgbuffer[idx];
  *fl = (float)euler.x();
  idx += sizeof(float);
  fl = (float *)&msgbuffer[idx];
  *fl = (float)euler.y();
  idx += sizeof(float);
  fl = (float *)&msgbuffer[idx];
  *fl = (float)euler.z();
  idx += sizeof(float);

  fl = (float *)&msgbuffer[idx];
  *fl = (float)accel.x();
  idx += sizeof(float);
  fl = (float *)&msgbuffer[idx];
  *fl = (float)accel.y();
  idx += sizeof(float);
  fl = (float *)&msgbuffer[idx];
  *fl = (float)accel.z();
  idx += sizeof(float);

  msgbuffer[2] = idx;

  SLIPSerial.beginPacket();
  SLIPSerial.write(msgbuffer, idx);
  SLIPSerial.endPacket();

  digitalWrite(ACTIVITY_PIN, LOW);

  now = rtc.now();
  frame_end_ms = millis();
  if (frame_end_ms - frame_start_ms < 40) {
    delay(frame_end_ms - frame_start_ms);
  }
}
