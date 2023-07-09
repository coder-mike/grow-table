#include "arduino_secrets.h"
// Timezone - Version: Latest
#include <Timezone.h>

// Time - Version: Latest
#include <TimeLib.h>

// NTPClient - Version: Latest
#include <NTPClient.h>

#include <vector>


#include "thingProperties.h"
#include "io.h"
#include <Arduino_LSM6DS3.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include "interval_timer.h"
#include "averaging_filter.h"

AveragingFilter<1024> temperatureFilter;
int32_t temperatureMilliC = 0; // Filtered
int64_t temperatureDerivative = 0; // 32.32 fixed point milli-degrees per sample period
float temperature = 0.0; // Filtered
float temperatureDerivative = 0.0; // Degrees per minute

bool builtinLedCtrl = false;
bool criticalHighTemperature = false;

// For the time server
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
TimeChangeRule aEST = {"AEST", First, Sun, Apr, 3, 600};    // Standard time = UTC + 10 hours
TimeChangeRule aEDT = {"AEDT", First, Sun, Oct, 2, 660};    // Daylight time = UTC + 11 hours
Timezone melbourneTz(aEST, aEDT);

// Timing
IntervalTimer tmrTimeSync(1000 * 3600);
IntervalTimer tmrOneSecond(1000);
IntervalTimer tmrBlinkLed(500);
IntervalTimer tmrRemoteControlExpire(1000 * 60 * 5); // 5 min
IntervalTimer tmrTemperatureSample(10); // 10ms

int currentHour = 0; // 0 to 23
const int lightOnHour = 7; // 7 AM
const int lightOffHour = 19; // 7 PM
const float targetTemperature = 25;
const float absoluteMaximumTemperature = 35.0;

void remoteControl();
void controlLoop();
void refreshCurrentHour();
String formatLocalTime();

void setup() {
  initIo();
  buzzer_beepLong();

  // Yellow at startup
  rgb_yellow();

  // Initialize serial and wait for port to open:
  Serial.begin(9600);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500);

  // Defined in thingProperties.h
  initProperties();

  // Set color to green while connecting
  rgb_green();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);


  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  lastMillis = millis();

  // Wait for cloud connection before starting NTP UDP
  while (ArduinoCloud.connected() != 1) {
    ArduinoCloud.update();
    delay(100);
  }

  // Now that we are connected to the cloud, we can safely start the NTP Client
  timeClient.begin();

  // Set color to blue once connected to the Arduino server
  rgb_blue();

  // Beep at and of startup
  buzzer_beepShort();

  loopCounter = 0;
}

void loop() {
  if (reboot) while(1);

  ArduinoCloud.update();

  loopCounter++;

  if (tmrTimeSync.check()) timeClient.update();

  // Flash builtin LED
  if (tmrBlinkLed.check()) {
    builtinLedCtrl = !builtinLedCtrl;
    builtinLed_set(builtinLedCtrl);
  }

  if (tmrTemperatureSample.check()) {
    temperatureRaw = tempSense1_read();
    temperatureMilliC = temperatureFilter.filter(temperatureRaw);
  }

  if (tmrOneSecond.check()) {
    temperature = (float)temperatureMilliC * 0.001f;
    cloud_temperature = temperature;

    // Temperature derivative
    temperatureDerivativeMilliC = temperatureFilter.slope();

    // Convert to degrees per minute.
    // Ratio 60000/tmrTemperatureSample.intervalMs() to convert from sample period to minutes.
    // Ratio 32768 to convert from 32.32 fixed point to float.
    temperatureDerivative = (float)temperatureDerivativeMilliC / tmrTemperatureSample.intervalMs() * 60000 / 32768;

    cloud_temperatureDerivative = temperatureDerivative;
    cloud_fanSpeedRpmOut = fan_readSpeed();
    cloud_time = formatLocalTime();

    refreshCurrentHour();
    controlLoop();
  }

  if (cloud_remoteCtrl) {
    // Remote control
    lampCtrl = cloud_lampCtrl;
    heatCtrl = cloud_heatCtrl;
    fanOnOff = cloud_fanOnOff;
    fanSpeedCtrl = cloud_fanSpeedCtrl;
    if (cloud_buzzer) { buzzer_beepLong(); cloud_buzzer = false; }
    nextRemoteControlExpire -= deltaMs;
    if (nextRemoteControlExpire < 0) {
      cloud_remoteCtrl = false;
      nextRemoteControlExpire = 0;
    }
  }

  // output
  relayL1_set(lampCtrl);
  relayL2_set(heatCtrl);
  fanOnOff_set(fanOnOff);
  fan_setSpeed(fanSpeedCtrl);
}

// Called every second
void controlLoop() {
  // Lamp control
  if ((currentHour >= lightOnHour) && (currentHour < lightOffHour)) {
    lampCtrl = true; // Lamp on during daylight
  } else {
    lampCtrl = false; // Off at night
  }

  /* PID controller */

  // How much we are over the threshold temperature, in degrees, or negative if below temp.
  float amountOverTemp = (temperature - targetTemperature);

  // Proportional contribution to PI fan control. As the temperature goes up,
  // this has an immediate effect on the fan speed.
  fanSpeedProportional = amountOverTemp * (1.0/3.0);

  // Integral contribution to PI fan control. The longer the temperature stays
  // above the target, the higher the fan speed will get to try bring it back.
  // The further out it is, the faster the fan speed will change.
  fanSpeedIntegrator += amountOverTemp * 0.001;
  fanSpeedIntegrator = constrain(fanSpeedIntegrator, -0.1, 1.0);

  fanSpeedD = 0;//temperatureDerivative * 0.3;

  fanSpeedCtrl = fanSpeedIntegrator + fanSpeedProportional + fanSpeedD;
  fanSpeedCtrl = constrain(fanSpeedCtrl, -0.1, 1.0);

  // Treating the fan on/off and the heat pad as just extensions
  // of the fan speed in the negative direction. Basically, fanSpeedCtrl
  // is the amount by which the control circuit is trying to cool the
  // system

  if (fanSpeedCtrl > -0.04) {
    fanOnOff = true; // Fan on
  } else if (fanSpeedCtrl < -0.05) {
    fanOnOff = false; // Fan off
  }
  if (fanSpeedCtrl <= -0.09) {
    heatCtrl = true; // Heat pad on
  } else if (fanSpeedCtrl >= -0.05) {
    heatCtrl = false; // Heat pad off
  }

  /* Emergency overrides if we exceed the absolute maximum */
  if (temperature >= absoluteMaximumTemperature) {
    criticalHighTemperature = true;
  } else if (temperature < absoluteMaximumTemperature - 1) {
    criticalHighTemperature = false;
  }

  if (criticalHighTemperature) {
    lampCtrl = false; // Lamp off
    fanOnOff = true; // Fan on
    heatCtrl = false; // Heat pad off
    fanSpeedCtrl = 1.0; // Full fan speed
  }
}

void refreshCurrentHour() {
  // Get the current epoch time in UTC
  unsigned long epochTime = timeClient.getEpochTime();

  // Convert the UTC time to local time in Melbourne, taking DST into account
  time_t localTime = melbourneTz.toLocal(epochTime);

  // Get the current local hour (0-23)
  currentHour = hour(localTime);
}

String formatLocalTime() {
  time_t localTime = melbourneTz.toLocal(timeClient.getEpochTime());

  char timeStr[9];
  sprintf(timeStr, "%02d:%02d:%02d", hour(localTime), minute(localTime), second(localTime));
  return String(timeStr);
}

void onCloudBuzzerChange()  {
}
void onCloudFanSpeedCtrlChange()  {
}
void onCloudRemoteCtrlChange()  {
  nextRemoteControlExpire = remoteControlExpireMs;
}
void onCloudLampCtrlChange()  {
}
void onCloudHeatCtrlChange()  {
}
void onCloudFanOnOffChange()  {
}
void onRebootChange()  {
  if (reboot) {
    while(1);
  }
}