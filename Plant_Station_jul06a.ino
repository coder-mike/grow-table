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

AveragingFilter<1024> temperatureFilter; // Note: don't make this too big or the device runs out of dynamic memory and reboots constantly.
AveragingFilter<32> temperatureDerivativeFilter;
int32_t temperatureMilliC = 0; // Filtered
float temperatureDerivativeMilliC = 0;
float temperature = 0.0; // Filtered
int32_t lastTemperatureMilliC = 0; // For derivative
float temperatureDerivative = 0.0; // Filtered (Degrees per second)

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
IntervalTimer tmrArudinoCloudUpdate(1000);
IntervalTimer tmrLoopMaxSpeed(1);

int currentHour = 0; // 0 to 23
const int lightOnHour = 7; // 7 AM
const int lightOffHour = 19; // 7 PM
const float targetTemperature = 25;
const float absoluteMaximumTemperature = 35.0;

void remoteControl();
void controlLoop();
void refreshCurrentHour();
String formatLocalTime();
void doReboot();

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

  // Wait for cloud connection before starting NTP UDP
  while (ArduinoCloud.connected() != 1) {
    ArduinoCloud.update();
    delay(100);
  }

  // Now that we are connected to the cloud, we can safely start the NTP Client
  Serial.println("Connecting to time server...");
  timeClient.begin();
  Serial.print(String("Time is ") + formatLocalTime());

  // Set color to blue once connected to the Arduino server
  rgb_blue();

  // Beep at and of startup
  buzzer_beepShort();

  loopCounter = 0;

  Serial.println("Finished setup");
}

void loop() {

  if (reboot) doReboot();

  if (!tmrLoopMaxSpeed.check()) return;
  loopCounter++;

  if (tmrArudinoCloudUpdate.check()) {
    // Note: this seems to be a blocking call and can be really slow. So doing this once per second
    // instead of every loop cycle seems to be a compromise that at least helps the temperature
    // reading to continue in the background.
    unsigned long start = millis();
    ArduinoCloud.update();
    unsigned long end = millis();
    arduinoCloudUpdateDelay = end - start;
  }

  if (ArduinoCloud.connected()) {
    rgb_blue();
  } else {
    rgb_green();
  }

  if (tmrTimeSync.check()) {
    timeClient.update();
  }

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

    // Temperature derivative (calculated once per second to reduce noise and make the calculations easier, and averaged over 32 seconds)
    if (lastTemperatureMilliC == 0) lastTemperatureMilliC = temperatureMilliC;
    temperatureDerivative = temperatureDerivativeFilter.filter(temperatureMilliC - lastTemperatureMilliC) * 0.001f; // Conversion from millidegrees to degrees.
    lastTemperatureMilliC = temperatureMilliC;

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
    if (tmrRemoteControlExpire.check()) {
      cloud_remoteCtrl = false;
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
  fanSpeedIntegrator += amountOverTemp * 0.0005;
  fanSpeedIntegrator = constrain(fanSpeedIntegrator, -1.0, 1.0);

  // Proportional to the derivative. 0.1 increment to the control signal for
  // every 1 degree per minute. Or to put another way, if we expect the temperature
  // to increase by 5 degrees over the next minute (quite fast) then this will add
  // 50% to the fan speed. This may be a little bit conservative, but making this
  // factor too high might make the system unstable.
  // fanSpeedD = (temperatureDerivative * 60) * 0.1;
  fanSpeedD = 0;

  fanSpeedCtrl = fanSpeedIntegrator + fanSpeedProportional + fanSpeedD;
  fanSpeedCtrl = constrain(fanSpeedCtrl, -1.0, 1.0);

  Serial.print(temperature, 3);
  Serial.print(",");
  Serial.print(temperatureDerivative, 3);
  Serial.print(",");
  Serial.print(fanSpeedProportional, 3);
  Serial.print(",");
  Serial.print(fanSpeedIntegrator, 3);
  Serial.print(",");
  Serial.print(fanSpeedD, 3);
  Serial.print(",");
  Serial.println(fanSpeedCtrl, 3);

  // Treating the fan on/off and the heat pad as just extensions
  // of the fan speed in the negative direction. Basically, fanSpeedCtrl
  // is the amount by which the control circuit is trying to cool the
  // system

  if (fanSpeedCtrl > -0.4) {
    fanOnOff = true; // Fan on
  } else if (fanSpeedCtrl < -0.5) {
    fanOnOff = false; // Fan off
  }
  if (fanSpeedCtrl <= -0.9) {
    heatCtrl = true; // Heat pad on
  } else if (fanSpeedCtrl >= -0.5) {
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
  tmrRemoteControlExpire.reset();
}
void onCloudLampCtrlChange()  {
}
void onCloudHeatCtrlChange()  {
}
void onCloudFanOnOffChange()  {
}
void onRebootChange()  {
  if (reboot) {
    doReboot();
  }
}

void doReboot() {
  Serial.println("Rebooting...");
  while(1);
}