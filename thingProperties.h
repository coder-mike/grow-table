// Code generated by Arduino IoT Cloud, DO NOT EDIT.

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>

const char SSID[]     = SECRET_SSID;    // Network SSID (name)
const char PASS[]     = SECRET_OPTIONAL_PASS;    // Network password (use for WPA, or use as key for WEP)

void onCloudTemperatureHighChange();
void onCloudTemperatureLowChange();
void onCloudLampCtrlChange();
void onCloudRemoteCtrlChange();
void onRebootChange();

String cloud_time;
float cloud_fanSpeedRpmOut;
float cloud_temperature;
float cloud_temperatureHigh;
float cloud_temperatureLow;
float fanSpeedIntegrator;
float fanSpeedProportional;
bool cloud_lampCtrl;
bool cloud_remoteCtrl;
bool reboot;

void initProperties(){

  ArduinoCloud.addProperty(cloud_time, READ, 1 * SECONDS, NULL);
  ArduinoCloud.addProperty(cloud_fanSpeedRpmOut, READ, 1 * SECONDS, NULL);
  ArduinoCloud.addProperty(cloud_temperature, READ, 1 * SECONDS, NULL);
  ArduinoCloud.addProperty(cloud_temperatureHigh, READWRITE, ON_CHANGE, onCloudTemperatureHighChange);
  ArduinoCloud.addProperty(cloud_temperatureLow, READWRITE, ON_CHANGE, onCloudTemperatureLowChange);
  ArduinoCloud.addProperty(fanSpeedIntegrator, READ, 1 * SECONDS, NULL);
  ArduinoCloud.addProperty(fanSpeedProportional, READ, 1 * SECONDS, NULL);
  ArduinoCloud.addProperty(cloud_lampCtrl, READWRITE, ON_CHANGE, onCloudLampCtrlChange);
  ArduinoCloud.addProperty(cloud_remoteCtrl, READWRITE, 1 * SECONDS, onCloudRemoteCtrlChange);
  ArduinoCloud.addProperty(reboot, READWRITE, ON_CHANGE, onRebootChange);

}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
