/**
 * ESP8266 12F and HTU21 (GY-21) Temperature and Humidity Sersor with Vera integration.
 *
 * Partly based on
 *  https://github.com/Ispep/Hemautomation/blob/master/ESP8266/ESP8266%2012E%20TemperaturSensor/ESP8266-12E-TemperaturSensor.ino
 *  https://github.com/G6EJD/BME280-Sleep-and-Address-change
 *  https://github.com/devxplained/HTU21D-Sensor-Library/tree/main/examples
 *
 * To be powerd by 3xAA or 3xAAA NiMH Cells
 * 
 * Create vera devices :
 * Temperature sensor :
 *   curl "http://VERAIP:3480/data_request?id=action&serviceId=urn:micasaverde-com:serviceId:HomeAutomationGateway1&action=CreateDevice&deviceType=urn:schemas-micasaverde-com:device:TemperatureSensor:1&internalID=&Description=WiFiTemp1&UpnpDevFilename=D_TemperatureSensor1.xml&UpnpImplFilename=&MacAddress=&RoomNum=0&Reload=1&IpAddress="
 * Humidity sensor :
 *   curl "http://VERAIP:3480/data_request?id=action&serviceId=urn:micasaverde-com:serviceId:HomeAutomationGateway1&action=CreateDevice&deviceType=urn:schemas-micasaverde-com:device:HumiditySensor:1&internalID=&Description=WiFiHum1&UpnpDevFilename=D_HumiditySensor1.xml&UpnpImplFilename=&MacAddress=&RoomNum=0&Reload=1&IpAddress="
 *
 **/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <coredecls.h> //crc32
#include <HTU21D.h>

// initialize HTU21 https://github.com/devxplained/HTU21D-Sensor-Library/tree/main library
HTU21D htu21;

// ESP8266 PIN 4 SDA
// ESP8266 PIN 5 SCL

//Atomic OTA Updates
#define ATOMIC_FS_UPDATE

// The ESP8266 RTC memory is arranged into blocks of 4 bytes. The access methods read and write 4 bytes at a time,
// so the RTC data structure should be padded to a 4-byte multiple.
struct {
  uint32_t crc32=0;   // 4 bytes
  uint8_t channel=0;  // 1 byte, 5 in total
  uint8_t bssid[6]={0}; // 6 bytes, 11 in total
  float old_temp=0;   // 4 bytes, 15
  uint8_t padding=0;  // 1 byte, 16 in total
} rtcData;

//Read battery voltage
ADC_MODE(ADC_VCC);

#include "config.h"

String ProgramVersion  = "0.1";

float glb_temp=0,glb_hum=0,glb_pres=0 ;
int glb_batterylevel=0;
long glb_rssi=0;

WiFiClient client;
HTTPClient http;

void gotoDeepSleep(int SleepDelay){
  Serial.print("Sleeping " + String(SleepDelay) + " seconds\n");
  ESP.deepSleepInstant(SleepDelay * 1000000,WAKE_RF_DISABLED);
}

//Update temperature reading into RTC memory
void updateOldTemp () {
   rtcData.old_temp = glb_temp;
   rtcData.crc32 = crc32(((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4,0xffffffff);
   ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtcData, sizeof( rtcData ));
}

void update_started() {
  Serial.println("HTTP update process started");
}

void update_finished() {
  Serial.println("HTTP update process finished");
}

void update_progress(int cur, int total) {
  Serial.printf("HTTP update process at %d of %d bytes...\n", cur, total);
}

void update_error(int err) {
  Serial.printf("HTTP update fatal error code %d\n", err);
}

int checkForUpdates() {
    ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);

    // Add optional callback notifiers
    ESPhttpUpdate.onStart(update_started);
    ESPhttpUpdate.onEnd(update_finished);
    ESPhttpUpdate.onProgress(update_progress);
    ESPhttpUpdate.onError(update_error);

    if (glb_batterylevel < 50) {
      Serial.print("\nBattery Level too low for OTA update\n");
      return 1;
    }

    Serial.print("\nChecking for update from " + String(UpdateURL) + ". Current FW version " + String(FWVersion) + "\n");
    t_httpUpdate_return ret = ESPhttpUpdate.update(client, String(UpdateURL),String(FWVersion));

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK");
        delay(5000);
        ESP.restart();
        break;
    }

  return 0;
}

int connect_wifi (){
  int retries = 0;
  int wifiStatus = WiFi.status();

  WiFi.forceSleepWake();
  delay(1);
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, subnet);

  // Try to read WiFi settings from RTC memory
  bool rtcValid = false;
  if( ESP.rtcUserMemoryRead(0,(uint32_t*)&rtcData, sizeof(rtcData))) {
    // Calculate the CRC of what we just read from RTC memory, but skip the first 4 bytes as that's the checksum itself.
    uint32_t crc = crc32(((uint8_t*)&rtcData) + 4,sizeof(rtcData) - 4,0xffffffff);
    if( crc == rtcData.crc32 ) {
      rtcValid = true;
    }
  }

  if( rtcValid ) {
    // The RTC data was good, make a quick connection
    Serial.printf("[WIFI] ... quick connect %s, status: %d\nConnecting.",WifiSSID, wifiStatus);
    WiFi.begin( WifiSSID, WifiPass, rtcData.channel, rtcData.bssid, true );
  }
    else {
    // The RTC data was not valid, so make a regular connection
    Serial.printf("[WIFI] ... normal connect %s, status: %d\n",WifiSSID, wifiStatus);
    WiFi.begin( WifiSSID, WifiPass, WifiChannel, WifiBSSID, true );
  }

  while( wifiStatus != WL_CONNECTED ) {
    retries++;
    Serial.write('.');
     if( retries == 100 ) {
       // Quick connect is not working, reset WiFi and try regular connection
       rtcValid = false;
       WiFi.disconnect();
       delay( 10 );
       WiFi.forceSleepBegin();
       delay( 10 );
       WiFi.forceSleepWake();
       delay( 10 );
       WiFi.begin( WifiSSID, WifiPass );
    }
    if( retries >= 600 ) {
      // Giving up after 30 seconds and going back to sleep
      WiFi.disconnect( true );
      delay( 1 );
      WiFi.mode( WIFI_OFF );
      gotoDeepSleep(sleepTimeS);
      return 1; // Not expecting this to be called, the previous call will never return.
    }
    delay( 50 );
    wifiStatus = WiFi.status();
  }

  Serial.print("Connected\n");

  if( rtcValid == false ) {
    // Write current connection info back to RTC
   rtcData.channel = WiFi.channel();
   memcpy( rtcData.bssid, WiFi.BSSID(), 6 ); // Copy 6 bytes of BSSID (AP's MAC address)
   rtcData.crc32 = crc32(((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4,0xffffffff);
   ESP.rtcUserMemoryWrite( 0, (uint32_t*)&rtcData, sizeof( rtcData ) );

   //Check for updates
   checkForUpdates();
 }

return wifiStatus;
}

int ReadSensor() {
  Serial.print("\nRequesting HTU21 Sensor data...\n");

  htu21.begin();

  //Lower resolution for quicker measurements 
  htu21.setResolution(RESOLUTION_RH11_T11);

  //Allow sensor time to refresh, 50ms
  delay(50);
  
  if(htu21.measure()) {
    glb_temp = htu21.getTemperature(); // get temperature in °C
    glb_hum = htu21.getHumidity(); // get humidity in rH%
  }
  else {
    Serial.println("Failed to read from HTU21 sensor!");
    return 1;
  }

  Serial.println("Temp: " + String(glb_temp) + "  Hum: " + String(glb_hum));
  return 0;
}

int getBatteryStatus(){
  glb_batterylevel=ESP.getVcc();
    
  Serial.println("Battery VCC: " + String(glb_batterylevel*0.001)+ "V");

  //Calculate estimated percentage
  if (glb_batterylevel >= 3800) {
    glb_batterylevel = 100;
    }
  else if ( glb_batterylevel <= 3799 || glb_batterylevel >= 3600 ) {
    glb_batterylevel = 50;
    }
  else if ( glb_batterylevel <= 3599 || glb_batterylevel >= 3300 ) {
    glb_batterylevel = 10;
    }
  else {
    //Battery too low (<3300), sleep 6h or forever which ever comes first.
    gotoDeepSleep(21600);
    }

  Serial.println("Battery level: " + String(glb_batterylevel) + "%");
    
  return 0; 
}

int getRSSI(){
  glb_rssi=WiFi.RSSI();

  Serial.println("WiFi RSSI: " + String(glb_rssi) + "dBm");
  
  return 0; 
}

int GetHttpURL(String MyURL){
  Serial.print("[HTTP] begin...\n");

  http.begin(client,String(VeraBaseURL) + MyURL);
    
  int httpCode = http.GET();

   if (httpCode > 0) {
     Serial.printf("[HTTP] GET... code: %d\n", httpCode);
    
     if (httpCode == HTTP_CODE_OK) {
        http.writeToStream(&Serial);
        }
  }
  else {
    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
 
 return httpCode;
}

void setup() {
  //switch radio off to save energy
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();

  //Disable WiFi status LED to save battery
  wifi_status_led_uninstall();

  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.print("\n" + String(ESPName) + " started\n");

  getBatteryStatus();

  int SensorStatus = ReadSensor(); 

  if (SensorStatus > 0) {
    gotoDeepSleep(sleepTimeS);
  }
  
  // Connect WiFi
  connect_wifi();

  getRSSI();

  http.setReuse(true);
  
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraTempDeviceID) + "&serviceId=urn:upnp-org:serviceId:TemperatureSensor1&Variable=CurrentTemperature&Value=" + String(glb_temp));
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraTempDeviceID) + "&serviceId=urn:upnp-org:serviceId:TemperatureSensor1&Variable=CurrentRSSI&Value=" + String(glb_rssi));
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraTempDeviceID) + "&serviceId=urn:micasaverde-com:serviceId:HaDevice1&Variable=BatteryLevel&Value=" + String(glb_batterylevel));
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraHumDeviceID) + "&serviceId=urn:micasaverde-com:serviceId:HumiditySensor1&Variable=CurrentLevel&Value=" + String(glb_hum));

  http.end();
  
  //switch radio off to save energy
  WiFi.mode(WIFI_OFF);

  //Sleep longer if level change is small
  if (abs(abs(glb_temp) - abs(rtcData.old_temp)) >= tempThreshold) {
    updateOldTemp();
    gotoDeepSleep(sleepTimeS);
  }
  else {
    gotoDeepSleep(sleepTimeL);
  }
}

void loop() {
// Nothing Here
}
//EOF
