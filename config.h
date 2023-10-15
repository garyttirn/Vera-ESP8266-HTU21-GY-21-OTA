/* ESP8266 Config*/
IPAddress ip(192,168,1,21);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

const char* WifiSSID      = "SSID";
const char* WifiPass      = "PASSWORD";
uint8_t WifiBSSID[6]      = {0x31,0xE3,0x3D,0x1F,0xF2,0x77};
uint8_t WifiChannel       = 11;
const int sleepTimeS      = 300; // Sleep time in seconds
const int sleepTimeL      = 600; // Sleep time in seconds
const float tempThreshold  = 0.5; // A change greater than levelThreshold will trigger sleepTimeS

int VeraTempDeviceID = 141;
int VeraHumDeviceID = 142;
const char* ESPName = "Vera-htu21";
const char* VeraBaseURL = "http://192.168.1.42:3480/";
const char* UpdateURL = "http://192.168.1.254:4080/8266OTA.php";
const char* FWVersion = "15102023";
