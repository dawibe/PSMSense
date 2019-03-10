// ESP Gies-O-Mat MQTT Soil Moisture Controller
// Plant Soil Moisture Sense (PSMSense)
//
// Arduino IDE 1.8.5
// ESP8266 2.4.1
// WIFIManager 0.12.0
// 
// Arduino Config
// 4M (1M SPIFFS);v2 Prebuild (MSS=536); 160MHz; 
//
// ToDo
// - Values via JSON
// - 1-Wire Temp Sensor
// - measure battery voltage
// - input check
// - NTP timezone selection
//
// Bugs
// - WLAN reconnect partly not working in Battery Mode (system stay in wlan autoconfig mode)
//
// 0.1.0 First stable working version (@160MHz)
// 0.1.1 MQTT is working
// 0.1.2 Battery Mode
// 0.1.3 NTP
// 0.1.4 Added configurable measure interval
// 0.1.5 Changed position after decimal point to two
// 0.1.6 Added Multiplexer support for max 8 Sensors with CD74AC151
// 0.1.7 Added Sensor enable output mode setting
// 0.1.8 Support for analog sensors added (different Multiplexer needed!)
// 0.1.9 Changed direct function call via ticker to flag based via loop



// **************************************************************************
//  Includes
// **************************************************************************
#include <ESP8266WiFi.h>
//#include <WiFiClient.h>
#include <DNSServer.h>                  // Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>           // Local WebServer used to serve the configuration portal
#include <WiFiManager.h>                // https://github.com/tzapu/WiFiManager WiFi Configuration Magic
//#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <Ticker.h>                     // Periodically call of a function
#include <ArduinoJson.h>                // Config file handling
#include <TimeLib.h>                    // https://github.com/PaulStoffregen/Time
#include "FS.h"

#define MQTT_VERSION MQTT_VERSION_3_1   // That's needed to change in PubSubClient.h !!!
#include <PubSubClient.h>               // MQTT


ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

Ticker startMeasure;

WiFiClient espClient;                   // Only needed for MQTT PubSubClient!
PubSubClient client(espClient);         // Enable MQTT

WiFiUDP Udp;                            // Needed fpr NTP
unsigned int NtpLocalPort = 123;        // local port to listen for UDP packets (NTP)
  
// **************************************************************************
//  Defines (Do not use D3, D4 and D8)
// **************************************************************************
//#define DEBUG                         // Define for Debug output on serial interface      
#define SENSOR_ENABLE           D1    // Gies-o-mat power supply (you can connect VCC of the Sensore direcly to this pin if <20mA!)
#define SENSOR_IN               D7    // Gies-o-mat sensor input
#define SENSOR_IN_ANALOG        A0    // Analog sensor input
#define LED_PIN                 D4    // unused
#define CLEAR_BTN               D2    // LOW during PowerUp will clear the json config file
#define POWER_MODE              D5    // High for Power Supply, Low for Battery Mode
#define DATA_SOURCE_SELECTOR_A  D6    // Multiplexer Address Bit A
#define DATA_SOURCE_SELECTOR_B  D9   // Multiplexer Address Bit B
#define DATA_SOURCE_SELECTOR_C  D10   // Multiplexer Address Bit C

const String FIRMWARE_NAME = "PSM-Sense";
const String VERSION       = "v0.1.9";

// **************************************************************************
// **************************************************************************
//  DO NOT CHANGE ANYTHING BELOW THIS LINE
// **************************************************************************
// **************************************************************************

// **************************************************************************
//  Debug
// **************************************************************************
#ifdef DEBUG
#define DEBUG_PRINT(x)  Serial.print (x)
#define DEBUG_PRINTLN(x)  Serial.println (x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif


// **************************************************************************
//  Variables
// **************************************************************************
// config.json
char host_name[20] = "";
char ntpserver[30] = "";
char mqtt_server[30] = "";
char mqtt_topic[30] = "";
bool mqtt_enabled = true;
char defaultmqtttopic[30] = "Sensors/Plant/Moisture";
int power_sampling_interval = 10;                    // Measure Frequency in Power Mode in Minutes
int bat_sampling_interval = 10;                      // Measure Frequency in Battery Mode in Minutes
int sensors_count = 1;                               // How many sensors connected (via Multiplexer if >1)
bool sens_enable_mode = true;                        // Sensor enable PIN High or Low active (True = High Active)
bool sensor_type_analog = false;                     // Sensor Type, true for analog sensor and false for Giesomat sensor

//NTP
bool ntp_enabled = false;                            // Set to false to disable querying for the time
char poolServerName[30] = "europe.pool.ntp.org";     // default NTP Server when not configured in config.json
char boottime[20] = "";                              // PSMSene Boot Time
const int timeZone = 1;                              // Central European Time
const int NTP_PACKET_SIZE = 48;                      // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE];                  // Buffer to hold incoming & outgoing packets

// Webserver
String javaScript;
String htmlHeader;
String htmlFooter;

// Interrupt
volatile unsigned long interruptCounter = 0;    // volatile because of ISR

// Other
bool shouldSaveConfig = false;                  // Flag for saving data
float freqValue[8] = {0};                       // Sensor Frequency
int analogValue[8] = {0};                       // Sensor Voltage
bool powermode = true;                          // Power Mode True for Power Supply, False for Battery Mode
bool func_measure_call = false;                 // Flag for jump into the measure function


// **************************************************************************
// JSON Configuration Management
// **************************************************************************
bool loadConfig() {
  if (SPIFFS.exists("/config.json")) {
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        DEBUG_PRINTLN("opened config file");
        size_t size = configFile.size();
        if (size > 1024) {
          DEBUG_PRINTLN("Config file size is too large");
          return false;
        }
        std::unique_ptr<char[]> buf(new char[size]);            // Allocate a buffer to store contents of the file.
        configFile.readBytes(buf.get(), size);                  // Input Buffer (needed by ArduinoJson)
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        #ifdef DEBUG
          json.printTo(Serial);
        #endif
        if (json.success()) {
          DEBUG_PRINTLN("\nparsed json");
          if (json.containsKey("hostname")) strncpy(host_name, json["hostname"], 20);
          if (json.containsKey("mqttserver")) strncpy(mqtt_server, json["mqttserver"], 30);
          if (json.containsKey("mqtttopic")) strncpy(mqtt_topic, json["mqtttopic"], 30);
          if (json.containsKey("mqttenabled")) mqtt_enabled = json["mqttenabled"];
          if (json.containsKey("ntpserver")) strncpy(ntpserver, json["ntpserver"], 30);
          if (json.containsKey("ntpenabled")) ntp_enabled = json["ntpenabled"];
          if (json.containsKey("sampleintpower")) power_sampling_interval = json["sampleintpower"];
          if (json.containsKey("sampleintbat")) bat_sampling_interval = json["sampleintbat"];
          if (json.containsKey("sensorscount")) sensors_count = json["sensorscount"];
          if (json.containsKey("sensorsenablemode")) sens_enable_mode = json["sensorsenablemode"];
          if (json.containsKey("sensortype")) sensor_type_analog = json["sensortype"];
        }
        else {
          DEBUG_PRINTLN("Failed to parse config file");
          return false;
        }
      }
      else {
        DEBUG_PRINTLN("Failed to open config file");
        return false;
      }
  }
  return true;
}


// **************************************************************************
//  Callback notifying us of the need to save config
// **************************************************************************
void saveConfigCallback ()
{
  DEBUG_PRINTLN("Should save config");
  shouldSaveConfig = true;
}


// **************************************************************************
//  Gets called when WiFiManager enters configuration mode
// **************************************************************************
void configModeCallback (WiFiManager *myWiFiManager)
{
  DEBUG_PRINTLN("Entered config mode");
  DEBUG_PRINTLN(WiFi.softAPIP());
  DEBUG_PRINTLN(myWiFiManager->getConfigPortalSSID());    //if you used auto generated SSID, print it
}


// **************************************************************************
//  IP Address to String
// **************************************************************************
String ipToString(IPAddress ip)
{
  String s = "";
  for (int i = 0; i < 4; i++)
    s += i  ? "." + String(ip[i]) : String(ip[i]);
  return s;
}


// **************************************************************************
//  NTP Time Syncronization
// **************************************************************************
time_t getNtpTime()
{
  IPAddress ntpServerIP;                            // NTP server's ip address

  while (Udp.parsePacket() > 0) ;                   // Discard any previously received packets
  DEBUG_PRINTLN("NTP: Transmit NTP Request");
  WiFi.hostByName(ntpserver, ntpServerIP);          // Lookup IP from Hostname
  DEBUG_PRINT("NTP: ");
  DEBUG_PRINT(ntpserver);
  DEBUG_PRINT(" IP: ");
  DEBUG_PRINTLN(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      DEBUG_PRINTLN("NTP: Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + (timeZone * 60 * 60); // NTP Time - 70 Years + TimeZone Hours
    }
  }
  DEBUG_PRINTLN("NTP: No NTP Response :-(");
  return 0;                                          // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  memset(packetBuffer, 0, NTP_PACKET_SIZE);                 // set all bytes in the buffer to 0
  packetBuffer[0] = 0b11100011;                             // LI (Clock is unsynchronized), Version (4), Mode (Client)
  packetBuffer[1] = 0;                                      // Stratum, or type of clock (Unspecified or invalid)
  packetBuffer[2] = 6;                                      // Polling Interval (log2 seconds)
  packetBuffer[3] = 0xEC;                                   // Peer Clock Precision (log2 seconds)
                                                            // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;                                    // 
  packetBuffer[13] = 0x4E;                                  //
  packetBuffer[14] = 49;                                    //
  packetBuffer[15] = 52;                                    //
  Udp.beginPacket(address, 123);                            // NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}


// **************************************************************************
//  HTML Header Template
// **************************************************************************
void buildHeader()
{
  htmlHeader="<!DOCTYPE html PUBLIC '-//W3C//DTD XHTML 1.0 Strict//EN' 'http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd'>\n";
  htmlHeader+="<html xmlns='http://www.w3.org/1999/xhtml' xml:lang='en'>\n";
  htmlHeader+="  <head>\n";
  htmlHeader+="    <meta name='viewport' content='width=device-width, initial-scale=.75' />\n";
  htmlHeader+="    <link rel='stylesheet' href='https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/css/bootstrap.min.css' />\n";
  htmlHeader+="    <style>@media (max-width: 991px) {.nav-pills>li {float: none; margin-left: 0; margin-top: 5px; text-align: center;}}</style>\n";
  htmlHeader+="    <title>" + FIRMWARE_NAME + " - " + VERSION + "</title>\n";
  htmlHeader+="  </head>\n";
  htmlHeader+="  <body>\n";
  htmlHeader+="    <div class='container'>\n";
  htmlHeader+="      <h1>" + FIRMWARE_NAME + " - " + VERSION + "</h1>\n";
  htmlHeader+="      <div class='row'>\n";
  htmlHeader+="        <div class='col-md-12'>\n";
  htmlHeader+="          <ul class='nav nav-pills'>\n";
  htmlHeader+="            <li class='active'>\n";
  htmlHeader+="              <a href='#'>Hostname <span class='badge'>" + String(host_name) + "</span></a></li>\n";
  htmlHeader+="            <li class='active'>\n";
  htmlHeader+="              <a href='http://" + ipToString(WiFi.localIP()) + ":80'>Local IP<span class='badge'>" + ipToString(WiFi.localIP()) + "</span></a></li>\n";
  htmlHeader+="            <li class='active'>\n";
  htmlHeader+="              <a href='#'>MAC <span class='badge'>" + String(WiFi.macAddress()) + "</span></a></li>\n";
  htmlHeader+="            <li class='active'>\n";
  htmlHeader+="              <a href='/config'>Configuration</a></li>\n";
  htmlHeader+="            <li class='active'>\n";
  htmlHeader+="              <a href='#'><span class='glyphicon glyphicon-signal'></span> "+ String(WiFi.RSSI()) + " dBm</a></li>\n";
  htmlHeader+="          </ul>\n";
  htmlHeader+="        </div>\n";
  htmlHeader+="      </div><hr />\n";
}


// **************************************************************************
//  HTML Footer Template
// **************************************************************************
void buildFooter()
{
  htmlFooter="      <div class='row'><div class='col-md-12'><em>MQTT Plant Soil Moisture Sense Gies-O-Mat Zeit: "+ String(hour()) + ":" + String(minute()) + ":" + String(second()) +"</em></div></div>\n";
  htmlFooter+="    </div>\n";
  htmlFooter+="  </body>\n";
  htmlFooter+="</html>\n";
}


// **************************************************************************
//  HTML Page for ESP Reboot
// **************************************************************************
void Handle_Reboot()
{
  httpServer.sendHeader("Connection", "close");
  httpServer.send(200, "text/html", F("<body>Reboot OK, please reload page.</body>"));
  delay(500);
  ESP.restart();
}

// **************************************************************************
//  HTML Page for ESP Clear Json config-file
// **************************************************************************
void Handle_ClearConfig()
{
  httpServer.sendHeader("Connection", "close");
  httpServer.send(200, "text/html", F("<body>Config File deleted, ESP reboot, please reload page.</body>"));
  SPIFFS.remove("/config.json");
  delay(500);
  ESP.restart();
}

// **************************************************************************
//  HTML Page for ESP Clear Json config-file
// **************************************************************************
void Handle_config()
{
  if (httpServer.method() == HTTP_GET) {
    DEBUG_PRINTLN("WEB: Connection received - /config");
    sendConfigPage("", "", 0, 200);
  } else {
    DEBUG_PRINTLN("WEB: Connection received - /config (save)");
    sendConfigPage("Settings saved successfully! Please Reboot!", "Success!", 1, 200);
    }
}

void sendConfigPage(String message, String header, int type, int httpcode)
{
  char host_name_conf[20] = "";
  char mqtt_server_conf[30] = "";
  char mqtt_topic_conf[30] = "";
  char ntpserver_conf[30] = "";
  bool ntpenabled_conf;
  bool mqttenabled_conf;
  int power_sampling_interval_conf;
  int bat_sampling_interval_conf;
  int sensors_count_conf;
  bool sens_enable_mode_conf;
  bool sensor_type_analog_conf;

  if (type == 1){                                              // Type 1 -> save data
    String message = "WEB: Number of args received:";
    message += String(httpServer.args()) + "\n";
    for (int i = 0; i < httpServer.args(); i++) {
      message += "Arg " + (String)i + " â€“> ";
      message += httpServer.argName(i) + ":" ;
      message += httpServer.arg(i) + "\n";
    }
    if (httpServer.hasArg("ntpenabled")) {ntpenabled_conf = true;} else {ntpenabled_conf = false;}
    if (httpServer.hasArg("mqttenabled")) {mqttenabled_conf = true;} else {mqttenabled_conf = false;}
    strncpy(host_name_conf, httpServer.arg("host_name_conf").c_str(), 20);
    strncpy(mqtt_server_conf, httpServer.arg("mqtt_server_conf").c_str(), 30);
    strncpy(mqtt_topic_conf, httpServer.arg("mqtt_topic_conf").c_str(), 30);
    strncpy(ntpserver_conf, httpServer.arg("ntpserver_conf").c_str(), 30);
    power_sampling_interval_conf = atoi(httpServer.arg("power_sampling_interval_conf").c_str());
    bat_sampling_interval_conf = atoi(httpServer.arg("bat_sampling_interval_conf").c_str());
    sensors_count_conf = atoi(httpServer.arg("sensors_count_conf").c_str());
    if (httpServer.arg("sensorsenablemode") == "High") {sens_enable_mode_conf = true;} else {sens_enable_mode_conf = false;}
    if (httpServer.arg("sensortype") == "Analog") {sensor_type_analog_conf = true;} else {sensor_type_analog_conf = false;}

    DEBUG_PRINTLN(message);

    // validate values before saving
    bool validconf = true;
    if (validconf)
    {
      DEBUG_PRINTLN("SPI: save config.json...");
      DynamicJsonBuffer jsonBuffer;
      JsonObject& json = jsonBuffer.createObject();
      json["hostname"] = String(host_name_conf);
      json["mqttserver"] = String(mqtt_server_conf);
      json["mqtttopic"] = String(mqtt_topic_conf);
      json["mqttenabled"] = String(mqttenabled_conf);
      json["ntpserver"] = String(ntpserver_conf);
      json["ntpenabled"] = String(ntpenabled_conf);
      json["sampleintpower"] = String(power_sampling_interval_conf);
      json["sampleintbat"] = String(bat_sampling_interval_conf);
      json["sensorscount"] = String(sensors_count_conf);
      json["sensorsenablemode"] = String(sens_enable_mode_conf);
      json["sensortype"] = String(sensor_type_analog_conf);

      File configFile = SPIFFS.open("/config.json", "w");
      if (!configFile) {
        DEBUG_PRINTLN("SPI: failed to open config file for writing");
      }
      #ifdef DEBUG
        json.printTo(Serial);
      #endif
      DEBUG_PRINTLN("");
      json.printTo(configFile);
      configFile.close();
      //end save
    }
  } else {                                // Type 0 -> load data
    if (SPIFFS.begin())
    {
      DEBUG_PRINTLN("SPI: mounted file system");
      if (SPIFFS.exists("/config.json"))
      {
        //file exists, reading and loading
        DEBUG_PRINTLN("SPI: reading config file");
        File configFile = SPIFFS.open("/config.json", "r");
        if (configFile) {
          DEBUG_PRINTLN("SPI: opened config file");
          size_t size = configFile.size();
          // Allocate a buffer to store contents of the file.
          std::unique_ptr<char[]> buf(new char[size]);

          configFile.readBytes(buf.get(), size);
          DynamicJsonBuffer jsonBuffer;
          JsonObject& json = jsonBuffer.parseObject(buf.get());
          DEBUG_PRINT("JSO: ");
          #ifdef DEBUG
            json.printTo(Serial);
          #endif
          if (json.success()) {
            DEBUG_PRINTLN("\nJSO: parsed json");
            if (json.containsKey("hostname")) strncpy(host_name_conf, json["hostname"], 20);
            if (json.containsKey("mqttserver")) strncpy(mqtt_server_conf, json["mqttserver"], 30);
            if (json.containsKey("mqtttopic")) strncpy(mqtt_topic_conf, json["mqtttopic"], 30);
            if (json.containsKey("mqttenabled")) mqttenabled_conf = json["mqttenabled"];
            if (json.containsKey("ntpserver")) strncpy(ntpserver_conf, json["ntpserver"], 30);
            if (json.containsKey("ntpenabled")) ntpenabled_conf = json["ntpenabled"];
            if (json.containsKey("sampleintpower")) power_sampling_interval_conf = json["sampleintpower"];
            if (json.containsKey("sampleintbat")) bat_sampling_interval_conf = json["sampleintbat"];
            if (json.containsKey("sensorscount")) sensors_count_conf = json["sensorscount"];
            if (json.containsKey("sensorsenablemode")) sens_enable_mode_conf = json["sensorsenablemode"];
            if (json.containsKey("sensortype")) sensor_type_analog_conf = json["sensortype"];
          } else {
            DEBUG_PRINTLN("JSO: failed to load json config");
          }
        }
      }
    } else {
      DEBUG_PRINTLN("SPI: failed to mount FS");
    }
  }
  String htmlDataconf;        // Hold the HTML Code
  buildHeader();
  buildFooter();
  htmlDataconf=htmlHeader;
  
  if (type == 1)
    htmlDataconf+="      <div class='row'><div class='col-md-12'><div class='alert alert-success'><strong>" + header + "!</strong> " + message + "</div></div></div>\n";
  if (type == 2)
    htmlDataconf+="      <div class='row'><div class='col-md-12'><div class='alert alert-warning'><strong>" + header + "!</strong> " + message + "</div></div></div>\n";
  if (type == 3)
    htmlDataconf+="      <div class='row'><div class='col-md-12'><div class='alert alert-danger'><strong>" + header + "!</strong> " + message + "</div></div></div>\n";
  htmlDataconf+="      <div class='row'>\n";
  htmlDataconf+="<form method='post' action='/config'>";
  htmlDataconf+="        <div class='col-md-12'>\n";
  htmlDataconf+="          <h3>Configuration</h3>\n";
  htmlDataconf+="          <table class='table table-striped' style='table-layout: fixed;'>\n";
  htmlDataconf+="            <thead><tr><th>Option</th><th>Current Value</th><th>New Value</th></tr></thead>\n";
  htmlDataconf+="            <tbody>\n";
  htmlDataconf+="            <tr class='text-uppercase'><td>Hostname</td><td><code>" + ((host_name_conf[0] == 0 ) ? String("(" + String(host_name) + ")") : String(host_name_conf)) + "</code></td><td><input type='text' id='host_name_conf' name='host_name_conf' value='" + String(host_name_conf) + "'></td></tr>\n";
  htmlDataconf+="            <tr class='text-uppercase'><td>MQTT Server</td><td><code>" + String(mqtt_server_conf) + "</code></td><td><input type='text' id='mqtt_server_conf' name='mqtt_server_conf' value='" + String(mqtt_server_conf) + "'></td></tr>\n";
  htmlDataconf+="            <tr class='text-uppercase'><td>MQTT Topic</td><td><code>" + String(mqtt_topic_conf) + "</code></td><td><input type='text' id='mqtt_topic_conf' name='mqtt_topic_conf' value='" + String(mqtt_topic_conf) + "'></td></tr>\n";
  htmlDataconf+="            <tr class='text-uppercase'><td>MQTT enabled?</td><td><code>" + (mqttenabled_conf ? String("Yes") : String("No")) + "</code></td><td><input type='checkbox' id='mqttena' name='mqttenabled' " + (mqttenabled_conf ? String("checked") : String("")) + "></td></tr>";
  htmlDataconf+="            <tr class='text-uppercase'><td>NTP Server</td><td><code>" + ((ntpserver_conf[0] == 0 ) ? String("(" + String(poolServerName) + ")") : String(ntpserver_conf)) + "</code></td><td><input type='text' id='ntpserver_conf' name='ntpserver_conf' value='" + String(ntpserver_conf) + "'></td></tr>\n";
  htmlDataconf+="            <tr class='text-uppercase'><td>NTP enabled?</td><td><code>" + (ntpenabled_conf ? String("Yes") : String("No")) + "</code></td><td><input type='checkbox' id='ntpena' name='ntpenabled' " + (ntpenabled_conf ? String("checked") : String("")) + "></td></tr>";
  htmlDataconf+="            <tr class='text-uppercase'><td>Measure Freq Power Mode (minutes)</td><td><code>" + String(power_sampling_interval_conf) + "</code></td><td><input type='text' id='power_sampling_interval_conf' name='power_sampling_interval_conf' value='" + String(power_sampling_interval_conf) + "'></td></tr>\n";
  htmlDataconf+="            <tr class='text-uppercase'><td>Measure Freq Battery Mode (minutes)</td><td><code>" + String(bat_sampling_interval_conf) + "</code></td><td><input type='text' id='bat_sampling_interval_conf' name='bat_sampling_interval_conf' value='" + String(bat_sampling_interval_conf) + "'></td></tr>\n";
  htmlDataconf+="            <tr class='text-uppercase'><td>Number of Sensors</td><td><code>" + String(sensors_count_conf) + "</code></td><td><select name='sensors_count_conf' size='1'><option " + ((sensors_count_conf == 1 ) ? String("selected") : String("")) + ">1</option><option "+ ((sensors_count_conf == 2 ) ? String("selected") : String("")) +">2</option><option "+ ((sensors_count_conf == 3 ) ? String("selected") : String("")) +">3</option><option "+ ((sensors_count_conf == 4 ) ? String("selected") : String("")) +">4</option><option "+ ((sensors_count_conf == 5 ) ? String("selected") : String("")) +">5</option><option "+ ((sensors_count_conf == 6 ) ? String("selected") : String("")) +">6</option><option "+ ((sensors_count_conf == 7 ) ? String("selected") : String("")) +">7</option><option "+ ((sensors_count_conf == 8 ) ? String("selected") : String("")) +">8</option></select></td></tr>\n";
  htmlDataconf+="            <tr class='text-uppercase'><td>Sensor Enable Port</td><td><code>" + (sens_enable_mode_conf ? String("High active") : String("Low active")) + "</code></td><td><select name='sensorsenablemode' size='1'><option " + ((sens_enable_mode_conf ) ? String("selected") : String("")) + ">High</option><option "+ ((!sens_enable_mode_conf ) ? String("selected") : String("")) +">Low</option></select></td></tr>";
  htmlDataconf+="            <tr class='text-uppercase'><td>Sensor Type</td><td><code>" + (sensor_type_analog_conf ? String("Analog") : String("Frequency")) + "</code></td><td><select name='sensortype' size='1'><option " + ((sensor_type_analog_conf ) ? String("selected") : String("")) + ">Analog</option><option "+ ((!sensor_type_analog_conf ) ? String("selected") : String("")) +">Frequency</option></select></td></tr>";
  htmlDataconf+=" <tr><td colspan='5' class='text-center'><em><a href='/reboot' class='btn btn-sm btn-danger'>Reboot</a>  <a href='/update' class='btn btn-sm btn-warning'>Update</a>  <button type='submit' class='btn btn-sm btn-success'>Save</button>  <a href='/' class='btn btn-sm btn-primary'>Cancel</a></em></td></tr>";
  htmlDataconf+="            </tbody></table>\n";
  htmlDataconf+="          </div></div>\n";
  
  htmlDataconf+=htmlFooter;

  httpServer.send(httpcode, "text/html; charset=utf-8", htmlDataconf);
  httpServer.client().stop();
}

// **************************************************************************
//  HTML get Value Page
// **************************************************************************
void Handle_getvalue(){
  String htmlDataconf;        // Hold the HTML Code

  for (int i = 0; i < sensors_count ; i++) {
    if (sensor_type_analog == true) {
      htmlDataconf+="" + String(analogValue[i]) + "";
    }
    else {
      htmlDataconf+="" + String(freqValue[i]) + "";
    }
    if ((i+1) < sensors_count)
      htmlDataconf+=";";
  }
  
  httpServer.send(200, "text/html; charset=utf-8", htmlDataconf);
  httpServer.client().stop(); 
}

// **************************************************************************
//  HTML Main Page
// **************************************************************************
void Handle_welcome(){
  
  String htmlDataMain;        // Hold the HTML Code
  buildHeader();
  buildFooter();
  htmlDataMain=htmlHeader;
  
  htmlDataMain+="        <div class='col-md-12'>\n";
  htmlDataMain+="          <h3>Plant Soil Moisture Sense</h3><br>";
  for (int i = 0; i < sensors_count ; i++) {
    if (sensor_type_analog == true) {
      htmlDataMain+="Last Value Sensor " + String(i+1) + ": " + String(analogValue[i]) + " mV<br><br>";
    }
    else {
      htmlDataMain+="Last Value Sensor " + String(i+1) + ": " + String(freqValue[i]) + " kHz<br><br>";
    }
  }
  htmlDataMain+="           Possible URL Prefix:<br>";
  htmlDataMain+="           /reboot<br>";
  htmlDataMain+="           /clearconfig<br>";
  htmlDataMain+="           /freemem<br>";
  htmlDataMain+="           /getvalue<br>";
  htmlDataMain+="           /config<br><br><br>";
  htmlDataMain+="          </div><br>";
  
  htmlDataMain+=htmlFooter;

  httpServer.send(200, "text/html; charset=utf-8", htmlDataMain);
  httpServer.client().stop();  
}

// **************************************************************************
//  MQTT reconnect
// **************************************************************************
void reconnectMQTT() {
    DEBUG_PRINT("Attempting MQTT connection...");
    if (client.connect(host_name)) {
      DEBUG_PRINTLN("connected");
    }
    else DEBUG_PRINTLN("not connected");
}


// **************************************************************************
//  MQTT Publish Frequency to MQTT Broker
// **************************************************************************
void sendMQTT() {
  bool mqttresult;
  char mqtt_humidity_topic[40] = "";
  char sensornr[2] = "";

  for (int i = 0; i < sensors_count ; i++) {
    sensornr[0] = i + '1';                          // Convert int to char, Array is needed for strcat
    strcat (mqtt_humidity_topic,mqtt_topic);
    strcat (mqtt_humidity_topic,"/humidity/");
    strcat (mqtt_humidity_topic,sensornr);
    if (sensor_type_analog == true) {
      mqttresult = client.publish(mqtt_humidity_topic, String(analogValue[i]).c_str(), true);
    }
    else {
      mqttresult = client.publish(mqtt_humidity_topic, String(freqValue[i]).c_str(), true);
    }
    if (!mqttresult) {
      DEBUG_PRINTLN("MQTT publish failed ...");
    } else {
      DEBUG_PRINTLN("MQTT publish successful ...");
    }
    mqtt_humidity_topic[0] = '\0';                  // Empty the array that strcat can work again
  }
}


// **************************************************************************
//  ISR for frequency counting
// **************************************************************************
void handleInterrupt() {
  interruptCounter++;
}


// **************************************************************************
//  Set the measure flag, called by ticker
// **************************************************************************
void set_measure_flag() {
  func_measure_call = true;
}


// **************************************************************************
//  Start measuring
// **************************************************************************
void measureFreq() {
  DEBUG_PRINTLN("Starting measurement ...");
  interruptCounter = 0;                                // Initialize with 0 to avoid partly wrong measue values
  int startInt = 0;
  
  if (sens_enable_mode == true) {
    digitalWrite(SENSOR_ENABLE, HIGH);                 // Enable Sensor Power (High active mode)
  }
  else {
    digitalWrite(SENSOR_ENABLE, LOW);                  // Enable Sensor Power (Low active mode)
  }
    
  delay(100);                                          // Wait for sensor stabilization
  
  if (sensor_type_analog == true) {
    //delay(500);                                       // A little bit longer for analog sensor
    startInt = millis();
    while ((millis()-startInt) <= 500);
  }

  // Start measuring for each sensor seperatly
  for (int i = 0; i < sensors_count ; i++) {
    // Set the address for multiplexer
    digitalWrite(DATA_SOURCE_SELECTOR_A, bitRead(i,0));
    digitalWrite(DATA_SOURCE_SELECTOR_B, bitRead(i,1));
    digitalWrite(DATA_SOURCE_SELECTOR_C, bitRead(i,2));
    delay(100);                                       // Wait some time for multiplexer
    
    if (sensor_type_analog == true) {                 // Analog sensor, read ADC value
      analogValue[i] = map(analogRead(SENSOR_IN_ANALOG), 0, 1023, 0, 3300);
      DEBUG_PRINTLN("Analog Sensor " + String(i) + ": " + String(analogValue[i]) + " mV Uptime: " + millis()+ " ms (" + String(hour()) + ":" + String(minute()) + ":" + String(second()) + ")");
    }
    else {                                            // Giesomat, read frequency
      int startInt = 0;
      startInt = millis();
      // Declaration of ISR
      attachInterrupt(digitalPinToInterrupt(SENSOR_IN), handleInterrupt, RISING);
      while ((millis()-startInt) <= 40);
      detachInterrupt(digitalPinToInterrupt(SENSOR_IN));
      freqValue[i] = interruptCounter;
      interruptCounter = 0;
      freqValue[i] = freqValue[i]/40.0;
      DEBUG_PRINTLN("Frequency Sensor " + String(i) + ": " + String(freqValue[i]) + " kHz Uptime: " + millis()+ " ms (" + String(hour()) + ":" + String(minute()) + ":" + String(second()) + ")");
    }
  }
  
  if (sens_enable_mode == true) {
    digitalWrite(SENSOR_ENABLE, LOW);                  // Disable Sensor Power
  }
  else {
    digitalWrite(SENSOR_ENABLE, HIGH);                 // Disable Sensor Power
  }

    digitalWrite(DATA_SOURCE_SELECTOR_A, LOW);
    digitalWrite(DATA_SOURCE_SELECTOR_B, LOW);
    digitalWrite(DATA_SOURCE_SELECTOR_C, LOW);
      
  if ((mqtt_enabled) && (client.connected()))
    sendMQTT();
}


// **************************************************************************
//  Setup
// **************************************************************************
void setup(void){

  #ifdef DEBUG
    Serial.begin(115200);
    Serial.println();
    Serial.println("Booting ...");
  #endif
  
  pinMode(CLEAR_BTN, INPUT_PULLUP);
  pinMode(SENSOR_IN, INPUT_PULLUP);
  pinMode(SENSOR_ENABLE, OUTPUT);
  pinMode(POWER_MODE, INPUT_PULLUP);

  pinMode(DATA_SOURCE_SELECTOR_A, OUTPUT);
  pinMode(DATA_SOURCE_SELECTOR_B, OUTPUT);
  pinMode(DATA_SOURCE_SELECTOR_C, OUTPUT);

  digitalWrite(DATA_SOURCE_SELECTOR_A, LOW);
  digitalWrite(DATA_SOURCE_SELECTOR_B, LOW);
  digitalWrite(DATA_SOURCE_SELECTOR_C, LOW);

  if (digitalRead(POWER_MODE) == HIGH) {
    powermode = true;
  } else {
    powermode = false;
  }
  
  WiFiManager wifiManager;
  if (digitalRead(CLEAR_BTN) == LOW) wifiManager.resetSettings();   // Clear WIFI data if CLEAR Button is pressed during boot

  wifiManager.setAPCallback(configModeCallback);          // set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setSaveConfigCallback(saveConfigCallback);  // set config save notify callback
  
  if (SPIFFS.begin()) {                                   // Mounting File System
    DEBUG_PRINTLN("mounted file system");
    if (!loadConfig()) {
      DEBUG_PRINTLN("Failed to load config");
    } else {
      DEBUG_PRINTLN("Config loaded");
    }
  }
  else {
    DEBUG_PRINTLN("failed to mount FS");
  }

  if (sens_enable_mode == true) {
    digitalWrite(SENSOR_ENABLE, LOW);                // Disable Sensor Power (High active mode)
  }
  else {
    digitalWrite(SENSOR_ENABLE, HIGH);               // Disable Sensor Power (Low active mode)
  }

  // Configure some additional 
  WiFiManagerParameter custom_hostname("hostname", "Optional Hostname", host_name, 20);
  wifiManager.addParameter(&custom_hostname);
  WiFiManagerParameter custom_mqttserver("mqttserver", "Optional MQTT Server", mqtt_server, 30);
  wifiManager.addParameter(&custom_mqttserver);
  WiFiManagerParameter custom_mqtttopic("mqtttopic", "Optional MQTT Topic", mqtt_topic, 30);
  wifiManager.addParameter(&custom_mqtttopic);
  WiFiManagerParameter custom_ntpserver("ntpserver", "Optional NTP Server", ntpserver, 30);
  wifiManager.addParameter(&custom_ntpserver);

  String autoconf_ssid = "PSMSense_Config_"+String(ESP.getChipId());
  wifiManager.setConnectTimeout(60);                                  // Workaround Test for reconnect issue
  wifiManager.setConfigPortalTimeout(120);                            // Timeout for SoftAP, try connect again to stored wlan
  wifiManager.autoConnect(autoconf_ssid.c_str());                     // Use PSMSense_Config_+Chip ID as AP-name with 192.168.4.1 as IP
  
  // Read and save the new values from AP config page
  strncpy(host_name, custom_hostname.getValue(), 20);
  strncpy(mqtt_server, custom_mqttserver.getValue(), 30);
  strncpy(mqtt_topic, custom_mqtttopic.getValue(), 30);
  strncpy(ntpserver, custom_ntpserver.getValue(), 30);

  DEBUG_PRINTLN("WiFi connected! IP: " + ipToString(WiFi.localIP()) + " Hostname: " + String(host_name) + " NTP-Server: " + String(ntpserver) + " MQTT Server: " + String(mqtt_server) + " MQTT Topic: " + String(mqtt_topic));

  // save the custom parameters to FS
  if (shouldSaveConfig) {
    DEBUG_PRINTLN(" config...");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["hostname"] = host_name;
    json["mqttserver"] = mqtt_server;
    json["mqtttopic"] = mqtt_topic;
    json["ntpserver"] = ntpserver;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      DEBUG_PRINTLN("SPI: failed to open config file for writing config");
    }
    #ifdef DEBUG
      json.printTo(Serial);
    #endif
    DEBUG_PRINTLN("");
    json.printTo(configFile);
    configFile.close();
  }

  if (host_name[0] == 0 ) strncpy(host_name, "PSMSense", 20);         //set default hostname when not set!
  if (mqtt_topic[0] == 0 ) strncpy(mqtt_topic, defaultmqtttopic, 30); //set default ntp server when not set!
  if (ntpserver[0] == 0 ) strncpy(ntpserver, poolServerName, 30);     //set default ntp server when not set!
  
  //MDNS.begin(host);

  // Enable the Web Services, only in Power Supply Mode
  if (powermode) { 
    // Enable the Free Memory Page
    httpServer.on("/freemem", []() {
      DEBUG_PRINTLN("WEB: Connection received: /freemem : ");
      DEBUG_PRINT(ESP.getFreeSketchSpace());
      httpServer.sendHeader("Connection", "close");
      httpServer.send(200, "text/plain", String(ESP.getFreeSketchSpace()).c_str());
    });

    httpServer.on("/reboot", Handle_Reboot);              // Reboot the ESP Controller
  
    httpServer.on("/clearconfig", Handle_ClearConfig);    // Delete the Json config file on Filesystem
  
    httpServer.on("/config", Handle_config);              // Show the configuration page
  
    httpServer.on("/getvalue", Handle_getvalue);          // Return the current frequency value as plain text without html header
    
    httpServer.on("/", Handle_welcome);                   // Show the main page
  
    httpUpdater.setup(&httpServer);                       // Enable the OTA
    httpServer.begin();                                   // Enable the WebServer
  }

  if (mqtt_enabled)
    client.setServer(mqtt_server, 1883);                  // Set the MQTT Broker

  //setTime(1514764800);  // 1.1.2018 00:00 Initialize time
  if (ntp_enabled) {
      DEBUG_PRINTLN("NTP: Starting UDP");
      Udp.begin(NtpLocalPort);
      DEBUG_PRINT("NTP: Local port: ");
      DEBUG_PRINTLN(Udp.localPort());
      DEBUG_PRINTLN("NTP: waiting for sync");
      setSyncProvider(getNtpTime);                       // set the external time provider
      setSyncInterval(3600);                             // set the number of seconds between re-sync
      //String boottimetemp = printDigits2(hour()) + ":" + printDigits2(minute()) + " " + printDigits2(day()) + "." + printDigits2(month()) + "." + String(year());
      //strncpy(boottime, boottimetemp.c_str(), 20);           // If we got time set boottime
  }

  if (mqtt_enabled){
    while (!client.connected()) {
      reconnectMQTT();
    }
  }
      
  // Only in Power Supply Mode
  if (powermode) {
    DEBUG_PRINTLN("Startup completed in Power Supply mode ...");
    measureFreq();                                                      // Start first measure immediately
    startMeasure.attach(power_sampling_interval*60, set_measure_flag);  // Start measure periodically each configured interval time (set the flag)
  } 
  else {
    DEBUG_PRINTLN("Startup completed in Battery mode ...");
    measureFreq();                                                    // Start measure and send via MQTT if enabled
    delay(5000);                                                      // Needed to do all MQTT tasks
    DEBUG_PRINTLN("Going into deepsleep mode ...");
    ESP.deepSleep(60000000*bat_sampling_interval);                    // 60s * Value in Minutes from config
    delay(100);                                                       // Only to ensure the Deepsleep is working
  }
}


// **************************************************************************
//  Main Loop
// **************************************************************************
void loop(void){
  // Only in Power Supply Mode
  if (powermode) { 
    httpServer.handleClient();
  
    if (mqtt_enabled){
      if (!client.connected()) {
        reconnectMQTT();
      }
      client.loop();
    }

    if (func_measure_call) {            // If the call function flag is set by ticker, execute measureFreq()
      measureFreq();
      func_measure_call = false;
    }
    
  }
}

