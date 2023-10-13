/**************************************************************************
 Class: ECE508 Summer 2023
 Student Name: Jonathan Rosales, Ryan Ahmed, Monsif Bouzana 
 
 Date: 06/27/2023
 FINAL
 Description: This program runs on nano33 connects to wifi and sends data collected from
              DHT, GPS, PIR senseor and MQTT broker publishes on a specified topic to shiftr.io
 Issues: all runs smooth no issues.
  **************************************************************************/
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <stdio.h>
#include "myiot33_library.h"
#include <WiFiNINA.h>
//MQTT library for Arduino by Joel Gaehwiler
//Arduino Nano 33 IoT library in https://github.com/256dpi/arduino-mqtt
//Download "arduino-mqtt-master.zip" and install using Add .ZIP library
#include <MQTT.h>
#include <Arduino_JSON.h>

#include "modTinyGPSpp.h"
TinyGPSPlus gps;

#include "DHT.h"
int pinDHT22 = 2;
DHT dht(pinDHT22, DHT22);
char tempBuff[32];
char humidBuff[32];

int inputPin = 5;
int valPIR = 0;
String detect = "";
// change the input pins based on wiring configuration.

#include <Arduino_LSM6DS3.h>
float aX, aY, aZ;
const char * spacer = ", ";


//*************************************************************
//*************************************************************
//IMPORTANT: Student shall update these three entries before uploading the code
//*************************************************************
const char gNumber[15] = "Team_07";  //Update with your G-Number
const char ssid[31] = "Ryan's iPhone";     //Update with your WiFi SSID
const char pass[31] = "";     //Update with your WiFi password
//const char ssid[31] = "TP-Link_77BA";     //Update with your WiFi SSID
//const char pass[31] = "";     //Update with your WiFi password
//*************************************************************
//*************************************************************
//*************************************************************

unsigned long currMillis, prevMillis;
char tmpBuffer[64];
String oledline[9];
const int oledLib = 1;

WiFiUDP ntpUDP;
WiFiClient wifiClient_1;
WiFiClient wifiClient_2;
MQTTClient mqttClient_1;
MQTTClient mqttClient_2;
//const char mqttBroker[63] = "127.0.0.1:1883"; //Update with your computer IP. Use ipconfig in CMD window
//const char mqttBroker[63] = "192.168.1.189"; //Update with your computer IP. Use ipconfig in CMD window
const char mqttBroker_1[63] = "test.mosquitto.org"; // first broker
const char mqttBroker_2[63] = "public.cloud.shiftr.io"; // second broker

//NTPClient ntpClient(ntpUDP, "id.pool.ntp.org", 0, 60000);
//NTPClient ntpClient(ntpUDP, "time-a-g.nist.gov", 0, 60000);
//NTPClient ntpClient(ntpUDP, "time.windows.com", 0, 60000);
//NTPClient ntpClient(ntpUDP, "time.google.com", 0, 60000);
NTPClient ntpClient(ntpUDP);

int myRand;
//char mqttClienName[31] = "client_59999_G12345678";
char mqttClienName[31] = "TEAM_07";
int intervalMQTT = 0;
long nmrMqttMessages = 0;
String mqttStringMessage;

char topicPub_1[61]  = "";
char topicPub_2[61]  = "";

char mac_add[50];
long epochTime = 0;
long epochTimeNTP = 0;

JSONVar sensorObj;

void setup() {
  Serial.begin(9600); delay(500);
  Serial1.begin(9600); delay(500);
  randomSeed(analogRead(A7));
//  sprintf(topicPub, "%s%s%s", "gmu/ece508/", gNumber, "/nano33iot");  //ece508/Gxxxx5678/nano33iot
  strcpy(topicPub_1,"gmu/ece508/Team_07/nano33iot");
  strcpy(topicPub_2,"gmu/ece508/Team_07/nano33iot");
  
  //Generate Random MQTT ClientID
  myRand = random(0, 9999);
  sprintf(mqttClienName, "client_%04d_%s", myRand, gNumber);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  dht.begin();
  
  iot33StartOLED(oledLib);
  oledline[1] = String(gNumber) + " Mosqto_Shiftr";
  for (int jj=2; jj<=8; jj++){ oledline[jj]=""; }
  
  // attempting to connect to Wifi network:  
  oledline[2] = "Connecting to Wifi..."; displayTextOLED(oledline, oledLib);
  prevMillis = millis();
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {    
    status = WiFi.begin(ssid, pass); delay(250);
  }
  sprintf(tmpBuffer, "Connected in %d sec", (millis()-prevMillis)/1000);
  oledline[2] = tmpBuffer; displayTextOLED(oledline, oledLib); delay(500);

  ntpClient.begin();

  mqttClient_1.begin(mqttBroker_1, wifiClient_1);
  mqttClient_1.onMessage(messageReceived);

  mqttClient_2.begin(mqttBroker_2, wifiClient_2);
  mqttClient_2.onMessage(messageReceived);

  oledline[2] = "Synchronizing NTP..."; displayTextOLED(oledline, oledLib); delay(500);
  Serial.print("Time Synchronization NTP");
  epochTimeNTP = ntpClient.getEpochTime();
  while (epochTimeNTP > 1680300000) {    
    epochTimeNTP = ntpClient.getEpochTime();    
    Serial.print(".");
    delay(2000);
  }  
  Serial.println(ntpClient.getEpochTime());
  oledline[2] = "NTP Synchronized"; displayTextOLED(oledline, oledLib); delay(500);
  
  /*
  Serial.print("Time Synchronization...");
  epochTime = WiFi.getTime();
  while (epochTime == 0) {    
    epochTime = WiFi.getTime();    
    Serial.print(".");
    delay(2000);
  }
  Serial.println("\n");
  */
  
  prevMillis = millis();
}

void loop() {
  
  mqttClient_1.loop();
  mqttClient_2.loop();

  if ((!mqttClient_1.connected())&&(!mqttClient_2.connected())) {
//  if (!mqttClient_1.connected()) {
    for (int jj=2; jj<=8; jj++){ oledline[jj]=""; }
    oledline[2] = String("Connecting to MQTT..."); displayTextOLED(oledline, oledLib);
    connectMqtt_1(mqttClienName);
    connectMqtt_2(mqttClienName);
  }

//  if (!mqttClient_2.connected()) {
//    for (int jj=2; jj<=8; jj++){ oledline[jj]=""; }
//    oledline[2] = String("Connecting to MQTT..."); displayTextOLED(oledline, oledLib);
//    connectMqtt_1(mqttClienName);
//    connectMqtt_2(mqttClienName);
//  }

  currMillis = millis();
  // publish a message roughly every second.
  if (currMillis - prevMillis > 1000) {
    ntpClient.update();
    prevMillis = currMillis;

    float myHumid = 0.0;
    float myTemp = 0.0;
    delay(200);
    myHumid = dht.readHumidity();
    myTemp = dht.readTemperature(true);
    sprintf(tempBuff, "%.2f F",myTemp);
    sprintf(humidBuff, "%.2f RH%%",myHumid);
    

    valPIR = digitalRead(inputPin);
    if (valPIR == HIGH) {
      detect = "**YES**";
    } else {
      detect = "--no--";
    }
    delay(200);

    if (IMU.accelerationAvailable()) {      
      IMU.readAcceleration(aX, aY, aZ);
      Serial.print(aX); Serial.print(spacer);
      Serial.print(aY); Serial.print(spacer);
      Serial.println(aZ);
    }

    oledline[2] = String(mqttBroker_1, 21);
    oledline[3] = String(mqttBroker_2, 21);
    oledline[5] = String(mqttClienName, 21);
    
    intervalMQTT++;
    if (intervalMQTT >= 3) {   
      intervalMQTT = 0;
      nmrMqttMessages++;
      oledline[4] = String("Send message: ") + String(nmrMqttMessages);
      
      //getMacWifiShieldMacRouterSS(tmpBuffer);
      smartDelay(3000);
      double latRnd = (double) random(0, 1000)/1000;
      double lonRnd = (double) random(0, 1000)/1000;
      sprintf(tmpBuffer, "%04d-%02d-%02d %02d:%02d:%02d", gps.date.year(), gps.date.month(), gps.date.day(), 
                                                      gps.time.hour(), gps.time.minute(), gps.time.second());
      
      //mqttStringMessage = "airSensors,sensor_id=TLM0100 temperature=71.15875225821217,humidity=35.12865369058315,co=0.5169902572341222 1680040153000000000";
      //epochTime = WiFi.getTime();
      epochTimeNTP = ntpClient.getEpochTime();
      
      
      sensorObj["name"] = "ece508_Team_07";
      sensorObj["nmrMqttMessages"] = nmrMqttMessages;
      sensorObj["rssi"] = WiFi.RSSI();
      sensorObj["epoch"] = epochTimeNTP;

      //sensorObj["gps"]["locvalid"] = gps.location.isValid();
      sensorObj["gps"]["locate"]["lat"] = String(gps.location.lat()+latRnd,5);
      sensorObj["gps"]["locate"]["lon"] = String(gps.location.lng()+lonRnd,5);
      //sensorObj["gps"]["alt"] = gps.altitude.isValid();
      sensorObj["gps"]["alt_ft"] = String(gps.altitude.feet(),1);
      //sensorObj["gps"]["speedvalid"] = gps.speed.isValid();
      sensorObj["gps"]["speedmph"] = String(gps.speed.kmph(),2);
      //sensorObj["gps"]["hdopvalid"] = gps.hdop.isValid();
      sensorObj["gps"]["hdop"] = String((float)gps.hdop.value()/100,2);
      //sensorObj["gps"]["satvalid"] = gps.satellites.isValid();
      sensorObj["gps"]["sat"] = gps.satellites.value();

      sensorObj["DHT"]["tempF"] = tempBuff;
      sensorObj["DHT"]["humidity"] = humidBuff;

      sensorObj["PIR"] = detect;

      sensorObj["IMU_acc"] = String(aX) + ", " +
                             String(aY) + ", " +
                             String(aZ);
      
      
            
      //mqttStringMessage = String("airSensor,device=nano33iot") + " " +
      //                           "nmrMqttMessages=" + String(nmrMqttMessages) + "," +
      //                           "rssi=" + WiFi.RSSI()  + " " +
      //                    String(epochTimeNTP) + "000000000";

      mqttStringMessage = JSON.stringify(sensorObj);
      
      mqttClient_1.publish(topicPub_1, mqttStringMessage);
      mqttClient_2.publish(topicPub_2, mqttStringMessage);
      Serial.println("Topic:" + String(topicPub_1) + " Message:" + String(mqttStringMessage));
      Serial.println("Topic:" + String(topicPub_2) + " Message:" + String(mqttStringMessage));
    }

    epochTimeNTP = ntpClient.getEpochTime();
    convCurrentTimeET_DST(epochTimeNTP, tmpBuffer, 1);
    oledline[7] = String(tmpBuffer);          
    
    convDDHHMMSS(currMillis/1000, tmpBuffer);
    oledline[8] = "Uptime: " + String(tmpBuffer);
    displayTextOLED(oledline, oledLib);
  }
}

void connectMqtt_1(char *mqttClienName) 
{
  Serial.println("connectMqtt: Checking WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("."); delay(1000);
  }
  Serial.println("connectMqtt: WiFi Ok...");
 
  Serial.println("connectMqtt: Checking MQTT...");
//  while (!mqttClient.connect(mqttClienName, "public", "public")) {
  while (!mqttClient_1.connect(mqttClienName)) {
    Serial.print("."); delay(1000);
  }
  Serial.println("connectMqtt: MQTT Ok...");
 
  //sprintf(topicSubHeartbeat, "%s%s", gNumber, "/heartbeat");
  //mqttClient.subscribe(topicSubHeartbeat);
  //Serial.println("Subscribe: " + String(topicSub));
  //mqttClient.subscribe("ece508/hello");
}

void connectMqtt_2(char *mqttClienName) 
{
  Serial.println("connectMqtt: Checking WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("."); delay(1000);
  }
  Serial.println("connectMqtt: WiFi Ok...");
 
  Serial.println("connectMqtt: Checking MQTT...");
  while (!mqttClient_2.connect(mqttClienName, "public", "public")) {
//  while (!mqttClient_1.connect(mqttClienName)) {
    Serial.print("."); delay(1000);
  }
  Serial.println("connectMqtt: MQTT Ok...");
 
  //sprintf(topicSubHeartbeat, "%s%s", gNumber, "/heartbeat");
  //mqttClient.subscribe(topicSubHeartbeat);
  //Serial.println("Subscribe: " + String(topicSub));
  //mqttClient.subscribe("ece508/hello");
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.
}

/********************************************************************
*******************************************************************
additional libraires included
********************************************************************/

// This custom version of delay() ensures that the gps object is being "fed".
// from the nano33_tinyGPS_JSON.
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}
