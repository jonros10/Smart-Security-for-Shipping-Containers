// This example uses an ESP32 C3 DevKitM1 together with a WiFi to connect to MQTT
//
// For MQTTX software use Host name "public.cloud.shiftr.io", username: "public" and pasword: "public"
// mqttClient_1.connect(mqttClienName, "public", "public")
/**************************************************************************
 Class: ECE508 Summer 2023
 Student Names: Jonathan Rosales, Ryan Ahmed, Monsif Bouzana
 Date: 06/27/2023
 Team07 
 Description: This program uses MQTT to send readings of temperature and humidity to the MQTT Broker
 Issues: No issues.
  **************************************************************************/
#include <stdio.h>
#include "myesp32_library.h"
#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include <Arduino_JSON.h>

//*************************************************************
//IMPORTANT: Student shall update these three entries before uploading the code
//*************************************************************
const char gNumber[15] = "Team_07";  //Update with the last digits of your G-number
const char* ssid = "Jon";     //Update with your WiFi SSID
const char* pass = "opensesame";     //Update with your WiFi password
//*************************************************************

unsigned long currMillis, prevMillis;
char tmpBuffer1[64];
char tmpBuffer2[64];
char lcdBuffer[64];
String oledline[9];
const int oledLib = 1;
int ledStatus = 0;

#include <SimpleDHT.h>
int pinDHT22 = 3;
SimpleDHT22 dht22(pinDHT22);
float temperature = 0;
float humidity = 0;
int errDHT22 = SimpleDHTErrSuccess;

WiFiClient wifiClient_1;
MqttClient mqttClient_1(wifiClient_1);

//const char mqttBroker_1[63] = "test.mosquitto.org";
const char mqttBroker_1[63] = "public.cloud.shiftr.io";

int myRand;
// The program will replace "_1234_" entry with random numbers, the G muber will be replace with gNumber entry
char mqttClienName[31] = "client_1234_Gxxxx8279";
int intervalMQTT = 0;
long nmrMqttMesages = 0;
String mqttStringMessage;
String mqttStringMessagehumid;
char topicPub1[61]  = "";
char topicPub2[61]  = "";
char topicSubControl[61] = "";
//char topicSubHeartbeat[61]  = "";

JSONVar sensorObj;
int analogPin = 4;

void setup() {
  pinMode(RGB_BUILTIN, OUTPUT);
  Serial.begin(115200); delay(500);
  digitalWrite(RGB_BUILTIN, HIGH);
  pinMode(analogPin, INPUT);
  
  randomSeed(analogRead(analogPin));
  strcpy(topicPub1,"gmu/ece508/Team_07/esp32c3");

  //Generate Random MQTT ClientID
  myRand = random(0, 9999);
  sprintf(mqttClienName, "client_%04d_%s", myRand, gNumber);
  
  esp32StartOLED(4, 5, oledLib);
  oledline[1] = String(gNumber) + " MQTT";
  for (int jj=2; jj<=8; jj++){ oledline[jj]=""; }
  
  WiFi.begin(ssid, pass);
  oledline[2] = "Connecting to Wifi..."; displayTextOLED(oledline, oledLib);
  prevMillis = millis();
  while (WiFi.status() != WL_CONNECTED) {    
    Serial.println("Attempting to connect to WiFi...");   
    delay(250);
  }
  currMillis = millis();
  sprintf(lcdBuffer, "Connected in %d sec", (currMillis-prevMillis)/1000);
  oledline[2] = lcdBuffer; displayTextOLED(oledline, oledLib);
  delay(250);

  prevMillis = millis();
}

void loop() {
  mqttClient_1.poll();

  if (!mqttClient_1.connected()) {
    mqttClient_1.setUsernamePassword("public", "public");
    for (int jj=2; jj<=8; jj++){ oledline[jj]=""; }
    oledline[2] = String("Connecting to MQTT_1"); displayTextOLED(oledline, oledLib);
    while (!mqttClient_1.connect(mqttBroker_1, 1883)) {
       delay(500);
    }
    mqttClient_1.onMessage(onMqttMessage_1);
    mqttClient_1.subscribe("ece508/Gxxxxxxxx/ctrl");   
  }

  currMillis = millis();
  // publish a message roughly every second.
  if (currMillis - prevMillis > 1000) {
    prevMillis = currMillis;
    digitalWrite(RGB_BUILTIN, (ledStatus++)%2);
    
    oledline[2] = String(mqttBroker_1, 21);
    oledline[3] = String(mqttClienName, 21);
    errDHT22 = dht22.read2(&temperature, &humidity, NULL);
      if (errDHT22 != 0) {
        temperature = (-1 - 32)/1.8;
        humidity = 57;
      }
 
    intervalMQTT++;
    if (intervalMQTT >= 3) {   
      intervalMQTT = 0;
      nmrMqttMesages++;
      oledline[6] = String("Send message: ") + String(nmrMqttMesages);
      //myJsonDoc["board"] = "ESP32 C3 DevKitM1";
      //myJsonDoc["msgnum"] = nmrMqttMesages;
      
      getMacWifiShieldMacRouterSS(lcdBuffer);
      //myJsonDoc["mac"] = String(lcdBuffer);

      convDDHHMMSS(currMillis/1000, lcdBuffer);
      //myJsonDoc["uptime"] = String(lcdBuffer);
      //myJsonDoc["analogPin"] = analogRead(analogPin);
      temperature = 1.8*temperature + 32;
      sensorObj["sensor"]["dht22"]["tempF"] = (int)temperature;
      sprintf(tmpBuffer1, "Temp in F:%.2f", (int)temperature);

      sensorObj["sensor"]["dht22"]["humRH"] = (int)humidity;
      sprintf(tmpBuffer1, "Humidity in RH%%:%.2f", (int)humidity);
      
      mqttStringMessage = JSON.stringify(sensorObj);
      //mqttStringMessagehumid = tmpBuffer2;

      //mqttStringMessage = JSON.stringify(myJsonDoc);
      //Serial.print("Publishing..."); Serial.println(mqttStringMessage);

      mqttClient_1.beginMessage(topicPub1); mqttClient_1.print(mqttStringMessage); mqttClient_1.endMessage();
    }
          
    convDDHHMMSS(currMillis/1000, lcdBuffer);
    oledline[8] = "Uptime: " + String(lcdBuffer);
    displayTextOLED(oledline, oledLib);
  }
}

void onMqttMessage_1(int messageSize) {
  // we received a message, print out the topic and contents
  Serial.println("Received a message with topic '");
  Serial.print(mqttClient_1.messageTopic());
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  // use the Stream interface to print the contents
  while (mqttClient_1.available()) {
    Serial.print((char)mqttClient_1.read());
  }
  Serial.println();

  Serial.println();
}
