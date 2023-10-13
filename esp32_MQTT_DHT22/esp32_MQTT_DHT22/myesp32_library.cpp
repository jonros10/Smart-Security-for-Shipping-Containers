#include <time.h>
#include <WiFi.h>
#include "myesp32_library.h"
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;
Adafruit_SSD1306 myOled(128, 64, &Wire, -1);

void esp32StartOLED(int oledSDA, int oledSCL, int oledLibrary)
{
  if (oledLibrary == 1){
    esp32StartOLED_Adafruit(oledSDA, oledSCL);
  } else {
    esp32StartOLED_Ascii(oledSDA, oledSCL);
  }
}

void displayTextOLED(String oledline[9], int oledLibrary)
{
  if (oledLibrary == 1){
    displayTextOLED_Adafruit(oledline);
  } else {
    displayTextOLED_Ascii(oledline);
  }
}

void esp32StartOLED_Ascii(int oledSDA, int oledSCL)
{
  Wire.begin(oledSDA, oledSCL); //ESP32
  oled.begin(&Adafruit128x64, I2C_ADDRESS, -1);
  oled.setFont(Adafruit5x7);
}

void displayTextOLED_Ascii(String oledline[9])
{
  oled.clear();
  oled.set1X();
  for (int jj = 1; jj <= 8; jj++) {  
     oled.println(oledline[jj]);
  }
}

void esp32StartOLED_Adafruit(int oledSDA, int oledSCL)
{
  Wire.begin(oledSDA, oledSCL); //ESP32
  if(!myOled.begin(SSD1306_SWITCHCAPVCC, I2C_ADDRESS)) { 
    Serial.println("SSD1306 allocation failed");
  }
}

void displayTextOLED_Adafruit(String oledline[9])
{
  int jj;
  
  myOled.clearDisplay();
  myOled.setTextSize(1);
  myOled.setTextColor(SSD1306_WHITE);
  myOled.setCursor(0, 0);

  for (jj=1; jj<=8; jj++) { 
    myOled.println(oledline[jj]);
    }
  
  myOled.display(); 
}

void convHHMMSS(unsigned long currSeconds, char *uptimeHHMMSS)
{
  int hh, mm, ss;

  hh = (currSeconds/3600);
  mm = (currSeconds -(3600*hh))/60;
  ss = (currSeconds -(3600*hh)-(mm*60));
  sprintf(uptimeHHMMSS, "Time: %02d:%02d:%02d", hh, mm, ss);

};

 
void convDDHHMMSS(unsigned long currSeconds, char *uptimeDDHHMMSS)
{
  int dd, hh, mm, ss;

  ss = currSeconds; //258320.0 2 23:45:20
  dd = (ss/86400);
  hh = (ss-(86400*dd))/3600; 
  mm = (ss-(86400*dd)-(3600*hh))/60;
  ss = (ss-(86400*dd)-(3600*hh)-(60*mm));

  sprintf(uptimeDDHHMMSS, "%d %02d:%02d:%02d", dd, hh ,mm, ss);
};

void getWiFiRSSI(char *wifiRSSI) 
{
    sprintf(wifiRSSI, "%lddBm %d.%d.%d.%d", WiFi.RSSI(), WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
};

void getMacWifiShield(char *macWifiShield)
{
    byte mac[6];

    WiFi.macAddress(mac);
    sprintf(macWifiShield, "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
};

void getMacRouterESP32(char *macRouter)
{
    sprintf(macRouter, "%02X:%02X:%02X:%02X:%02X:%02X", WiFi.BSSID()[0], WiFi.BSSID()[1], WiFi.BSSID()[2], WiFi.BSSID()[3], WiFi.BSSID()[4], WiFi.BSSID()[5]);
};

void getMacWifiShieldMacRouterSS(char *macCombined)
{
    byte macWifiShield[6];
    long currMillis;

    currMillis = millis();
    WiFi.macAddress(macWifiShield);
    sprintf(macCombined, "%02X:%02X:%02X:%02X:%02X:%02d", macWifiShield[0], WiFi.BSSID()[4], macWifiShield[1], WiFi.BSSID()[2], macWifiShield[2], (currMillis/1000) % 60);

};
