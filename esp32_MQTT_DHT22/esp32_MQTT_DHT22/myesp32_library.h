#ifndef MY_LIBRARY_H
#define MY_LIBRARY_H
#endif

#include <Arduino.h>

int addTwoInts(int a, int b);

void esp32StartOLED(int oledSDA, int oledSCL, int oledLibrary);
void displayTextOLED(String oledline[9], int oledLibrary);

void esp32StartOLED_Ascii(int oledSDA, int oledSCL);
void displayTextOLED_Ascii(String oledline[9]);
void esp32StartOLED_Adafruit(int oledSDA, int oledSCL);
void displayTextOLED_Adafruit(String oledline[9]);

void convHHMMSS(unsigned long currSeconds, char *uptimeHHMMSS);
void convDDHHMMSS(unsigned long currSeconds, char *uptimeDDHHMMSS);

void getWiFiRSSI(char *wifiRSSI);
void getMacWifiShield(char *macWifiShield);
void getMacRouterESP32(char *macRouter);
void getMacWifiShieldMacRouterSS(char *macCombined);
