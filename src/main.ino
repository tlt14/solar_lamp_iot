#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <TimeLord.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <TimeLib.h>
#include <WidgetRTC.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include "./lib/ota.h"

#define vPIN_LED1_EN    V1
#define vPIN_LED1_PWM   V2
#define vPIN_LED2_EN    V3
#define vPIN_LED2_PWM   V4
//debug virtual pin
#define vPIN_SETTIME    V5
#define vPIN_DEBUG      V6
#define vPIN_TEMP       V7
#define vPIN_HUMI       V8
#define vPIN_PRES       V9

#define vPIN_CURRENT_TIME V12
#define vPIN_START_TIME   V13
#define vPIN_STOP_TIME    V14
//define led pins
#define LED1_PWM        15
#define LED1_EN         13
#define LED2_PWM        14
#define LED2_EN         12

long currentTime;
long startTime;
long stopTime;

bool hasStart = false;
bool hasStop = false;
bool deBug = false;
bool firstBoot = true;
//Google map
float const LONGITUDE = 107.2869734; // FIXME: -180 -> 180 
float const LATITUDE = 11.8530421; // FIXME: -90 -> 90

char auth[] = "uLNTTGmLV9fDOHI6jNEJ_7Py2NQhli8-";
// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Tan Tien";
char pass[] = "123456789";
char server[] = "192.168.1.10";
uint16_t port = 8080;

BlynkTimer timer;
WidgetRTC rtc;
WidgetBridge brd1(V15);
TimeLord tardis; 
Adafruit_BME280 bme; // I2C

long getSun(bool isRise)
{
    
    long sunTime = 0;
    byte yearYY = (byte) (year()%100);
    byte today[] = {0, 0, 12, (byte)day(), (byte)month(),yearYY}; // store today's date (at noon) in an array for TimeLord to use
    if(isRise){
    if (tardis.SunRise(today)) // if the sun will rise today (it might not, in the [ant]arctic)
        sunTime =  (today[tl_hour] * 3600) + (today[tl_minute] * 60);
    else  
        sunTime = -1; 
    }
    else{
        if (tardis.SunSet(today)) // if the sun will set today (it might not, in the [ant]arctic)
            sunTime = (today[tl_hour] * 3600) + (today[tl_minute] *60);
        else 
            sunTime = -1;
    }
    return sunTime;
}
void setLedState(bool state)
{
    Blynk.virtualWrite(vPIN_LED1_EN, state);
    Blynk.virtualWrite(vPIN_LED2_EN, state);
    digitalWrite(LED1_EN, state);
    digitalWrite(LED2_EN, state);
    brd1.virtualWrite(V55, !state);
}
void timeChecker()
{
  currentTime = hour() * 3600 + minute() * 60 + second() ;
  if(hour() == 0 && minute() == 0 && second() < 2 ){
    if(!hasStart){
        startTime = getSun(false);
    }
    if(!hasStop){
        stopTime = getSun(true);
    }
  }
  if(deBug == 1){
        Blynk.virtualWrite(vPIN_CURRENT_TIME, currentTime);
        Blynk.virtualWrite(vPIN_START_TIME, startTime);
        Blynk.virtualWrite(vPIN_STOP_TIME, stopTime);   
  }
  if((currentTime >= startTime || (currentTime <= stopTime && firstBoot)) && (currentTime <= startTime + 1 || firstBoot)){
    setLedState(true);
    if(firstBoot)
        firstBoot = false;
    }
    if(currentTime >= stopTime&&(currentTime <= stopTime + 1 || firstBoot)) {
        setLedState(false);
    if(firstBoot)
        firstBoot = false;
}
}
BLYNK_CONNECTED()
{
    // Request the latest state from the server
    rtc.begin();
    Blynk.syncAll();
    brd1.setAuthToken("xDUrmGQpaIzHvzrv5ZpLoDeF0vcjLZoZ");
}
BLYNK_WRITE_DEFAULT(){
  int vPin = request.pin;
  int value = param.asInt();
  switch(vPin){
    case vPIN_LED1_PWM:
        analogWrite(LED1_PWM, value);
    break;
    case vPIN_LED1_EN:
        digitalWrite(LED1_EN, value);
    break;
    case vPIN_LED2_PWM:
        analogWrite(LED2_PWM, value);
    break;
    case vPIN_LED2_EN:
        digitalWrite(LED2_EN, value);
    break;
    case vPIN_DEBUG:
        deBug = value;
    break;
    }
}
BLYNK_WRITE(vPIN_SETTIME)
{
  TimeInputParam t(param);
    if(t.hasStartTime())
    {
      startTime = (t.getStartHour() * 3600) + (t.getStartMinute() * 60);
      hasStart = true;
    }
    else{
      startTime = getSun(false);
      hasStart = false;
    }
    if (t.hasStopTime())
    {
      stopTime = (t.getStopHour() * 3600) + (t.getStopMinute() * 60); 
      hasStop = true;
    }
    else{
      stopTime = getSun(true);
      hasStop = false;
    }
}
void sendSensor()
{
    Blynk.virtualWrite(vPIN_TEMP, bme.readTemperature());
    Blynk.virtualWrite(vPIN_HUMI, bme.readHumidity());
    Blynk.virtualWrite(vPIN_PRES, bme.readPressure()/100.0f);
}
void setup()
{
    pinMode(LED1_EN, OUTPUT);
    pinMode(LED2_EN, OUTPUT);
    pinMode(LED1_PWM,OUTPUT);
    pinMode(LED2_PWM, OUTPUT);
    analogWriteFreq(1000);
    analogWriteRange(255);
    tardis.TimeZone(7 * 60);
    tardis.Position(LATITUDE, LONGITUDE); // tell TimeLord where in the world we are
    bme.begin(0x76); 
    Blynk.begin(auth, ssid, pass, server, port);
    arduinoOtaSetup();
    setSyncInterval(60); // Sync interval in seconds (1 minutes)
    
    timer.setInterval(1000L, timeChecker);
    timer.setInterval(1000L, sendSensor);
}
void loop()
{
  Blynk.run();
  timer.run();
  ArduinoOTA.handle();
}