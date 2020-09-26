#include <Wire.h> // I2C library
#include <Adafruit_Sensor.h> //BNO055 9DOF Sensor library dependency
#include <Adafruit_BNO055.h> //BNO055 9DOF Sensor library
#include <utility/imumaths.h>
#include <neomatrix.h> // Adafruit Neopixel RGB board display word library
#include <neopixel.h> // Adafruit Neopixel RGB board library
#include "ntp-time.h" // Current UTC time library
#include <blynk.h> // BLYNK library
#include "SparkCorePolledTimer.h" // Timer library
#include <google-maps-device-locator.h>
#include "Particle.h"
#include "application.h"
#include <JsonParserGeneratorRK.h>
#include "WatchDog_WCL.h"
#define PIXEL_PIN A0             // Set up for Neopixel RGB board side 1 Analog Pin
#define PIXEL_PIN_1 A1           // Set up for Neopixel RGB board side 2 Analog Pin
#define PIXEL_COUNT 128          // Neopixel RGB board total pixel count
#define PIXEL_TYPE WS2812B       // Neopixel RGB board total pixel type
SYSTEM_THREAD(ENABLED);         // Enable Threads RTOS
Thread main;    // Main Thread
Thread weather;  // Thread to read weather from the web constantly
int currentTimeZone = 5;     // Timezone initialize can change with Blynk
NtpTime* ntpTime;       //ntp start
float x,y,z,ori;        //9DOF ints
String timeNow;          // Current time string
int timerTime;   // Timer ints from here
int timerDone, timerOn;
int minutes, seconds, stopWatchOn;  // to here
int brightness = 30;    // Display ints from here
int r = 255;
int g =255;
int b = 255;
float secondsdisplay;   // to here
unsigned int nextTime = 30;    // Next time to contact the server
String tempReceived = "";
String currentStatus;
String precipProb;
JsonParserStatic<512, 50> jsonParser;
String global_lat = ""; //variables for location data
String global_lon = "";
String global_ip = ""; //through here
SparkCorePolledTimer updateTimer(timerTime);    //initialize timer
Adafruit_BNO055 bno = Adafruit_BNO055(55);      //initialize 9DOF
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(16, 8, PIXEL_PIN,    //initialize Screen one
  NEO_MATRIX_TOP     + NEO_MATRIX_RIGHT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE,
  PIXEL_TYPE);
Adafruit_NeoMatrix matrix_1 = Adafruit_NeoMatrix(16, 8, PIXEL_PIN_1,    //initialize Screen two
  NEO_MATRIX_BOTTOM     + NEO_MATRIX_LEFT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE,
  PIXEL_TYPE);
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN_1, PIXEL_TYPE);    //initialize one of the screen for rainbow lights
int x1 = matrix.width();    //int for text scrolling on screens
int x2 = matrix_1.width();
static unsigned long waitMillis;    //for rainbow lights from here
struct epochMillis now;
void rainbow(uint8_t wait);
uint32_t Wheel(byte WheelPos);    //to here
WatchDog wd;


String hhmmss(unsigned long int now, int timeZone)  //format value as "hh:mm:ss"
{
    String hour;
    if(Time.hour(now)-timeZone<0){    // if the hour minus time difference is negative
        hour = String::format("%02i",Time.hour(now)-timeZone+24);   //add 24
    } else if(Time.hour(now)-timeZone>24) {   //if the hour minues time difference is over 24
        hour = String::format("%02i",Time.hour(now)-timeZone-24); //minus 24
    } else {
        hour = String::format("%02i",Time.hour(now)-timeZone);    //else normal
    }
    String minute = String::format("%02i",Time.minute(now));    //include minutes
    return hour + ":" + minute;     //returns value of hour and minute
}

void setup(void)
{
  Serial.begin(9600);     //begin Serial port for debugging
  if(!bno.begin())        //send error to Serial if 9DOF is not initializing
  {
    while(1);
    Serial.println("not working");
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  Serial.println("complete");
  matrix.begin();       //matrix(screen) setup from here
  matrix.setTextWrap(false);
  matrix.setBrightness(30);
  matrix.setTextColor(matrix.Color(80,255,0));
  matrix_1.begin();
  matrix_1.setTextWrap(false);
  matrix_1.setBrightness(30);
  matrix_1.setTextColor(matrix.Color(80,255,0));      //to here
  updateTimer.SetCallback(OnTimer);     //when timer off go to function OnTimer
  Blynk.begin("8d7cb678e67a48d49e443ce542cbf215",IPAddress(167,99,150,124),8080);     //connect to Blynk
  Particle.subscribe("particle/device/ip", ipHandler);  //subscribes to global IP get
  Particle.subscribe("hook-response/geoip", geoIpHandler, MY_DEVICES);    //subscribes to geoip get
  Particle.subscribe("hook-response/tempRead", tempHandler, MY_DEVICES);    //subscribes to weather get
  weather = Thread("name", weatherFunction);    //sets up Threads
  main = Thread("nubeds", mainFunction);
  pinMode(A5, OUTPUT); 
  pinMode(D3, OUTPUT);
  strip.begin();    //neopixel start for rainbow function
  strip.show();
  wd.initialize(15000);    //watchdog setup for 5 seconds
  wd.pet();   //pet watchdog
}

void rainbow(uint8_t wait) {    //rainbow function from the examples in neopixel
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

uint32_t Wheel(byte WheelPos) {   //rainbow function from the examples in neopixel
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

void weatherFunction(){   //gather weather information
  while(1){
  // Step 1: get IP
  Particle.publish("particle/device/ip"); //requests public ip from particle
  delay(4000);
  // Step 2: get geo location
  Particle.publish("geoip", global_ip.c_str(), PRIVATE); //requests data from particle and translates it into a char*
  delay(4000);
  // Step 3: get temp
  char data[64]; //data buffer for integrating lat and lon for webhook
  sprintf(data, "%s,%s", global_lat.c_str(), global_lon.c_str());//move to geoip handler so is step by step
  Particle.publish("tempRead", data, PRIVATE);//                    that way it is event driven
  delay(10000);
  }
}

void tempHandler(const char *event, const char *data) { //callback for temp webhook
  // Handle the integration response
  jsonParser.addString(data); //adds json to buffer
  if (jsonParser.parse()) {
    jsonParser.getOuterValueByKey("tempReceived", tempReceived); //parses
    jsonParser.getOuterValueByKey("currentStatus", currentStatus);
    jsonParser.getOuterValueByKey("percipProb", precipProb);
    // Put code to do something with tempMin and tempMax here
    Serial.printlnf("tempReceived=%s, currentStatus=%s, precipitationProbability:%s", tempReceived.c_str(), currentStatus.c_str(), precipProb.c_str()); //translates String to char* 
  }
  jsonParser.clear();//clears buffer so that geographical data can be parsed correctly
}

void geoIpHandler(const char *event, const char *data) { //callback for geoip webhook
  jsonParser.addString(data); //this input uses a response template on the particle console to simplify the response
  if (jsonParser.parse()) {
    jsonParser.getOuterValueByKey("latitude", global_lat);
    jsonParser.getOuterValueByKey("longitude", global_lon);
    Serial.printlnf("lat=%s,lon=%s", global_lat.c_str(), global_lon.c_str());
  }
  jsonParser.clear(); //clears the parser buffer so the temp can be parsed correctly
}

void ipHandler(const char *event, const char *data) {
  global_ip = data; //sets ip
  Serial.printlnf("global_ip=%s", global_ip.c_str()); //prints ip
}

BLYNK_WRITE(V0){    //blynk set timerTime slot
  timerTime = param.asInt() * 1000;
  Serial.println(timerTime);
}

BLYNK_WRITE(V1){    //blynk button timer start detection
  Serial.println(param.asInt());
  if(param.asInt()==1){
    timerOn = 1;
  } else {
    timerOn = 0;
  }
}

BLYNK_WRITE(V2){    //blynk button stopwatch start detection
  Serial.println(param.asInt());
  if(param.asInt()==1){
    stopWatchOn = 1;
  } else {
    stopWatchOn = 0;
  }
}

BLYNK_WRITE(V3){    //blynk set brightness slider
  brightness = param.asInt();
}

BLYNK_WRITE(V4){    //blynk zeRGBra color customization
    r = param[0].asInt();
    g = param[1].asInt();
    b = param[2].asInt();
}

BLYNK_WRITE(V6){    //blynk timeZone selection
  switch(param.asInt()){
    case 1:{
      currentTimeZone = 7;
      break;  
    }
    case 2:{
      currentTimeZone = 6;
      break;
    }
    case 3:{
      currentTimeZone = 5;
      break;
    }
    case 4:{
      currentTimeZone = 4;
      break;
    }
    case 5:{
      currentTimeZone = 3;
      break;
    }
    case 6:{
      currentTimeZone = 0;
      break;
    }
    case 7:{
      currentTimeZone = -1;
      break;
    }
    case 8:{
      currentTimeZone = -2;
      break;
    }
    case 9:{
      currentTimeZone = -3;
      break;
    }
    case 10:{
      currentTimeZone = -4;
      break;
    }
    case 11:{
      currentTimeZone = -8;
      break;
    }
    case 12:{
      currentTimeZone = -9;
      break;
    }
    case 13:{
      currentTimeZone = -10;
      break;
    }
    case 14:{
      currentTimeZone = -12;
      break;
    }
  }
}

void OnTimer() {
  timerDone = true;
  timerOn = false;
}

void timer(){
  wd.pet();
  if (timerDone==true){
    Blynk.virtualWrite(V1, LOW);
    Blynk.notify("Time's UP!!!");
    x2 = matrix_1.width();
    for(int i = 0;i<54;i++){
    matrix_1.fillScreen(0);
    matrix_1.setCursor(x2, 0);
    tone(D3, 3401, 500); //connect two to D2
    tone(A5, 3817, 500);
    tone(D3, 3817, 500);
    tone(A5, 3401, 500); // connect two to A5
    matrix_1.print(F("Time's Up   "));
    if(--x2 < -36) {
        x2 = matrix_1.width();
        matrix_1.setTextColor(matrix_1.Color(80, 255, 0));
    }
    tone(D3, 3401, 500); //connect two to D2
    tone(A5, 3817, 500);
    tone(D3, 3817, 500);
    tone(A5, 3401, 500); // connect two to A5
    matrix_1.show();
    delay(100);
    }
    timerDone = false;
  } else if(timerOn == true){
    for(int i = timerTime;i>0;i-=1000){
    wd.pet();
    updateTimer.Update();
    Serial.println(i/1000);
    matrix_1.fillScreen(0);
    matrix_1.setCursor(2, 0);
    matrix_1.print(i/1000);
    if(--x2 < -36) {
      x2 = matrix_1.width();
      matrix_1.setTextColor(matrix_1.Color(80, 255, 0));
    }
    matrix_1.show();
    delay(1000);
    }
  } else {
    matrix_1.fillScreen(0);
    matrix_1.setCursor(x2, 0);
    matrix_1.print(F("TIMER"));
    if(--x2 < -36) {
      x2 = matrix_1.width();
      matrix_1.setTextColor(matrix_1.Color(80, 255, 0));
    }
    matrix_1.show();
    delay(100);
    }
}

void stopwatch(){
  wd.pet();
  if(stopWatchOn==1){
  secondsdisplay+= 0.01;
  seconds += 1;
  if(seconds==60){
    minutes+= 1;
    secondsdisplay= 0;
    seconds = 0;
    matrix_1.fillScreen(0);
    matrix_1.setCursor(1, 0);
    matrix_1.print(minutes);
    matrix_1.show();
    delay(500);
    matrix_1.fillScreen(0);
    matrix_1.setCursor(0, 0);
    matrix_1.print("M");
    matrix_1.show();
    delay(500);
  } else {
    matrix_1.fillScreen(0);
    matrix_1.setCursor(1, 0);
    matrix_1.print(seconds);
    matrix_1.show();
    delay(1000);
  }
  } else {
    if(seconds!=0 || minutes!=0){
      matrix_1.fillScreen(0);
      matrix_1.setCursor(1, 0);
      matrix_1.print(minutes);
      matrix_1.show();
      delay(500);
      x1 = matrix_1.width();
      for(int i =0;i<45;i++){
        wd.pet();
        matrix_1.fillScreen(0);
        matrix_1.setCursor(x1, 0);
        matrix_1.print("minutes");
        if(--x1 < -36) {
        x1 = matrix_1.width();
        matrix_1.setTextColor(matrix_1.Color(80, 255, 0));
        }
        matrix_1.show();
        delay(50);
      }
      matrix_1.fillScreen(0);
      matrix_1.setCursor(1, 0);
      matrix_1.print(seconds);
      matrix_1.show();
      delay(500);
      x1 = matrix_1.width();
      for(int i =0;i<45;i++){
        wd.pet();
        matrix_1.fillScreen(0);
        matrix_1.setCursor(x1, 0);
        matrix_1.print("seconds");
        if(--x1 < -36) {
        x1 = matrix_1.width();
        matrix_1.setTextColor(matrix_1.Color(80, 255, 0));
        }
        matrix_1.show();
        delay(50);
      }
      seconds = 0;
      minutes = 0;
      secondsdisplay = 0;
    } else {
      matrix_1.fillScreen(0);
      matrix_1.setCursor(x1, 0);
      matrix_1.print(F("STOPWATCH"));
      if(--x1 < -36) {
        x1 = matrix_1.width();
        matrix_1.setTextColor(matrix_1.Color(80, 255, 0));
      }
      matrix_1.show();
      delay(100);
    }
  }
}

void mainFunction()
{
  while(1){
  wd.pet();
  Blynk.run();
  BLYNK_WRITE(V1);
  BLYNK_WRITE(V0);
  BLYNK_WRITE(V2);
  BLYNK_WRITE(V3);
  matrix.setBrightness(brightness);
  matrix_1.setBrightness(brightness);
  matrix.setTextColor(matrix.Color(r,g,b));
  matrix_1.setTextColor(matrix.Color(r,g,b));
  sensors_event_t accelData, orientationData;
  bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  x = accelData.acceleration.x;
  y = accelData.acceleration.y;
  z = accelData.acceleration.z;
  ori = orientationData.orientation.x;
  Serial.println(x);
  Serial.println(y);
  Serial.println(z);
  Serial.println(ori);
  ntpTime->nowMillis(&now);
  timeNow = hhmmss(now.seconds, currentTimeZone);
  if(z>9&&x<3.00&&y<3.00){ 
    matrix_1.clear();
    matrix_1.show();
    if(millis() > waitMillis) {
      matrix.fillScreen(0);
      matrix.setCursor(x1, 0);
      matrix.print(timeNow);
      if(--x1 < -36) {
        x1 = matrix.width();
      }
      matrix.show();
      waitMillis = millis() + (300);
    }
  } else if(z<-9&&y<3.00&&x<3.00){
    matrix.clear();
    matrix.show();
    timer();
  } else if(y>9&&x<3.00&&z<3.00){
    matrix.clear();
    matrix.show();
    stopwatch();
  } else if(y<-9&&x<3.00&&z<3.00){
    matrix_1.clear();
    matrix_1.show();
    if (tempReceived!="")
  {
    matrix.fillScreen(0);
    matrix.setCursor(x1, 0);
    matrix.print(tempReceived + "F " + currentStatus + " Rain: " + precipProb + "%");
    if(--x1 < -230) {
    x1 = matrix.width();
    }
    matrix.show();
    delay(100);
  }
  } else if(x>9&&z<3.00&&y<3.00){
    if (ori>0&&ori<90){
      matrix_1.clear();
      matrix_1.show();
      matrix.fillScreen(matrix.Color(200,200,200));
      matrix.setBrightness(brightness);
      matrix.show();
      Serial.println("light");
    } else if (ori>90&&ori<180){
      matrix_1.clear();
      matrix_1.show();
      matrix.fillScreen(matrix.Color(255,255,255));
      matrix.setBrightness(brightness);
      matrix.show();
      delay(500);
      matrix.setBrightness(0);
      matrix.show();
      delay(500);
      Serial.println("flash");
    } else if (ori>180&&ori<270){
      matrix.clear();
      matrix.show();
      matrix_1.fillScreen(matrix.Color(r,g,b));
      matrix_1.setBrightness(brightness);
      matrix_1.show();
      Serial.println("customlight");
    } else if (ori>270&&ori<360){
      matrix.clear();
      matrix.show();
      matrix_1.setBrightness(brightness);
      strip.setBrightness(brightness);
      rainbow(1);
  } else if(x<-9&&z<3.00&&y<3.00){
    matrix.fillScreen(0);
    matrix.setCursor(x1, 0);
    matrix.print("BOX");
    if(--x1 < -36) {
    x1 = matrix.width();
    matrix.setTextColor(matrix.Color(80, 255, 0));
    }
    matrix.show();
    delay(300);
  }
  }
}
}

void loop(){
  delay(CONCURRENT_WAIT_FOREVER);
}
