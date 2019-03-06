//#include <SPI.h>
//#include <Adafruit_GFX.h>    // Core graphics library
#include <SPI.h>
#include <URTouch.h>
#include <TinyGPS++.h>
//#include <Adafruit_ILI9341.h>
#include <SdFat.h>
#include "SPI.h"
#include "ILI9341_t3.h"
SdFat SD;
File myFile;
//communication
const byte numChars = 200;
char receivedChars[numChars];
char tempChars[numChars];
char messageFromPC[numChars] = {0};
long ultrasonic[4] = {0, 0, 0, 0};
boolean newData = false;
//communication
#define _sdcs 2
#define TFT_CS 20
#define TFT_DC 21
#define TFT_CS_2 10
#define TFT_DC_2 6
#define t_SCK  14
#define t_CS   7
#define t_MOSI 8 //din
#define t_MISO 3 //do
#define t_IRQ  2
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);
ILI9341_t3 tft2 = ILI9341_t3(TFT_CS_2, TFT_DC_2);
URTouch ts(t_SCK, t_CS, t_MOSI, t_MISO, t_IRQ);

//horizont
#define REDRAW_DELAY 1 // minimum delay in milliseconds between display updates

#define HOR 240    // Horizon vector line length

#define ILI9341_BROWN      0x5140 //0x5960
#define ILI9341_BLUE   0x02B5 //0x0318 //0x039B //0x34BF
#define DARK_RED   0x8000
#define DARK_GREY  0x39C7

#define XC 120 // x coord of centre of horizon
#define YC 160 // y coord of centre of horizon

#define DEG2RAD 0.0174532925




// Variables for test only
int test_roll = 0;
int delta = 0;

unsigned long redrawTime = 0;
//

int but = 0;
int arm = 0;
int last_arm;
int last_but;
//variables
int buttonState = 0;
int trig = 1;
int i = 0;
int o = 0;
float start_latitude = 0.0;
float start_longtitude = 0.0;

double distanceToHome;
double courseToHome;
double courseToNavigate;
int smoothHeadingDegrees;
int pitch;
int roll;
float temperature;
long fix_data;
double latitude;
double longtitude;
long gps_month;
long gps_day;
long gps_year;
long time_hour;
long time_minute;
long fix_age;
float alt;
float speed_kmph;
float distanceToLondon;
//variables
bool state_arm = false;
bool state_data = false;
int times = 1;
double last_courseToHome;
float lat_des;
float long_des;
//double distanceToHome;
float bmp_temperature;
float bmp_pressure;
float bmp_altitude;
float altitude_offset = 0;
int roll_offset = 0;
int pitch_offset = 0;
int course_offset = 0;
int a = 0;
int b = 0;
int c = 0;
//last
int last_smoothHeadingDegrees;
int last_pitch;
int last_roll;
float last_temperature;
long last_fix_data = 0;
double last_latitude = 0;
double last_longtitude = 0;
long last_fix_age = 0;
long last_gps_month = 0, last_gps_day = 0, last_gps_year = 0;
long last_time_hour = 0, last_time_minute = 0, last_time_second = 0;
float last_alt = 0;
float last_speed_kmph = 0;
double last_distanceToLondon = 0;
double last_distanceToPoint = 0;
double distanceToPoint;
float last_start_latitude;
float last_start_longtitude;
double last_distanceToHome = 0;
float last_bmp_temperature = 0;
float last_bmp_pressure = 0;
float last_bmp_altitude = 0;
long last_millis_printDate = 0;
long last_millis_printTime = 0;
bool compass = false;
bool horizont = false;
bool data = false;
bool settings = false;
bool onoff = false;
bool setLatLong = false;
bool noSetLatLong = false;
bool navigate = false;
double last_courseToNavigate = 0;
float LONGTITUDE;
float LATITUDE;
float last_LONGTITUDE;
float last_LATITUDE;
int last_compass = 0;
int last_gps_course;
int gps_course;
String Latitude;
String Longtitude;
String inputLatLong;
//last
int count = 1;
unsigned long previousMillis = 0;
long previousmls = 0;
const long interval = 700;
class Communication {
  public:
    void initialization() {
      Serial1.begin(9600);
    }
    void receive()  {
      recvWithStartEndMarkers();
      if (newData == true) {
        strcpy(tempChars, receivedChars);
        parseData();
        newData = false;
      }
    }

    void recvWithStartEndMarkers() {
      static boolean recvInProgress = false;
      static byte ndx = 0;
      char startMarker = '<';
      char endMarker = '>';
      char rc;
      while (Serial1.available() > 0 && newData == false) {
        rc = Serial1.read();
        if (recvInProgress == true) {
          if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
              ndx = numChars - 1;
            }
          }
          else {
            receivedChars[ndx] = '\0';
            recvInProgress = false;
            ndx = 0;
            newData = true;
          }
        }
        else if (rc == startMarker) {
          recvInProgress = true;
        }
      }
    }
    void parseData() {
      int packet;
      char * strtokIndx;
      strtokIndx = strtok(tempChars, ",");
      strcpy(messageFromPC, strtokIndx);
      strtokIndx = strtok(NULL, ",");
      packet = atoi(strtokIndx);
      if (packet == 1) {
        strtokIndx = strtok(NULL, ",");
        smoothHeadingDegrees = atoi(strtokIndx);
        strtokIndx = strtok(NULL, ",");
        pitch = atoi(strtokIndx);
        strtokIndx = strtok(NULL, ",");
        roll = atoi(strtokIndx);
        pitch += 5;
      }
      else if (packet == 2)  {
        strtokIndx = strtok(NULL, ",");
        temperature = atof(strtokIndx);
        strtokIndx = strtok(NULL, ",");
        fix_data = atol(strtokIndx);
        strtokIndx = strtok(NULL, ",");
        latitude = atof(strtokIndx);
        strtokIndx = strtok(NULL, ",");
        longtitude = atof(strtokIndx);
        strtokIndx = strtok(NULL, ",");
        gps_month = atol(strtokIndx);
        strtokIndx = strtok(NULL, ",");
        gps_day = atol(strtokIndx);
        strtokIndx = strtok(NULL, ",");
        gps_year = atol(strtokIndx);
        strtokIndx = strtok(NULL, ",");
        time_hour = atol(strtokIndx);
        strtokIndx = strtok(NULL, ",");
        time_minute = atol(strtokIndx);
        strtokIndx = strtok(NULL, ",");
        fix_age = atol(strtokIndx);
        strtokIndx = strtok(NULL, ",");
        speed_kmph = atoi(strtokIndx);
        strtokIndx = strtok(NULL, ",");
        bmp_altitude = atoi(strtokIndx);
        strtokIndx = strtok(NULL, ",");
        gps_course = atoi(strtokIndx);
      }
    }
};
void setup(void) {
  Communication comp;
  comp.initialization();
  /*if (!SD.begin(_sdcs, SPI_FULL_SPEED)) {
    //if (!SD.begin(_sdcs)) {
    Serial.println(F("failed!"));
    }
    myFile = SD.open("test.txt", FILE_WRITE);

    // if the file opened okay, write to it:
    if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    Serial.println("done.");
    } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
    }*/
  ts.InitTouch();
  ts.setPrecision(PREC_MEDIUM);
  Serial.begin(9600);
  ts.InitTouch();
  ts.setPrecision(PREC_MEDIUM);
  tft.begin();
  tft2.begin();
  tft.setRotation(0);
  tft2.setRotation(0);

  tft.fillScreen(ILI9341_BLUE);
  tft2.fillScreen(ILI9341_WHITE);
  tft2.setRotation(0);
  tft2.setTextColor(ILI9341_BLACK);
  tft2.setTextSize(2);
  tft2.setCursor(5, 2);
  tft2.print("Incoming data: ");

  tft2.setRotation(1);

  tft2.drawLine(285, 240, 285, 0, ILI9341_BLACK);
  tft2.drawLine(286, 240, 286, 0, ILI9341_BLACK);
  tft2.drawLine(284, 240, 284, 0, ILI9341_BLACK);

  tft2.drawLine(285, 60, 320, 60, ILI9341_BLACK);
  tft2.drawLine(285, 61, 320, 61, ILI9341_BLACK);
  tft2.drawLine(285, 59, 320, 59, ILI9341_BLACK);

  tft2.drawLine(285, 120, 320, 120, ILI9341_BLACK);
  tft2.drawLine(285, 121, 320, 121, ILI9341_BLACK);
  tft2.drawLine(285, 119, 320, 119, ILI9341_BLACK);

  tft2.drawLine(285, 180, 320, 180, ILI9341_BLACK);
  tft2.drawLine(285, 181, 320, 181, ILI9341_BLACK);
  tft2.drawLine(285, 179, 320, 179, ILI9341_BLACK);

  tft2.drawLine(320, 119, 250 - 25, 119, ILI9341_BLACK);
  tft2.drawLine(320, 120, 250 - 25, 120, ILI9341_BLACK);
  tft2.drawLine(320, 121, 250 - 25, 121, ILI9341_BLACK);


  tft2.drawLine(249 - 25, 120, 249 - 25, 0, ILI9341_BLACK);
  tft2.drawLine(250 - 25, 120, 250 - 25, 0, ILI9341_BLACK);
  tft2.drawLine(251 - 25, 120, 251 - 25 , 0, ILI9341_BLACK);

  tft2.drawLine(249 + 5, 120, 249 + 5, 0, ILI9341_BLACK);
  tft2.drawLine(250 + 5, 120, 250 + 5, 0, ILI9341_BLACK);
  tft2.drawLine(251 + 5, 120, 251 + 5 , 0, ILI9341_BLACK);

  tft2.setRotation(0);

  tft2.setTextColor(ILI9341_RED);
  tft2.setTextSize(3);
  tft2.setCursor(73, 293);
  tft2.print("ON");
  tft2.setTextSize(2);

  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLUE);
  pinMode(22, OUTPUT);
  pinMode(17, INPUT);


  tft2.setTextSize(1);
  tft2.setTextColor(ILI9341_RED);
  tft2.setCursor(175, 2);
  tft2.print("FALSE");
  tft2.setTextSize(2);
  printDataText();

  tft.setRotation(0);
  tft.fillRect(0,  0, 240, 160, ILI9341_BLUE);
  tft.fillRect(0, 160, 240, 160, ILI9341_BROWN);
  drawInfo();
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_RED);
  tft.setCursor(140, 225);
  tft.print("DISARMED");
}

void loop() {
  Communication comp;
  comp.receive();
  if (roll > 20 or roll < -20 or pitch > 20 or pitch < -20) {
    unsigned long currentmls = millis();
    long a = 500;
    a = map(abs(roll), 20, 90, 700, 0);
    int buzzerState = LOW;
    if (currentmls - previousmls > a)  {
      previousmls = currentmls;
      if (buzzerState == LOW)  {
        buzzerState = HIGH;
      }
      else
        buzzerState = LOW;
      //Serial.println(a);
    }
    digitalWrite(22, buzzerState);
    delay(5);
  }
  else  {
    digitalWrite(22, LOW);
  }

  if (last_smoothHeadingDegrees != 0 or
      last_pitch != 0 or
      last_roll != 0 or
      last_temperature != 0 or
      last_fix_data != 0 or
      last_latitude != 0 or
      last_longtitude != 0 or
      last_gps_month != 0 or
      last_gps_day != 0 or
      last_gps_year != 0 or
      last_time_hour != 0 or
      last_time_minute != 0 or
      last_fix_age != 0 or
      last_alt != 0 or
      last_speed_kmph != 0 or
      last_distanceToLondon != 0 )  {
    if (times == 1)  {
      tft2.setTextSize(1);
      tft2.setTextColor(ILI9341_WHITE);
      tft2.setCursor(175, 2);
      tft2.print("FALSE");
      tft2.setTextColor(ILI9341_GREEN);
      tft2.setCursor(175, 2);
      tft2.print("TRUE");
      times++;
    }

  }
  printTouchBut();
  buttonState = digitalRead(17);
  if (buttonState == HIGH && state_arm == false) {
    but = 0;
    arm = 1;
    Serial1.print("<DATA,");
    Serial1.print(but);
    Serial1.print(",");
    Serial1.print(arm);
    Serial1.println(">");
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_BROWN);
    tft.setCursor(140, 225);
    tft.print("DISARMED");
    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(140, 225);
    tft.print("ARMED");
    state_arm = true;
  }
  else if (buttonState == LOW && state_arm == true) {
    but = 0;
    arm = 0;
    Serial1.print("<DATA,");
    Serial1.print(but);
    Serial1.print(",");
    Serial1.print(arm);
    Serial1.println(">");
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_BROWN);
    tft.setCursor(140, 225);
    tft.print("ARMED");
    tft.setTextColor(ILI9341_RED);
    tft.setCursor(140, 225);
    tft.print("DISARMED");
    state_arm = false;
  }
  long x, y;
  tft2.setTextSize(1);
  while (ts.dataAvailable())  {
    ts.read();
    x = ts.getX() + 15;
    y = ts.getY() + 5;
    if ((x != -1) && (y != -1)) {
      Serial.print("X: ");
      Serial.print(x);
      Serial.print("   Y: ");
      Serial.println(y);
      if (y > 180 &&  x >= 285) {
        but = 1;
        Serial1.print("<DATA,");
        Serial1.print(but);
        Serial1.print(",");
        Serial1.print(arm);
        Serial1.println(">");
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(15, 300);
        tft2.print("FOTO");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (y < 180 && y > 120 && x >= 285)  {
        but = 2;
        Serial1.print("<DATA,");
        Serial1.print(but);
        Serial1.print(",");
        Serial1.print(arm);
        Serial1.println(">");
        trig++;
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
        delay(100);
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (y < 120 && y > 60 && x >= 285) {
        but = 3;
        Serial1.print("<DATA,");
        Serial1.print(but);
        Serial1.print(",");
        Serial1.print(arm);
        Serial1.println(">");
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(135, 300);
        tft2.print("ZOOM +");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (y < 60 && x >= 285)  {
        but = 4;
        Serial1.print("<DATA,");
        Serial1.print(but);
        Serial1.print(",");
        Serial1.print(arm);
        Serial1.println(">");
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(195, 300);
        tft2.print("ZOOM -");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (y <= 125 and x >= 250 + 10 and x <= 280 + 10)  {
        but = 5;
        a++;
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (y <= 125 and x >= 260 - 35 and x <= 290 - 35)  {
        but = 6;
        b++;
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (y <= 125 and x >= 203 and x <= 230 and data == false)  {
        but = 7;
        c++;
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (y <= 125 and x >= 182 and x <= 198 and data == false and setLatLong == false)  {
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_WHITE);
        tft2.setCursor(70, 65 + 15);
        tft2.print(last_LONGTITUDE, 6);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(40 + 110, 182);
        tft2.print("LONGTITUDE");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(40 + 110, 182);
        tft2.print("LONGTITUDE");
        Longtitude = inputLatLong;
        inputLatLong = "";
        printInput();
        LONGTITUDE = Longtitude.toFloat();
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(70, 65 + 15);
        tft2.print(LONGTITUDE, 6);
      }
      if (y >= 125 and x >= 182 and x <= 198 and data == false and setLatLong == false)  {
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_WHITE);
        tft2.setCursor(70, 55 + 15);
        tft2.print(last_LATITUDE, 6);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(35, 182);
        tft2.print("LATITUDE");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(35, 182);
        tft2.print("LATITUDE");
        Latitude = inputLatLong;
        inputLatLong = "";
        printInput();
        LATITUDE = Latitude.toFloat();
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(70, 55 + 15);
        tft2.print(LATITUDE, 6);
      }
      if (y >= 201 and x >= 142 and x <= 159 and data == false and setLatLong == false) {
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(20, 182 - 40);
        tft2.print("1");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(20, 182 - 40);
        tft2.print("1");
        inputLatLong += 1;
        printInput();
      }
      if (y >= 151 and y <= 200 and x >= 142 and x <= 159 and data == false and setLatLong == false) {
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(20 + 48, 182 - 40);
        tft2.print("2");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(20 + 48, 182 - 40);
        tft2.print("2");
        inputLatLong += 2;
        printInput();
      }
      if (y >= 101 and y <= 150 and x >= 142 and x <= 159 and data == false and setLatLong == false) {
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(20 + 48 * 2, 182 - 40);
        tft2.print("3");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(20 + 48 * 2, 182 - 40);
        tft2.print("3");
        inputLatLong += 3;
        printInput();
      }
      if (y >= 56 and y <= 100 and x >= 142 and x <= 159 and data == false and setLatLong == false) {
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(20 + 48 * 3, 182 - 40);
        tft2.print("4");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(20 + 48 * 3, 182 - 40);
        tft2.print("4");
        inputLatLong += 4;
        printInput();
      }
      if (y <= 55 and x >= 142 and x <= 159 and data == false and setLatLong == false) {
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(20 + 48 * 4, 182 - 40);
        tft2.print("5");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(20 + 48 * 4, 182 - 40);
        tft2.print("5");
        inputLatLong += 5;
        printInput();
      }
      if (y >= 201 and x >= 117 and x <= 137 and data == false and setLatLong == false) {
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(20, 182 - 60);
        tft2.print("C");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(20, 182 - 60);
        tft2.print("C");
        inputLatLong = "";
        printInput();
      }
      if (y >= 151 and y <= 200 and x >= 117 and x <= 137 and data == false and setLatLong == false) {
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(15 + 48, 182 - 60);
        tft2.print("<-");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(15 + 48, 182 - 60);
        tft2.print("<-");
        int length = inputLatLong.length();
        inputLatLong.remove(length - 1);
        printInput();
      }
      if (y >= 101 and y <= 150 and x >= 117 and x <= 137 and data == false and setLatLong == false) {
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(20 + 48 * 2, 182 - 60);
        tft2.print(".");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(20 + 48 * 2, 182 - 60);
        tft2.print(".");
        inputLatLong += '.';
        printInput();
      }
      if (y >= 56 and y <= 100 and x >= 117 and x <= 137 and data == false and setLatLong == false) {
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(20 + 48 * 3, 182 - 60);
        tft2.print("0");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(20 + 48 * 3, 182 - 60);
        tft2.print("0");
        inputLatLong += 0;
        printInput();
      }
      if (y <= 55 and x >= 117 and x <= 137 and data == false and setLatLong == false) {
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(10 + 48 * 4, 182 - 60);
        tft2.print("ENTER");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(10 + 48 * 4, 182 - 60);
        tft2.print("ENTER");
        navigate = true;
        if (horizont == true)  {
          //b++; back to main screen
          a++;
        }
      }
      if (y >= 201 and x >= 163 and x <= 176 and data == false and setLatLong == false) {
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(20, 182 - 20);
        tft2.print("6");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(20, 182 - 20);
        tft2.print("6");
        inputLatLong += 6;
        printInput();
      }
      if (y >= 151 and y <= 200 and x >= 163 and x <= 176 and data == false and setLatLong == false) {
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(20 + 48, 182 - 20);
        tft2.print("7");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(20 + 48, 182 - 20);
        tft2.print("7");
        inputLatLong += 7;
        printInput();
      }
      if (y >= 101 and y <= 150 and x >= 163 and x <= 176 and data == false and setLatLong == false) {
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(20 + 48 * 2, 182 - 20);
        tft2.print("8");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(20 + 48 * 2, 182 - 20);
        tft2.print("8");
        inputLatLong += 8;
        printInput();
      }
      if (y >= 56 and y <= 100 and x >= 163 and x <= 176 and data == false and setLatLong == false) {
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(20 + 48 * 3, 182 - 20);
        tft2.print("9");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(20 + 48 * 3, 182 - 20);
        tft2.print("9");
        inputLatLong += 9;
        printInput();
      }
      if (y <= 55 and x >= 163 and x <= 176 and data == false and setLatLong == false) {
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(20 + 48 * 4, 182 - 20);
        tft2.print("-");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(20 + 48 * 4, 182 - 20);
        tft2.print("-");
        inputLatLong += '-';
        printInput();
      }
      if (x >= 50 and x <= 75 and y >= 100 and y <= 135 and data == false and noSetLatLong == false) {
        altitude_offset += 1;
        printAltitude(40, 65, 1);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(115, 55);
        tft2.print("+1");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (x >= 50 and x <= 75 and y >= 70 and y <= 99 and data == false and noSetLatLong == false) {
        altitude_offset -= 1;
        printAltitude(40, 65, 1);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(115 + 35, 55);
        tft2.print("-1");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (x >= 50 and x <= 75 and y >= 35 and y <= 69 and data == false and noSetLatLong == false) {
        altitude_offset += 10;
        printAltitude(40, 65, 1);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(115 + 35 * 2, 55);
        tft2.print("+10");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (x >= 50 and x <= 75 and y < 35 and data == false and noSetLatLong == false) {
        altitude_offset -= 10;
        printAltitude(40, 65, 1);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(115 + 35 * 3, 55);
        tft2.print("-10");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      //printHeading(40, 65 + 35, 1);
      if (x >= 84 and x <= 109 and y >= 100 and y <= 135 and data == false and noSetLatLong == false) {
        course_offset += 1;
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(115, 55 + 35);
        tft2.print("+1");
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(40, 65);
        tft2.print(smoothHeadingDegrees);
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);

      }
      if (x >= 84 and x <= 109 and y >= 70 and y <= 99 and data == false and noSetLatLong == false) {
        course_offset -= 1;
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(115 + 35, 55 + 35);
        tft2.print("-1");
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(40, 65);
        tft2.print(smoothHeadingDegrees);
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (x >= 84 and x <= 109 and y >= 35 and y <= 69 and data == false and noSetLatLong == false) {
        course_offset += 10;
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(115 + 35 * 2, 55 + 35);
        tft2.print("+10");
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(40, 65);
        tft2.print(smoothHeadingDegrees);
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (x >= 84 and x <= 109 and y < 35 and data == false and noSetLatLong == false) {
        course_offset -= 10;
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(115 + 35 * 3, 55 + 35);
        tft2.print("-10");
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_BLACK);
        tft2.setCursor(40, 65);
        tft2.print(smoothHeadingDegrees);
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (x >= 118 and x <= 143 and y >= 100 and y <= 135 and data == false and noSetLatLong == false) {
        pitch_offset += 1;
        printPitch(40, 65 + 70, 1);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(115, 55 + 70);
        tft2.print("+1");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (x >= 118 and x <= 143 and y >= 70 and y <= 99 and data == false and noSetLatLong == false) {
        pitch_offset -= 1;
        printPitch(40, 65 + 70, 1);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(115 + 35, 55 + 70);
        tft2.print("-1");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (x >= 118 and x <= 143 and y >= 35 and y <= 69 and data == false and noSetLatLong == false) {
        pitch_offset += 10;
        printPitch(40, 65 + 70, 1);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(115 + 35 * 2, 55 + 70);
        tft2.print("+10");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (x >= 118 and x <= 143 and y < 35 and data == false and noSetLatLong == false) {
        pitch_offset -= 10;
        printPitch(40, 65 + 70, 1);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(115 + 35 * 3, 55 + 70);
        tft2.print("-10");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }

      if (x >= 153 and x <= 181 and y >= 100 and y <= 135 and data == false and noSetLatLong == false) {
        roll_offset += 1;
        printRoll(40, 65 + 105, 1);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(115, 55 + 105);
        tft2.print("+1");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (x >= 153 and x <= 181 and y >= 70 and y <= 99 and data == false and noSetLatLong == false) {
        roll_offset -= 1;
        printRoll(40, 65 + 105, 1);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(115 + 35, 55 + 105);
        tft2.print("-1");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (x >= 153 and x <= 181 and y >= 35 and y <= 69 and data == false and noSetLatLong == false) {
        roll_offset += 10;
        printRoll(40, 65 + 105, 1);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(115 + 35 * 2, 55 + 105);
        tft2.print("+10");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (x >= 153 and x <= 181 and y < 35 and data == false and noSetLatLong == false) {
        roll_offset -= 10;
        printRoll(40, 65 + 105, 1);
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setCursor(115 + 35 * 3, 55 + 105);
        tft2.print("-10");
        digitalWrite(22, HIGH);
        delay(100);
        digitalWrite(22, LOW);
      }
      if (trig % 2 == 0) {
        tft2.setTextColor(ILI9341_GREEN);
        tft2.setTextSize(3);
        tft2.setCursor(73, 293);
        tft2.print("ON");
      }
      if (trig % 2 != 0)  {
        tft2.setTextColor(ILI9341_RED);
        tft2.setTextSize(3);
        tft2.setCursor(73, 293);
        tft2.print("ON");
      }
    }
  }
  if (a % 2 == 0 and horizont == false) {
    tft.setRotation(0);
    tft.fillRect(0,  0, 240, 160, ILI9341_BLUE);
    tft.fillRect(0, 160, 240, 160, ILI9341_BROWN);
    tft2.setTextColor(ILI9341_WHITE);
    tft2.setTextSize(2);
    tft2.setCursor(135, 263);
    tft2.print("HORIZONT");
    tft2.setTextColor(ILI9341_RED);
    tft2.setTextSize(2);
    tft2.setCursor(140, 263);
    tft2.print("COMPASS");
    drawInfo();
  }
  if (a % 2 != 0 and compass == false) {
    tft2.setTextColor(ILI9341_WHITE);
    tft2.setTextSize(2);
    tft2.setCursor(140, 263);
    tft2.print("COMPASS");
    tft2.setTextColor(ILI9341_BLUE);
    tft2.setTextSize(2);
    tft2.setCursor(135, 263);
    tft2.print("HORIZONT");
    tft.fillScreen(ILI9341_BLACK);
    tft.drawCircle(120, 160, 98 + 15, ILI9341_WHITE);
    tft.drawCircle(120, 160, 99 + 15, ILI9341_BLACK);
    tft.drawCircle(120, 160, 100 + 15, ILI9341_BLACK);
    tft.drawCircle(120, 160, 101 + 15, ILI9341_BLACK);
    tft.drawCircle(120, 160, 102 + 15, ILI9341_WHITE);
    tft.fillCircle(120, 160, 96 + 15, ILI9341_WHITE);
    drawCompassInfo();

  }
  if (a % 2 != 0)  {
    horizont = false;
    compass = true;

    drawCourseToHome(courseToHome, 0);

    updateCompass();
  }
  if (a % 2 == 0) {
    compass = false;
    horizont = true;
    updateHorizont();
  }

  if (b % 2 == 0 and data == false) {
    tft2.setRotation(0);
    tft2.fillRect(0,  44, 240, 180, ILI9341_WHITE);
    printDataText();
    tft2.setTextColor(ILI9341_WHITE);
    tft2.setTextSize(2);
    tft2.setCursor(135 + 20, 263 - 30);
    tft2.print("DATA");
    tft2.setTextColor(ILI9341_RED);
    tft2.setTextSize(2);
    tft2.setCursor(135, 263 - 30);
    tft2.print("SETTINGS");
  }
  if (b % 2 != 0 and settings == false) {
    tft2.setRotation(1);
    tft2.drawLine(249 + 5 - 60, 120, 249 + 5 - 60, 0, ILI9341_BLACK);
    tft2.drawLine(250 + 5 - 60, 120, 250 + 5 - 60, 0, ILI9341_BLACK);
    tft2.drawLine(251 + 5 - 60, 120, 251 + 5 - 60, 0, ILI9341_BLACK);
    tft2.drawLine(249 + 5 - 60, 120, 230, 120, ILI9341_BLACK);
    tft2.drawLine(249 + 5 - 60, 121, 230, 121, ILI9341_BLACK);
    tft2.drawLine(249 + 5 - 60, 119, 230, 119, ILI9341_BLACK);
    tft2.setRotation(0);
    tft2.fillRect(0,  44, 240, 149, ILI9341_WHITE);
    tft2.fillRect(0,  40, 118, 180, ILI9341_WHITE);
    tft2.fillRect(120,  40, 240, 150, ILI9341_WHITE);
    tft2.setTextColor(ILI9341_WHITE);
    tft2.setTextSize(2);
    tft2.setCursor(135, 263 - 30);
    tft2.print("SETTINGS");
    tft2.setTextColor(ILI9341_BLUE);
    tft2.setTextSize(2);
    tft2.setCursor(135 + 20, 263 - 30);
    tft2.print("DATA");
    if (c % 2 != 0)  {
      printLATLOT();
      tft2.setRotation(0);
      tft2.setTextSize(2);
      tft2.setTextColor(ILI9341_RED);
      tft2.setCursor(55, 38 + 15);
      tft2.print("Navigate to:");
      tft2.setTextSize(1);
      tft2.setTextColor(ILI9341_BLACK);
      tft2.setCursor(5, 55 + 15);
      tft2.print("Latitude:");
      tft2.setCursor(5, 65 + 15);
      tft2.print("Longtitude:");
      tft2.setTextColor(ILI9341_BLUE);
      tft2.setRotation(0);
      tft2.setTextSize(2);
      tft2.setCursor(145, 263 - 60);
      tft2.fillRect(122,  170 + 27, 240, 27, ILI9341_WHITE);
      tft2.print("OFFSET");
    }
    else  {
      tft2.setTextColor(ILI9341_RED);
      tft2.setRotation(0);
      tft2.setTextSize(2);
      tft2.setCursor(135, 263 - 60);
      tft2.fillRect(122,  170 + 27, 240, 27, ILI9341_WHITE);
      tft2.print("NAVIGATE");
    }

  }
  if (b % 2 != 0)  {
    data = false;
    settings = true;
    if (c % 2 == 0 and setLatLong == false) {
      tft2.setRotation(1);
      tft2.drawLine(249 + 5 - 60, 120, 249 + 5 - 60, 0, ILI9341_BLACK);
      tft2.drawLine(250 + 5 - 60, 120, 250 + 5 - 60, 0, ILI9341_BLACK);
      tft2.drawLine(251 + 5 - 60, 120, 251 + 5 - 60, 0, ILI9341_BLACK);
      tft2.drawLine(249 + 5 - 60, 120, 230, 120, ILI9341_BLACK);
      tft2.drawLine(249 + 5 - 60, 121, 230, 121, ILI9341_BLACK);
      tft2.drawLine(249 + 5 - 60, 119, 230, 119, ILI9341_BLACK);
      tft2.setRotation(0);
      tft2.fillRect(0,  44, 240, 153, ILI9341_WHITE);
      tft2.fillRect(120,  170 + 25, 240, 29, ILI9341_WHITE);
      tft2.fillRect(122,  170 + 27, 240, 27, ILI9341_WHITE);
      tft2.setTextColor(ILI9341_WHITE);
      tft2.setTextSize(2);
      tft2.setCursor(145, 263 - 60);
      tft2.print("OFFSET");
      tft2.setTextColor(ILI9341_RED);
      tft2.setTextSize(2);
      tft2.setCursor(135, 263 - 60);
      tft2.print("NAVIGATE");

    }
    if (c % 2 != 0 and noSetLatLong == false) {
      tft2.setRotation(1);
      tft2.drawLine(249 + 5 - 60, 120, 249 + 5 - 60, 0, ILI9341_BLACK);
      tft2.drawLine(250 + 5 - 60, 120, 250 + 5 - 60, 0, ILI9341_BLACK);
      tft2.drawLine(251 + 5 - 60, 120, 251 + 5 - 60, 0, ILI9341_BLACK);
      tft2.drawLine(249 + 5 - 60, 120, 230, 120, ILI9341_BLACK);
      tft2.drawLine(249 + 5 - 60, 121, 230, 121, ILI9341_BLACK);
      tft2.drawLine(249 + 5 - 60, 119, 230, 119, ILI9341_BLACK);


      tft2.setRotation(0);
      tft2.fillRect(0,  44, 240, 149, ILI9341_WHITE);
      tft2.fillRect(122,  170 + 27, 240, 27, ILI9341_WHITE);
      tft2.setTextColor(ILI9341_WHITE);
      tft2.setTextSize(2);
      tft2.setCursor(135, 263 - 60);
      tft2.print("NAVIGATE");
      tft2.setTextColor(ILI9341_BLUE);
      tft2.setTextSize(2);
      tft2.setCursor(145, 263 - 60);
      tft2.print("OFFSET");
      printLATLOT();
      tft2.setRotation(0);
      tft2.setTextSize(2);
      tft2.setTextColor(ILI9341_RED);
      tft2.setCursor(55, 38 + 15);
      tft2.print("Navigate to:");
      tft2.setTextSize(1);
      tft2.setTextColor(ILI9341_BLACK);
      tft2.setCursor(5, 55 + 15);
      tft2.print("Latitude:");
      tft2.setCursor(5, 65 + 15);
      tft2.print("Longtitude:");
    }
    if (c % 2 != 0)  {
      setLatLong = false;
      noSetLatLong = true;
    }
    if ( c % 2 == 0) {
      setLatLong = true;
      noSetLatLong = false;
    }
    if (noSetLatLong == false)  {
      tft2.setRotation(1);
      tft2.drawLine(249 + 5 - 60, 120, 249 + 5 - 60, 0, ILI9341_BLACK);
      tft2.drawLine(250 + 5 - 60, 120, 250 + 5 - 60, 0, ILI9341_BLACK);
      tft2.drawLine(251 + 5 - 60, 120, 251 + 5 - 60, 0, ILI9341_BLACK);
      tft2.drawLine(249 + 5 - 60, 120, 230, 120, ILI9341_BLACK);
      tft2.drawLine(249 + 5 - 60, 121, 230, 121, ILI9341_BLACK);
      tft2.drawLine(249 + 5 - 60, 119, 230, 119, ILI9341_BLACK);
      tft2.setRotation(0);
      tft2.setTextColor(ILI9341_BLACK);
      tft2.setTextSize(2);
      tft2.setCursor(5, 45);
      tft2.print("Altitude");
      printAltitude(40, 65, 1);
      tft2.setTextSize(2);
      tft2.setCursor(10, 45 + 35);
      tft2.print("Course");
      printHeading(40, 65 + 35, 1);
      tft2.setTextSize(2);
      tft2.setCursor(18, 45 + 70);
      tft2.print("Pitch");
      printPitch(40, 65 + 70, 1);
      tft2.setTextSize(2);
      tft2.setCursor(25, 45 + 105);
      tft2.print("Roll");
      printRoll(40, 65 + 105, 1);
      tft2.setTextSize(1);
      tft2.setCursor(115, 55);
      tft2.print("+1");
      tft2.setCursor(115 + 35, 55);
      tft2.print("-1");
      tft2.setCursor(115 + 35 * 2, 55);
      tft2.print("+10");
      tft2.setCursor(115 + 35 * 3, 55);
      tft2.print("-10");
      tft2.drawFastVLine(105, 45, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106, 45, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107, 45, 25, ILI9341_BLACK);
      tft2.drawFastVLine(105 + 35, 45, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106 + 35, 45, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107 + 35, 45, 25, ILI9341_BLACK);
      tft2.drawFastVLine(105 + 35 * 2, 45, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106 + 35 * 2, 45, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107 + 35 * 2, 45, 25, ILI9341_BLACK);
      tft2.drawFastVLine(105 + 35 * 3, 45, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106 + 35 * 3, 45, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107 + 35 * 3, 45, 25, ILI9341_BLACK);
      tft2.drawFastVLine(105 + 35 * 4, 45, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106 + 35 * 4, 45, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107 + 35 * 4, 45, 25, ILI9341_BLACK);
      tft2.drawFastHLine(105, 45, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 46, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 44, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 45 + 25, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 46 + 25, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 44 + 25, 35 * 4, ILI9341_BLACK);
      int o = 35;
      tft2.setCursor(115, 55 + o);
      tft2.print("+1");
      tft2.setCursor(115 + 35, 55 + o);
      tft2.print("-1");
      tft2.setCursor(115 + 35 * 2, 55 + o);
      tft2.print("+10");
      tft2.setCursor(115 + 35 * 3, 55 + o);
      tft2.print("-10");
      tft2.drawFastVLine(105, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(105 + 35, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106 + 35, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107 + 35, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(105 + 35 * 2, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106 + 35 * 2, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107 + 35 * 2, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(105 + 35 * 3, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106 + 35 * 3, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107 + 35 * 3, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(105 + 35 * 4, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106 + 35 * 4, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107 + 35 * 4, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastHLine(105, 45 + o, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 46 + o, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 44 + o, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 45 + 25 + o, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 46 + 25 + o, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 44 + 25 + o, 35 * 4, ILI9341_BLACK);
      o = 70;
      tft2.setCursor(115, 55 + o);
      tft2.print("+1");
      tft2.setCursor(115 + 35, 55 + o);
      tft2.print("-1");
      tft2.setCursor(115 + 35 * 2, 55 + o);
      tft2.print("+10");
      tft2.setCursor(115 + 35 * 3, 55 + o);
      tft2.print("-10");
      tft2.drawFastVLine(105, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(105 + 35, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106 + 35, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107 + 35, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(105 + 35 * 2, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106 + 35 * 2, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107 + 35 * 2, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(105 + 35 * 3, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106 + 35 * 3, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107 + 35 * 3, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(105 + 35 * 4, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106 + 35 * 4, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107 + 35 * 4, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastHLine(105, 45 + o, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 46 + o, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 44 + o, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 45 + 25 + o, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 46 + 25 + o, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 44 + 25 + o, 35 * 4, ILI9341_BLACK);
      o = 105;
      tft2.setCursor(115, 55 + o);
      tft2.print("+1");
      tft2.setCursor(115 + 35, 55 + o);
      tft2.print("-1");
      tft2.setCursor(115 + 35 * 2, 55 + o);
      tft2.print("+10");
      tft2.setCursor(115 + 35 * 3, 55 + o);
      tft2.print("-10");
      tft2.drawFastVLine(105, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(105 + 35, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106 + 35, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107 + 35, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(105 + 35 * 2, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106 + 35 * 2, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107 + 35 * 2, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(105 + 35 * 3, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106 + 35 * 3, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107 + 35 * 3, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(105 + 35 * 4, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(106 + 35 * 4, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastVLine(107 + 35 * 4, 45 + o, 25, ILI9341_BLACK);
      tft2.drawFastHLine(105, 45 + o, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 46 + o, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 44 + o, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 45 + 25 + o, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 46 + 25 + o, 35 * 4, ILI9341_BLACK);
      tft2.drawFastHLine(105, 44 + 25 + o, 35 * 4, ILI9341_BLACK);
    }
  }
  if (b % 2 == 0) {
    settings = false;
    data = true;
    distanceToHome =  TinyGPSPlus::distanceBetween(latitude, longtitude, start_latitude, start_longtitude);
    courseToHome =    TinyGPSPlus::courseTo((double)latitude, (double)longtitude, (double)start_latitude, (double)start_longtitude);
    printFixData(90, 40, 1);
    printLatitude(130, 55, 2);
    printLongtitude(130, 75, 2);
    printTemperature(150, 145, 2);
    printFixAge(185, 40, 1);
    printSpeed(150, 165, 2);
    printStartLatLon(95, 95, 1);
    printHeading(185, 125, 2);
    printCourseToHome(185, 105, 2);
    printDistanceToHome(100, 185, 2);
    printBut(110, 270, 1);
    printArmState(110, 260, 1);
    printAltitude(110, 205, 2);
  }


  tft2.setTextSize(2);
  //<DATA,34,357,-28,8.69,17,48.06,18.33,1,29,2018,22,30,643,0.18,7.61,1055.65>

  if ( last_smoothHeadingDegrees != smoothHeadingDegrees or
       last_pitch != pitch or
       last_roll != roll or
       last_temperature != temperature or
       last_fix_data != fix_data or
       last_latitude != latitude or
       last_longtitude != longtitude or
       last_gps_month != gps_month or
       last_gps_day != gps_day or
       last_gps_year != gps_year or
       last_time_hour != time_hour or
       last_time_minute != time_minute or
       last_fix_age != fix_age or
       last_alt != alt or
       last_speed_kmph != speed_kmph or
       last_distanceToLondon != distanceToLondon
     ) {
    if (state_data == false)  {
      tft2.setTextSize(1);
      tft2.setTextColor(ILI9341_WHITE);
      tft2.setCursor(175, 10);
      tft2.print("MEMORY");
      tft2.setTextColor(ILI9341_BLUE);
      tft2.setCursor(175, 10);
      tft2.print("NEW DATA");
      state_data = true;
    }

    printTime(65, 20, 2);
    printDate(130, 20, 2);
    printPitch(50, 225, 1);
    printRoll(50, 235, 1);
    printGpsCourse(75, 245, 1);


    //serialOutput();

    last_smoothHeadingDegrees = smoothHeadingDegrees;
    last_pitch = pitch;
    last_roll = roll;
    last_temperature = temperature;
    last_fix_data = fix_data;
    last_latitude = latitude;
    last_longtitude = longtitude;
    last_fix_age = fix_age;
    last_gps_month = gps_month, last_gps_day = gps_day, last_gps_year = gps_year;
    last_time_hour = time_hour, last_time_minute = time_minute;
    last_alt = alt;
    last_speed_kmph = speed_kmph;
    last_distanceToLondon = distanceToLondon;
    last_start_longtitude = start_longtitude;
    last_start_latitude = start_latitude;
    last_courseToHome = courseToHome;
    last_but = but;
    last_arm = arm;
    last_distanceToHome = distanceToHome;
    last_bmp_temperature = bmp_temperature;
    last_bmp_pressure = bmp_pressure;
    last_bmp_altitude = bmp_altitude;
    last_gps_course = gps_course;
    last_courseToNavigate = courseToNavigate;
    last_LONGTITUDE = LONGTITUDE;
    last_LATITUDE = LATITUDE;
    last_distanceToPoint = distanceToPoint;

  }
  else  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      if (state_data == true)  {
        tft2.setTextSize(1);
        tft2.setTextColor(ILI9341_WHITE);
        tft2.setCursor(175, 10);
        tft2.print("NEW_DATA");
        tft2.setTextColor(ILI9341_RED);
        tft2.setCursor(175, 10);
        tft2.print("MEMORY");
        state_data = false;
      }
    }
  }
}
void updateCompass()  {
  if (last_smoothHeadingDegrees != smoothHeadingDegrees) {
    if (course_offset >= 0)  {
      if (last_smoothHeadingDegrees + course_offset < 359)  {
        last_smoothHeadingDegrees += course_offset;
      }
      else  {
        last_smoothHeadingDegrees = last_smoothHeadingDegrees + course_offset - 359;
      }
    }
    else  {
      if (last_smoothHeadingDegrees + course_offset <= 0) {
        last_smoothHeadingDegrees = 359 + (last_smoothHeadingDegrees + course_offset);
      }
      else  {
        last_smoothHeadingDegrees += course_offset;
      }
    }
    if (course_offset >= 0)  {
      if (smoothHeadingDegrees + course_offset < 359)  {
        smoothHeadingDegrees += course_offset;
      }
      else  {
        smoothHeadingDegrees = smoothHeadingDegrees + course_offset - 359;
      }
    }
    else  {
      if (smoothHeadingDegrees + course_offset <= 0) {
        smoothHeadingDegrees = 359 + (smoothHeadingDegrees + course_offset);
      }
      else  {
        smoothHeadingDegrees += course_offset;
      }
    }
    drawCourseToHome(courseToHome, 0);
    drawCompass(smoothHeadingDegrees, 0);
    if (navigate == true)  {
      courseToNavigate = TinyGPSPlus::courseTo((double)latitude, (double)longtitude, (double)LATITUDE, (double)LONGTITUDE);
      distanceToPoint =  TinyGPSPlus::distanceBetween((double)latitude, (double)longtitude, (double)LATITUDE, (double)LONGTITUDE);
      drawCourseToNavigate(courseToNavigate, 0);
      printToPoint(5, 5, 2);
      printCourseToPoint(5,25,2);
    }

  }
}
void updateCourseToHome() {
  if (last_courseToHome != courseToHome) {


  }
}
void  drawCourseToNavigate(int roll, int pitch)  {

  roll += 90;
  roll = map(roll, 0, 360, -180, 180);
  float sx = cos(roll * DEG2RAD);
  float sy = sin(roll * DEG2RAD);
  int16_t x0 = sx * 60;
  int16_t y0 = sy * 60;
  int16_t xd = 0;
  int16_t yd = 1;
  int16_t xdn  = 0;
  int16_t ydn = 0;
  if (roll > 45 && roll <  135) {
    xd = -1;
    yd =  0;
  }
  if (roll >=  135)             {
    xd =  0;
    yd = -1;
  }
  if (roll < -45 && roll > -135) {
    xd =  1;
    yd =  0;
  }
  if (roll <= -135)             {
    xd =  0;
    yd = -1;
  }
  tft.setRotation(0);
  if ((roll != last_roll) && ((abs(roll) > 35)  || (pitch != last_pitch)))
  {
    //tft.fillCircle(120, 160, 95, ILI9341_WHITE);
    //tft.fillCircle(120, 160, 84, ILI9341_WHITE);
    //tft.drawLine(XC - x0, YC - y0 - pitch,   XC + x0, YC + y0 - pitch,   ILI9341_RED);
    int offset = 0;
    tft.drawLine(XC - x0 + offset, YC - y0 - pitch + offset,   XC + x0 + offset, YC + y0 - pitch + offset,   ILI9341_GREEN);
    tft.drawLine(XC - x0 - offset, YC - y0 - pitch - offset,   XC + x0 - offset, YC + y0 - pitch - offset,   ILI9341_GREEN);
    tft.fillCircle(XC + x0, YC + y0 - pitch, 3, ILI9341_GREEN);
  }
}
void  drawCourseToHome(int roll, int pitch)  {

  roll += 90;
  roll = map(roll, 0, 360, -180, 180);
  float sx = cos(roll * DEG2RAD);
  float sy = sin(roll * DEG2RAD);
  int16_t x0 = sx * 60;
  int16_t y0 = sy * 60;
  int16_t xd = 0;
  int16_t yd = 1;
  int16_t xdn  = 0;
  int16_t ydn = 0;
  if (roll > 45 && roll <  135) {
    xd = -1;
    yd =  0;
  }
  if (roll >=  135)             {
    xd =  0;
    yd = -1;
  }
  if (roll < -45 && roll > -135) {
    xd =  1;
    yd =  0;
  }
  if (roll <= -135)             {
    xd =  0;
    yd = -1;
  }
  tft.setRotation(0);
  if ((roll != last_roll) && ((abs(roll) > 35)  || (pitch != last_pitch)))
  {
    //tft.fillCircle(120, 160, 95, ILI9341_WHITE);
    //tft.fillCircle(120, 160, 84, ILI9341_WHITE);
    //tft.drawLine(XC - x0, YC - y0 - pitch,   XC + x0, YC + y0 - pitch,   ILI9341_RED);
    int offset = 0;
    tft.drawLine(XC - x0 + offset, YC - y0 - pitch + offset,   XC + x0 + offset, YC + y0 - pitch + offset,   ILI9341_BLUE);
    tft.drawLine(XC - x0 - offset, YC - y0 - pitch - offset,   XC + x0 - offset, YC + y0 - pitch - offset,   ILI9341_BLUE);
    tft.fillCircle(XC + x0, YC + y0 - pitch, 3, ILI9341_BLUE);
  }
}
void drawCompassInfo() {
  tft.fillCircle(120, 160, 85, ILI9341_WHITE);
  drawCompassDATA(0, 0);
  drawCompassDATA(10, 0);
  drawCompassDATA(20, 0);
  drawCompassDATA(30, 0);
  drawCompassDATA(40, 0);
  drawCompassDATA(50, 0);
  drawCompassDATA(60, 0);
  drawCompassDATA(70, 0);
  drawCompassDATA(80, 0);
  drawCompassDATA(90, 0);
  drawCompassDATA(180, 0);
  drawCompassDATA(45, 0);
  drawCompassDATA45(315, 0);
  drawCompassDATA45(45, 0);
  drawCompassDATA(350, 0);
  drawCompassDATA(340, 0);
  drawCompassDATA(330, 0);
  drawCompassDATA(320, 0);
  drawCompassDATA(310, 0);
  drawCompassDATA(300, 0);
  drawCompassDATA(290, 0);
  drawCompassDATA(280, 0);
  drawCompassDATA(270, 0);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(115, 8);
  tft.print("N");
  tft.setCursor(115, 290);
  tft.print("S");
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_RED);
  tft.setCursor(112, 260);
  tft.print("180");
  tft.setCursor(112, 52);
  tft.print(" 0 ");
  tft.setCursor(218, 157);
  tft.print("90");
  tft.setCursor(10, 157);
  tft.print("270");

}
void drawCompassDATA45(int roll, int pitch)  {
  roll += 90;
  roll = map(roll, 0, 360, -180, 180);
  float sx = cos(roll * DEG2RAD);
  float sy = sin(roll * DEG2RAD);
  int16_t x0 = sx * 110;
  int16_t y0 = sy * 110;
  int16_t xd = 0;
  int16_t yd = 1;
  int16_t xdn  = 0;
  int16_t ydn = 0;
  if (roll > 45 && roll <  135) {
    xd = -1;
    yd =  0;
  }
  if (roll >=  135)             {
    xd =  0;
    yd = -1;
  }
  if (roll < -45 && roll > -135) {
    xd =  1;
    yd =  0;
  }
  if (roll <= -135)             {
    xd =  0;
    yd = -1;
  }
  tft.setRotation(0);



  tft.drawLine(XC - x0, YC - y0 - pitch,   XC + x0, YC + y0 - pitch,   ILI9341_ORANGE);
  //tft.drawLine(XC - x0+1, YC - y0 - pitch + 1,   XC + x0+1, YC + y0 - pitch + 1,   ILI9341_RED);
  //tft.drawLine(XC - x0-1, YC - y0 - pitch - 1,   XC + x0-1, YC + y0 - pitch - 1,   ILI9341_RED);

}
void drawCompassDATA(int roll, int pitch)  {
  roll += 90;
  roll = map(roll, 0, 360, -180, 180);
  float sx = cos(roll * DEG2RAD);
  float sy = sin(roll * DEG2RAD);
  int16_t x0 = sx * 95;
  int16_t y0 = sy * 95;
  int16_t xd = 0;
  int16_t yd = 1;
  int16_t xdn  = 0;
  int16_t ydn = 0;
  if (roll > 45 && roll <  135) {
    xd = -1;
    yd =  0;
  }
  if (roll >=  135)             {
    xd =  0;
    yd = -1;
  }
  if (roll < -45 && roll > -135) {
    xd =  1;
    yd =  0;
  }
  if (roll <= -135)             {
    xd =  0;
    yd = -1;
  }
  tft.setRotation(0);



  tft.drawLine(XC - x0, YC - y0 - pitch,   XC + x0, YC + y0 - pitch,   ILI9341_BLACK);
  //tft.drawLine(XC - x0+1, YC - y0 - pitch + 1,   XC + x0+1, YC + y0 - pitch + 1,   ILI9341_RED);
  //tft.drawLine(XC - x0-1, YC - y0 - pitch - 1,   XC + x0-1, YC + y0 - pitch - 1,   ILI9341_RED);

}
void drawCompass(int roll, int pitch) {
  roll += 90;
  roll = map(roll, 0, 360, -180, 180);
  float sx = cos(roll * DEG2RAD);
  float sy = sin(roll * DEG2RAD);
  int16_t x0 = sx * 80;
  int16_t y0 = sy * 80;
  int16_t xd = 0;
  int16_t yd = 1;
  int16_t xdn  = 0;
  int16_t ydn = 0;
  if (roll > 45 && roll <  135) {
    xd = -1;
    yd =  0;
  }
  if (roll >=  135)             {
    xd =  0;
    yd = -1;
  }
  if (roll < -45 && roll > -135) {
    xd =  1;
    yd =  0;
  }
  if (roll <= -135)             {
    xd =  0;
    yd = -1;
  }
  tft.setRotation(0);
  if ((roll != last_roll) && ((abs(roll) > 35)  || (pitch != last_pitch)))
  {
    tft.fillCircle(120, 160, 84, ILI9341_WHITE);

    tft.drawLine(XC - x0, YC - y0 - pitch,   XC + x0, YC + y0 - pitch,   ILI9341_RED);
    tft.fillCircle(XC + x0, YC + y0 - pitch, 3, ILI9341_RED);
    //tft.fillTriangle(XC + x0, YC + y0 - pitch,XC + x0-10, YC + y0 - pitch+5,XC + x0-10, YC + y0 - pitch-5,ILI9341_RED);
    //tft.drawLine(XC - x0+1, YC - y0 - pitch + 1,   XC + x0+1, YC + y0 - pitch + 1,   ILI9341_RED);
    //tft.drawLine(XC - x0-1, YC - y0 - pitch - 1,   XC + x0-1, YC + y0 - pitch - 1,   ILI9341_RED);
  }

}
void drawHorizon(int roll, int pitch) {
  roll += roll_offset;
  pitch += pitch_offset;
  float sx = cos(roll * DEG2RAD);
  float sy = sin(roll * DEG2RAD);
  int16_t x0 = sx * HOR;
  int16_t y0 = sy * HOR;
  int16_t xd = 0;
  int16_t yd = 1;
  int16_t xdn  = 0;
  int16_t ydn = 0;
  if (roll > 45 && roll <  135) {
    xd = -1;
    yd =  0;
  }
  if (roll >=  135)             {
    xd =  0;
    yd = -1;
  }
  if (roll < -45 && roll > -135) {
    xd =  1;
    yd =  0;
  }
  if (roll <= -135)             {
    xd =  0;
    yd = -1;
  }
  tft.setRotation(0);
  if ((roll != last_roll) && ((abs(roll) > 35)  || (pitch != last_pitch)))
  {
    xdn = 1 * xd;
    ydn = 1 * yd;
    tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
    tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  }
  if (roll == 0 and pitch == 0 /*and last_pitch != 0 and last_roll != 0*/)  {
    tft.setRotation(0);
    tft.fillRect(0,  0, 240, 160, ILI9341_BLUE);
    tft.fillRect(0, 160, 240, 160, ILI9341_BROWN);
  }
  if (pitch  > 20) {
    tft.fillTriangle(115, 290, 125, 290, 120, 310, ILI9341_RED);
    tft.drawLine(120, 260, 120, 290, ILI9341_RED);
    tft.drawLine(119, 260, 119, 290, ILI9341_RED);
    tft.drawLine(121, 260, 121, 290, ILI9341_RED);
  }
  else  {
    tft.fillTriangle(115, 290, 125, 290, 120, 310, ILI9341_BROWN);
    tft.drawLine(120, 260, 120, 290, ILI9341_BROWN);
    tft.drawLine(119, 260, 119, 290, ILI9341_BROWN);
    tft.drawLine(121, 260, 121, 290, ILI9341_BROWN);
  }
  if (pitch < -20) {
    tft.fillTriangle(115, 30, 125, 30, 120, 10, ILI9341_RED);
    tft.drawLine(120, 60, 120, 30, ILI9341_RED);
    tft.drawLine(119, 60, 119, 30, ILI9341_RED);
    tft.drawLine(121, 60, 121, 30, ILI9341_RED);
  }
  else  {
    tft.fillTriangle(115, 30, 125, 30, 120, 10, ILI9341_BLUE);
    tft.drawLine(120, 60, 120, 30, ILI9341_BLUE);
    tft.drawLine(119, 60, 119, 30, ILI9341_BLUE);
    tft.drawLine(121, 60, 121, 30, ILI9341_BLUE);
  }

  if (roll > 20 and pitch < 0) {
    tft.fillTriangle(70, 40, 100, 35, 100, 45, ILI9341_RED);
    tft.drawLine(118, 40, 100, 40, ILI9341_RED);
    tft.drawLine(118, 41, 100, 41, ILI9341_RED);
    tft.drawLine(118, 39, 100, 39, ILI9341_RED);
  }
  else  {
    tft.fillTriangle(70, 40, 100, 35, 100, 45, ILI9341_BLUE);
    tft.drawLine(118, 40, 100, 40, ILI9341_BLUE);
    tft.drawLine(118, 41, 100, 41, ILI9341_BLUE);
    tft.drawLine(118, 39, 100, 39, ILI9341_BLUE);
  }
  if (roll < -20 and pitch < 0)  {
    tft.fillTriangle(170, 40, 140, 35, 140, 45, ILI9341_RED);
    tft.drawLine(122, 40, 140, 40, ILI9341_RED);
    tft.drawLine(122, 41, 140, 41, ILI9341_RED);
    tft.drawLine(122, 39, 140, 39, ILI9341_RED);
  }
  else  {
    tft.fillTriangle(170, 40, 140, 35, 140, 45, ILI9341_BLUE);
    tft.drawLine(122, 40, 140, 40, ILI9341_BLUE);
    tft.drawLine(122, 41, 140, 41, ILI9341_BLUE);
    tft.drawLine(122, 39, 140, 39, ILI9341_BLUE);
  }

  if (roll > 20 and pitch > 0) {
    tft.fillTriangle(70, 280, 100, 275, 100, 285, ILI9341_RED);
    tft.drawLine(118, 280, 100, 280, ILI9341_RED);
    tft.drawLine(118, 281, 100, 281, ILI9341_RED);
    tft.drawLine(118, 279, 100, 279, ILI9341_RED);
  }
  else  {
    tft.fillTriangle(70, 280, 100, 275, 100, 285, ILI9341_BROWN);
    tft.drawLine(118, 280, 100, 280, ILI9341_BROWN);
    tft.drawLine(118, 281, 100, 281, ILI9341_BROWN);
    tft.drawLine(118, 279, 100, 279, ILI9341_BROWN);
  }
  if (roll < -20 and pitch > 0)  {
    tft.fillTriangle(170, 280, 140, 275, 140, 285, ILI9341_RED);
    tft.drawLine(122, 280, 140, 280, ILI9341_RED);
    tft.drawLine(122, 281, 140, 281, ILI9341_RED);
    tft.drawLine(122, 279, 140, 279, ILI9341_RED);
  }
  else  {
    tft.fillTriangle(170, 280, 140, 275, 140, 285, ILI9341_BROWN);
    tft.drawLine(122, 280, 140, 280, ILI9341_BROWN);
    tft.drawLine(122, 281, 140, 281, ILI9341_BROWN);
    tft.drawLine(122, 279, 140, 279, ILI9341_BROWN);
  }
  drawInfo();
  xdn = 24 * xd;
  ydn = 24 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 23 * xd;
  ydn = 23 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 22 * xd;
  ydn = 22 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 21 * xd;
  ydn = 21 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 20 * xd;
  ydn = 20 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 19 * xd;
  ydn = 19 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 18 * xd;
  ydn = 18 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 17 * xd;
  ydn = 17 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 16 * xd;
  ydn = 16 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 15 * xd;
  ydn = 15 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 14 * xd;
  ydn = 14 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 13 * xd;
  ydn = 13 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 12 * xd;
  ydn = 12 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 11 * xd;
  ydn = 11 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 10 * xd;
  ydn = 10 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 9 * xd;
  ydn = 9 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 8 * xd;
  ydn = 8 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 7 * xd;
  ydn = 7 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 6 * xd;
  ydn = 6 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 5 * xd;
  ydn = 5 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 4 * xd;
  ydn = 4 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 3 * xd;
  ydn = 3 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 2 * xd;
  ydn = 2 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);
  xdn = 1 * xd;
  ydn = 1 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, ILI9341_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, ILI9341_BROWN);

  tft.drawLine(XC - x0, YC - y0 - pitch,   XC + x0, YC + y0 - pitch,   ILI9341_WHITE);
  /*last_roll = roll;
    last_pitch = pitch;*/
  drawInfo();
}
void drawInfo(void) {
  tft.setTextSize(1);
  tft.fillRect(120 - 1, 160 - 1, 3, 3, ILI9341_RED);
  tft.drawFastHLine(120 - 30,   160, 24, ILI9341_RED);
  tft.drawFastHLine(120 + 30 - 24, 160, 24, ILI9341_RED);
  tft.drawFastVLine(120 - 30 + 24, 160, 3, ILI9341_RED);
  tft.drawFastVLine(120 + 30 - 24, 160, 3, ILI9341_RED);

  tft.drawFastHLine(120 - 12,   160 - 60, 24, ILI9341_WHITE);
  tft.drawFastHLine(120 - 6,   160 - 50, 12, ILI9341_WHITE);
  tft.drawFastHLine(120 - 12,   160 - 40, 24, ILI9341_WHITE);
  tft.drawFastHLine(120 -  6,   160 - 30, 12, ILI9341_WHITE);
  tft.drawFastHLine(120 - 12,   160 - 20, 24, ILI9341_WHITE);
  tft.drawFastHLine(120 -  6,   160 - 10, 12, ILI9341_WHITE);

  tft.drawFastHLine(120 -  6,   160 + 10, 12, ILI9341_WHITE);
  tft.drawFastHLine(120 - 12,   160 + 20, 24, ILI9341_WHITE);
  tft.drawFastHLine(120 -  6,   160 + 30, 12, ILI9341_WHITE);
  tft.drawFastHLine(120 - 12,   160 + 40, 24, ILI9341_WHITE);
  tft.drawFastHLine(120 - 6,   160 + 50, 12, ILI9341_WHITE);
  tft.drawFastHLine(120 - 12,   160 + 60, 24, ILI9341_WHITE);

  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(120 - 12 - 13, 160 - 20 - 3);
  tft.print("20");
  tft.setCursor(120 + 12 + 1, 160 - 20 - 3);
  tft.print("20");
  tft.setCursor(120 - 12 - 13, 160 + 20 - 3);
  tft.print("20");
  tft.setCursor(120 + 12 + 1, 160 + 20 - 3);
  tft.print("20");

  tft.setCursor(120 - 12 - 13, 160 - 40 - 3);
  tft.print("40");
  tft.setCursor(120 + 12 + 1, 160 - 40 - 3);
  tft.print("40");
  tft.setCursor(120 - 12 - 13, 160 + 40 - 3);
  tft.print("40");
  tft.setCursor(120 + 12 + 1, 160 + 40 - 3);
  tft.print("40");

  tft.setCursor(120 - 12 - 13, 160 - 60 - 3);
  tft.print("60");
  tft.setCursor(120 + 12 + 1, 160 - 60 - 3);
  tft.print("60");
  tft.setCursor(120 - 12 - 13, 160 + 60 - 3);
  tft.print("60");
  tft.setCursor(120 + 12 + 1, 160 + 60 - 3);
  tft.print("60");
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BROWN);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(2);
}

void printTime(int x, int y, int size)  {
  long last_millis_printTime = 0;
  long interval = 1;
  unsigned long currentmillis = millis();
  if (last_time_hour != time_hour or
      last_time_minute != time_minute and time_minute < 60 and time_minute > 0 and currentmillis - last_millis_printTime > interval)  {
    last_millis_printTime = currentmillis;
    tft2.fillRect(x, y, 55, 15, ILI9341_WHITE);
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_WHITE);
    tft2.setCursor(x, y);
    tft2.print(last_time_hour);
    tft2.setCursor(x + 20, y);
    tft2.print(":");
    //tft2.fillRect(x+30,y, 90, 15,ILI9341_WHITE);
    tft2.setCursor(x + 30, y);
    tft2.print(last_time_minute);

    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_BLACK);
    tft2.setCursor(x, y);
    tft2.print(time_hour);
    tft2.setCursor(x + 20, y);
    tft2.print(":");
    tft2.setCursor(x + 30, y);
    tft2.print(time_minute);
  }
}
void printDate(int x, int y, int size)  {
  long interval = 1000;
  unsigned long currentmillis = millis();
  if (last_gps_month != gps_month or
      last_gps_day != gps_day or
      last_gps_year != gps_year
      or currentmillis - last_millis_printDate > interval) {
    last_millis_printDate = currentmillis;
    tft2.fillRect(x, y, 55, 15, ILI9341_WHITE);
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_WHITE);
    tft2.setCursor(x, y);
    tft2.print(last_gps_day);
    tft2.setCursor(x + 20, y);
    tft2.print(".");
    tft2.setCursor(x + 30, y);
    tft2.print(last_gps_month);
    tft2.setCursor(x + 37, y);
    tft2.print(".");
    tft2.setCursor(x + 50, y);
    tft2.print(last_gps_year);

    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_BLACK);
    tft2.setCursor(x, y);
    tft2.print(gps_day);
    tft2.setCursor(x + 20, y);
    tft2.print(".");
    tft2.setCursor(x + 30, y);
    tft2.print(gps_month);
    tft2.setCursor(x + 37, y);
    tft2.print(".");
    tft2.setCursor(x + 50, y);
    tft2.print(gps_year);
    count++;
  }
}
void printFixData (int x, int y, int size) {
  if (last_fix_data != fix_data)  {
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_WHITE);
    tft2.setCursor(x, y);
    tft2.print(last_fix_data);
    tft2.setTextSize(1);
    tft2.setTextColor(ILI9341_BLACK);
    tft2.setCursor(x, y);
    tft2.print(fix_data);
  }
}
void printLatitude(int x, int y, int size)  {
  if (last_latitude != latitude) {
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_WHITE);
    tft2.setCursor(x, y);
    tft2.print(last_latitude, 6);
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_BLACK);
    tft2.setCursor(x, y);
    tft2.print(latitude, 6);
    //tft2.setCursor(210, 55);
    //tft2.print("'");
    if (latitude > 0 && i == 0)  {
      start_latitude = latitude;
      i++;
    }
  }
}
void printLongtitude(int x, int y, int size) {
  if (last_longtitude != longtitude) {
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_WHITE);
    tft2.setCursor(x, y);
    tft2.print(last_longtitude, 6);
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_BLACK);
    tft2.setCursor(x, y);
    tft2.print(longtitude, 6);
    //tft2.setCursor(210, 75);
    //tft2.print("'");
    if (longtitude > 0 && o == 0)  {
      start_longtitude = longtitude;
      o++;
    }
  }
}
void printTemperature(int x, int y, int size) {
  if (last_temperature != temperature) {
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_WHITE);
    tft2.setCursor(x, y);
    tft2.print(last_temperature);
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_BLACK);
    tft2.setCursor(x, y);
    tft2.print(temperature);
    tft2.setCursor(x + 60, y);
    tft2.print("'C");
  }
}
void printFixAge(int x, int y, int size) {
  if (last_fix_age != fix_age) {
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_WHITE);
    tft2.setCursor(x, y);
    tft2.print(last_fix_age);
    tft2.setTextSize(1);
    tft2.setTextColor(ILI9341_BLACK);
    tft2.setCursor(x, y);
    tft2.print(fix_age);
  }
}
void printSpeed(int x, int y, int size) {
  if (last_speed_kmph != speed_kmph and speed_kmph < 200 and speed_kmph > 0) {
    tft2.fillRect(x, y, 90, 15, ILI9341_WHITE);
    /*tft2.setTextSize(size);
      tft2.setTextColor(ILI9341_WHITE);
      tft2.setCursor(x, y);
      tft2.print(last_speed_kmph);*/
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_BLACK);
    tft2.setCursor(x, y);
    tft2.print((int)speed_kmph);
  }
}
void printStartLatLon(int x, int y, int size) {
  if (last_start_latitude != start_latitude) {
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_BLACK);
    tft2.setCursor(x, y);
    tft2.print(start_latitude);
    tft2.setCursor(x + 100, y);
    tft2.print(start_longtitude);
  }
}
void printHeading(int x, int y, int size)  {
  if (last_smoothHeadingDegrees != smoothHeadingDegrees) {
    tft2.fillRect(x, y, 50, 15, ILI9341_WHITE);
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_WHITE);
    tft2.setCursor(x, y);
    if (course_offset >= 0)  {
      if (last_smoothHeadingDegrees + course_offset < 359)  {
        last_smoothHeadingDegrees += course_offset;
      }
      else  {
        last_smoothHeadingDegrees = last_smoothHeadingDegrees + course_offset - 359;
      }
    }
    else  {
      if (last_smoothHeadingDegrees + course_offset <= 0) {
        last_smoothHeadingDegrees = 359 + (last_smoothHeadingDegrees + course_offset);
      }
      else  {
        last_smoothHeadingDegrees += course_offset;
      }
    }
    tft2.print(last_smoothHeadingDegrees);
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_BLACK);
    tft2.setCursor(x, y);
    if (course_offset >= 0)  {
      if (smoothHeadingDegrees + course_offset < 359)  {
        smoothHeadingDegrees += course_offset;
      }
      else  {
        smoothHeadingDegrees = smoothHeadingDegrees + course_offset - 359;
      }
    }
    else  {
      if (smoothHeadingDegrees + course_offset <= 0) {
        smoothHeadingDegrees = 359 + (smoothHeadingDegrees + course_offset);
      }
      else  {
        smoothHeadingDegrees += course_offset;
      }
    }
    tft2.print(smoothHeadingDegrees);
  }
}
void printCourseToHome(int x, int y, int size)  {
  if (last_courseToHome != courseToHome)  {
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_WHITE);
    tft2.setCursor(x, y);
    tft2.print((int)last_courseToHome);
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_BLACK);
    tft2.setCursor(x, y);
    tft2.print((int)courseToHome);
  }
}
void printGpsCourse(int x, int y, int size) {
  if (last_pitch != pitch)  {
    tft2.fillRect(x, y, 30 , 8, ILI9341_WHITE);
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_WHITE);
    tft2.setCursor(x, y);
    tft2.print(last_gps_course);
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_BLACK);
    tft2.setCursor(x, y);
    tft2.print(gps_course);
  }
}
void printPitch(int x, int y, int size) {
  if (last_pitch != pitch)  {
    tft2.fillRect(x, y, 50, 8, ILI9341_WHITE);
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_WHITE);
    tft2.setCursor(x, y);
    tft2.print(last_pitch + pitch_offset);
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_BLACK);
    tft2.setCursor(x, y);
    tft2.print(pitch + pitch_offset);
  }
}
void printRoll(int x, int y, int size)  {
  if (last_roll != roll)  {
    tft2.fillRect(x, y, 50, 8, ILI9341_WHITE);
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_WHITE);
    tft2.setCursor(x, y);
    tft2.print(last_roll + roll_offset);
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_BLACK);
    tft2.setCursor(x, y);
    tft2.print(roll + roll_offset);
  }
}
void printDistanceToHome(int x, int y, int size)  {
  if (last_distanceToHome != distanceToHome and distanceToHome < 10000)  {
    tft2.fillRect(x, y, 100, 15, ILI9341_WHITE);
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_WHITE);
    tft2.setCursor(x, y);
    tft2.print(last_distanceToHome, 4);
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_BLACK);
    tft2.setCursor(x, y);
    tft2.print(distanceToHome, 4);

  }
}
void printBut(int x, int y, int size) {
  if (last_but != but) {
    tft2.fillRect(x, y, 6, 8, ILI9341_WHITE);
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_WHITE);
    tft2.setCursor(x, y);
    tft2.print(last_but);
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_BLACK);
    tft2.setCursor(x, y);
    tft2.print(but);
  }
}
void printArmState(int x, int y, int size)  {
  if (last_arm != arm) {
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_WHITE);
    tft2.setCursor(x, y);
    tft2.print(last_arm);
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_BLACK);
    tft2.setCursor(x, y);
    tft2.print(arm);
  }
}
void printAltitude(int x, int y, int size)  {
  if (last_bmp_altitude != bmp_altitude and bmp_altitude > 0) {
    tft2.fillRect(x, y, 100, 15, ILI9341_WHITE);
    /*tft2.setTextSize(size);
      tft2.setTextColor(ILI9341_WHITE);
      tft2.setCursor(x,y);
      tft2.print(last_bmp_altitude);*/
    tft2.setTextSize(size);
    tft2.setTextColor(ILI9341_BLACK);
    tft2.setCursor(x, y);
    tft2.print((int)bmp_altitude + (int)altitude_offset);
  }
}
void updateHorizont() {
  if (last_roll != roll and (abs(roll) - abs(last_roll)) < 5)  {
    drawHorizon(roll, pitch);
    drawInfo();
  }
  if (last_pitch != pitch and (abs(pitch) - abs(last_pitch)) < 5)  {
    drawHorizon(roll, pitch);
    drawInfo();
  }
}
void serialOutput() {
  Serial.print("<DATA,");
  Serial.print(smoothHeadingDegrees);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  Serial.print(temperature);
  Serial.print(",");
  Serial.print(fix_data);
  Serial.print(",");
  Serial.print(latitude, 6);
  Serial.print(",");
  Serial.print(longtitude, 6);
  Serial.print(",");
  Serial.print(gps_month);
  Serial.print(",");
  Serial.print(gps_day);
  Serial.print(",");
  Serial.print(gps_year);
  Serial.print(",");
  Serial.print(time_hour);
  Serial.print(",");
  Serial.print(time_minute);
  Serial.print(",");
  Serial.print(fix_age);
  Serial.print(",");
  Serial.print(speed_kmph);
  Serial.print(",");
  Serial.print(bmp_altitude);
  Serial.println(">");

}
void printTouchBut()  {
  tft2.setTextSize(1);
  tft2.setTextColor(ILI9341_BLACK);
  tft2.setCursor(15, 300);
  tft2.print("FOTO");
  tft2.setCursor(135, 300);
  tft2.print("ZOOM +");
  tft2.setCursor(195, 300);
  tft2.print("ZOOM -");
}
void printDataText()  {
  tft2.setTextSize(2);
  tft2.setTextColor(ILI9341_BLACK);
  tft2.setCursor(5, 20);
  tft2.print("Time:");
  tft2.setTextSize(1);
  tft2.setCursor(5, 40);
  tft2.print("GPS fix data:");
  tft2.setCursor(135, 40);
  tft2.print("Fix age:");
  tft2.setTextSize(2);
  tft2.setCursor(5, 55);
  tft2.print("Latitude:");
  tft2.setCursor(5, 75);
  tft2.print("Longtitude:");
  tft2.setTextSize(1);
  tft2.setCursor(5, 95);
  tft2.print("Start Latitude: ");
  tft2.setCursor(130, 95);
  tft2.print("Longtitude: ");

  tft2.setTextSize(2);
  tft2.setCursor(5, 105);
  tft2.print("Course to home: ");
  tft2.setTextSize(2);
  tft2.setCursor(5, 125);
  tft2.print("Current course: ");

  tft2.setTextSize(2);
  tft2.setCursor(5, 145);
  tft2.print("Temperature: ");
  tft2.setCursor(5, 165);
  tft2.print("Speed(kmph): ");
  tft2.setCursor(5, 225);
  tft2.setTextSize(1);
  tft2.print("Pitch:");
  tft2.setCursor(5, 235);
  tft2.print("Roll:");
  tft2.setTextSize(2);
  tft2.setCursor(5, 185);
  tft2.print("HOME(m):");
  tft2.setCursor(5, 205);
  tft2.print("Altitude:");
  tft2.setTextSize(1);
  tft2.setCursor(5, 270);
  tft2.print("Last but pressed:");
  tft2.setTextColor(ILI9341_BLACK);
  tft2.setTextSize(1);
  tft2.setCursor(5, 245);
  tft2.print("GPS course:");

  tft2.setTextSize(1);
  tft2.setCursor(5, 260);
  tft2.print("Arm key state:");
}

void printLATLOT()  {
  tft2.setTextColor(ILI9341_BLACK);
  tft2.setTextSize(1);
  tft2.setCursor(35, 182);
  tft2.print("LATITUDE");
  tft2.setCursor(40 + 110, 182);
  tft2.print("LONGTITUDE");
  tft2.setCursor(20, 182 - 20);
  tft2.print("6");
  tft2.setCursor(20 + 48, 182 - 20);
  tft2.print("7");
  tft2.setCursor(20 + 48 * 2, 182 - 20);
  tft2.print("8");
  tft2.setCursor(20 + 48 * 3, 182 - 20);
  tft2.print("9");
  tft2.setCursor(20 + 48 * 4, 182 - 20);
  tft2.print("-");
  tft2.setCursor(20, 182 - 40);
  tft2.print("1");
  tft2.setCursor(20 + 48, 182 - 40);
  tft2.print("2");
  tft2.setCursor(20 + 48 * 2, 182 - 40);
  tft2.print("3");
  tft2.setCursor(20 + 48 * 3, 182 - 40);
  tft2.print("4");
  tft2.setCursor(20 + 48 * 4, 182 - 40);
  tft2.print("5");
  tft2.setCursor(20, 182 - 60);
  tft2.print("C");
  tft2.setCursor(15 + 48, 182 - 60);
  tft2.print("<-");
  tft2.setCursor(20 + 48 * 2, 182 - 60);
  tft2.print(".");
  tft2.setCursor(20 + 48 * 3, 182 - 60);
  tft2.print("0");
  tft2.setCursor(10 + 48 * 4, 182 - 60);
  tft2.print("ENTER");

  tft2.setRotation(1);
  tft2.drawLine(251 + 5 - 61, 240, 251 + 5 - 61, 0, ILI9341_BLACK);
  tft2.drawLine(251 + 5 - 61 + 1, 240, 251 + 5 - 61 + 1, 0, ILI9341_BLACK);
  tft2.drawLine(251 + 5 - 61 - 1, 240, 251 + 5 - 61 - 1, 0, ILI9341_BLACK);
  tft2.drawLine(251 + 5 - 61 - 80, 240, 251 + 5 - 61 - 80, 0, ILI9341_BLACK);
  tft2.drawLine(251 + 5 - 61 + 1 - 80, 240, 251 + 5 - 61 + 1 - 80, 0, ILI9341_BLACK);
  tft2.drawLine(251 + 5 - 61 - 1 - 80, 240, 251 + 5 - 61 - 1 - 80, 0, ILI9341_BLACK);
  tft2.drawLine(251 + 5 - 61 - 20, 240, 251 + 5 - 61 - 20, 0, ILI9341_BLACK);
  tft2.drawLine(251 + 5 - 61 + 1 - 20, 240, 251 + 5 - 61 + 1 - 20, 0, ILI9341_BLACK);
  tft2.drawLine(251 + 5 - 61 - 1 - 20, 240, 251 + 5 - 61 - 1 - 20, 0, ILI9341_BLACK);
  tft2.drawLine(251 + 5 - 61 - 40, 240, 251 + 5 - 61 - 40, 0, ILI9341_BLACK);
  tft2.drawLine(251 + 5 - 61 + 1 - 40, 240, 251 + 5 - 61 + 1 - 40, 0, ILI9341_BLACK);
  tft2.drawLine(251 + 5 - 61 - 1 - 40, 240, 251 + 5 - 61 - 1 - 40, 0, ILI9341_BLACK);
  tft2.drawLine(251 + 5 - 61 - 60, 240, 251 + 5 - 61 - 60, 0, ILI9341_BLACK);
  tft2.drawLine(251 + 5 - 61 + 1 - 60, 240, 251 + 5 - 61 + 1 - 60, 0, ILI9341_BLACK);
  tft2.drawLine(251 + 5 - 61 - 1 - 60, 240, 251 + 5 - 61 - 1 - 60, 0, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 20, 120, 249 + 5 - 60, 120, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 20, 121, 249 + 5 - 60, 121, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 20, 119, 249 + 5 - 60, 119, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48, 249 + 5 - 60 - 20, 48, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 47, 249 + 5 - 60 - 20, 47, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 49, 249 + 5 - 60 - 20, 49, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 2, 249 + 5 - 60 - 20, 48 * 2, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 2 - 1, 249 + 5 - 60 - 20, 48 * 2 - 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 2 + 1, 249 + 5 - 60 - 20, 48 * 2 + 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 3, 249 + 5 - 60 - 20, 48 * 3, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 3 - 1, 249 + 5 - 60 - 20, 48 * 3 - 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 3 + 1, 249 + 5 - 60 - 20, 48 * 3 + 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 4, 249 + 5 - 60 - 20, 48 * 4, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 4 - 1, 249 + 5 - 60 - 20, 48 * 4 - 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 4 + 1, 249 + 5 - 60 - 20, 48 * 4 + 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 5, 249 + 5 - 60 - 20, 48 * 5, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 5 - 1, 249 + 5 - 60 - 20, 48 * 5 - 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 5 + 1, 249 + 5 - 60 - 20, 48 * 5 + 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48, 249 + 5 - 60 - 60, 48, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 47, 249 + 5 - 60 - 60, 47, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 49, 249 + 5 - 60 - 60, 49, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 2, 249 + 5 - 60 - 60, 48 * 2, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 2 - 1, 249 + 5 - 60 - 60, 48 * 2 - 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 2 + 1, 249 + 5 - 60 - 60, 48 * 2 + 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 3, 249 + 5 - 60 - 60, 48 * 3, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 3 - 1, 249 + 5 - 60 - 60, 48 * 3 - 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 3 + 1, 249 + 5 - 60 - 60, 48 * 3 + 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 4, 249 + 5 - 60 - 60, 48 * 4, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 4 - 1, 249 + 5 - 60 - 60, 48 * 4 - 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 4 + 1, 249 + 5 - 60 - 60, 48 * 4 + 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 5, 249 + 5 - 60 - 60, 48 * 5, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 5 - 1, 249 + 5 - 60 - 60, 48 * 5 - 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 40, 48 * 5 + 1, 249 + 5 - 60 - 60, 48 * 5 + 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 80, 48, 249 + 5 - 60 - 60, 48, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 80, 47, 249 + 5 - 60 - 60, 47, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 80, 49, 249 + 5 - 60 - 60, 49, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 80, 48 * 2, 249 + 5 - 60 - 60, 48 * 2, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 80, 48 * 2 - 1, 249 + 5 - 60 - 60, 48 * 2 - 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 80, 48 * 2 + 1, 249 + 5 - 60 - 60, 48 * 2 + 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 80, 48 * 3, 249 + 5 - 60 - 60, 48 * 3, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 80, 48 * 3 - 1, 249 + 5 - 60 - 60, 48 * 3 - 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 80, 48 * 3 + 1, 249 + 5 - 60 - 60, 48 * 3 + 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 80, 48 * 4, 249 + 5 - 60 - 60, 48 * 4, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 80, 48 * 4 - 1, 249 + 5 - 60 - 60, 48 * 4 - 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 80, 48 * 4 + 1, 249 + 5 - 60 - 60, 48 * 4 + 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 80, 48 * 5, 249 + 5 - 60 - 60, 48 * 5, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 80, 48 * 5 - 1, 249 + 5 - 60 - 60, 48 * 5 - 1, ILI9341_BLACK);
  tft2.drawLine(249 + 5 - 60 - 80, 48 * 5 + 1, 249 + 5 - 60 - 60, 48 * 5 + 1, ILI9341_BLACK);
}
void printInput() {
  tft2.fillRect(15 + 48, 182 - 90, 240, 15, ILI9341_WHITE);
  tft2.setCursor(15 + 48, 182 - 90);
  tft2.setTextSize(2);
  tft2.setTextColor(ILI9341_BLACK);
  tft2.print(inputLatLong);
}
void printToPoint(int x, int y, int size)  {
  if (last_distanceToPoint != distanceToPoint) {
    tft.fillRect(x, y, 100, 15, ILI9341_BLACK);
    tft.setTextSize(size);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(x, y);
    tft.print("D:");
    tft.setCursor(x + 25, y);
    tft.print(distanceToPoint);
  }
}
void printCourseToPoint(int x, int y, int size) {
  if (last_courseToNavigate != courseToNavigate) {
    tft.fillRect(x, y, 100, 15, ILI9341_BLACK);
    tft.setTextSize(size);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(x, y);
    tft.print("C:");
    tft.setCursor(x + 25, y);
    tft.print((int)courseToNavigate);
  }
}

