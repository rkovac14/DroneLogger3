#include <TinyGPS++.h>

//communication
const byte numChars = 200;
char receivedChars[numChars];
char tempChars[numChars];
char messageFromPC[numChars] = {0};
long ultrasonic[4] = {0, 0, 0, 0};
boolean newData = false;
//communication
#define _sdcs 2

int AnimaciaState = LOW;
const long intervalAnimacia = 500;
unsigned long previousMillisAnimacia = 0;
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

int pitchToPrint;
int rollToPrint;
double courseToHome = 0;
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
int course_offset = 90;
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

long period_previousMillis = 0;
const long period_interval = 3000;
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
  attachInterrupt(digitalPinToInterrupt(2), sendTo, CHANGE);
  Serial.begin(115200);
  pinMode(A7, OUTPUT);
}

void loop() {
  /*if (roll > 20 or roll < -20 or pitch > 20 or pitch < -20) {
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
  */
  distanceToHome =  TinyGPSPlus::distanceBetween(latitude, longtitude, start_latitude, start_longtitude);
  courseToHome =    TinyGPSPlus::courseTo((double)latitude, (double)longtitude, (double)start_latitude, (double)start_longtitude);
  unsigned long period_currentMillis = millis();
  if (period_currentMillis - period_previousMillis > period_interval) {
    period_previousMillis = period_currentMillis;
    Serial.print("t0.txt=");
    Serial.print('"');
    Serial.print((int)bmp_altitude);
    Serial.print('"');
    writeToNextion();
    Serial.print("t1.txt=");
    Serial.print('"');
    Serial.print((int)speed_kmph);
    Serial.print('"');
    writeToNextion();
    Serial.print("t2.txt=");
    Serial.print('"');
    Serial.print(time_hour);
    Serial.print(':');
    if (time_minute < 10)  {
      Serial.print('0');
    }
    Serial.print(time_minute);
    Serial.print('"');
    writeToNextion();
    Serial.print("page1.t4.txt=");
    Serial.print('"');
    Serial.print(gps_day);
    Serial.print('.');
    Serial.print(gps_month);
    Serial.print('.');
    Serial.print(gps_year);
    Serial.print('"');
    writeToNextion();
    Serial.print("page1.t6.txt=");
    Serial.print('"');
    Serial.print(latitude, 6);
    Serial.print('"');
    writeToNextion();
    Serial.print("page1.t7.txt=");
    Serial.print('"');
    Serial.print(longtitude, 6);
    Serial.print('"');
    writeToNextion();

    Serial.print("page1.t17.txt=");
    Serial.print('"');
    Serial.print(courseToHome);
    Serial.print('"');
    writeToNextion();
    Serial.print("page1.t18.txt=");
    Serial.print('"');
    Serial.print(distanceToHome);
    Serial.print('"');
    writeToNextion();
  }
  if (last_bmp_altitude != bmp_altitude and bmp_altitude > 0)  {
    Serial.print("t0.txt=");
    Serial.print('"');
    Serial.print((int)bmp_altitude);
    Serial.print('"');
    writeToNextion();
    animacia();
  }
  if (last_speed_kmph != speed_kmph and speed_kmph > 0 and speed_kmph < 100)  {
    Serial.print("t1.txt=");
    Serial.print('"');
    Serial.print((int)speed_kmph);
    Serial.print('"');
    writeToNextion();
    animacia();
  }
  if (last_time_minute != time_minute and time_hour != 0) {
    Serial.print("t2.txt=");
    Serial.print('"');
    Serial.print(time_hour);
    Serial.print(':');
    if (time_minute < 10)  {
      Serial.print('0');
    }
    Serial.print(time_minute);
    Serial.print('"');
    writeToNextion();
    animacia();
  }
  if (last_gps_month != gps_month or last_gps_day != gps_day or last_gps_year != gps_year) {
    Serial.print("page1.t4.txt=");
    Serial.print('"');
    Serial.print(gps_day);
    Serial.print('.');
    Serial.print(gps_month);
    Serial.print('.');
    Serial.print(gps_year);
    Serial.print('"');
    writeToNextion();
    animacia();
  }
  if (last_latitude != latitude) {
    Serial.print("page1.t6.txt=");
    Serial.print('"');
    Serial.print(latitude, 6);
    Serial.print('"');
    if (latitude > 0 && i == 0)  {
      start_latitude = latitude;
      i++;
    }
    writeToNextion();
    animacia();
  }
  if (last_longtitude != longtitude) {
    Serial.print("page1.t7.txt=");
    Serial.print('"');
    Serial.print(longtitude, 6);
    Serial.print('"');
    if (longtitude > 0 && o == 0)  {
      start_longtitude = longtitude;
      o++;
    }
    writeToNextion();
    animacia();
  }
  if (last_start_latitude != start_latitude or last_start_longtitude != start_longtitude)  {
    Serial.print("page1.t11.txt=");
    Serial.print('"');
    Serial.print(start_latitude, 6);
    Serial.print('"');
    writeToNextion();
    Serial.print("page1.t12.txt=");
    Serial.print('"');
    Serial.print(start_longtitude, 6);
    Serial.print('"');
    writeToNextion();
    animacia();
  }
  if (last_smoothHeadingDegrees != smoothHeadingDegrees or last_courseToHome != courseToHome or last_distanceToHome != distanceToHome) {

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
    Serial.print("page1.z0.val=");
    Serial.print(smoothHeadingDegrees);
    writeToNextion();
    animacia();
    distanceToHome =  TinyGPSPlus::distanceBetween(latitude, longtitude, start_latitude, start_longtitude);
    courseToHome =    TinyGPSPlus::courseTo((double)latitude, (double)longtitude, (double)start_latitude, (double)start_longtitude);
    if (course_offset >= 0)  {
      if (courseToHome + course_offset < 359)  {
        courseToHome += course_offset;
      }
      else  {
        courseToHome = courseToHome + course_offset - 359;
      }
    }
    else  {
      if (courseToHome + course_offset <= 0) {
        courseToHome = 359 + (courseToHome + course_offset);
      }
      else  {
        courseToHome += course_offset;
      }
    }
    Serial.print("page1.z1.val=");
    Serial.print(courseToHome);
    writeToNextion();
    Serial.print("page1.t17.txt=");
    Serial.print('"');
    Serial.print(courseToHome);
    Serial.print('"');
    writeToNextion();
    Serial.print("page1.t18.txt=");
    Serial.print('"');
    Serial.print(distanceToHome);
    Serial.print('"');
    writeToNextion();
    animacia();
  }
  if (last_roll != roll or last_pitch != pitch) {
    printHorizont();
  }
  if (last_distanceToHome != distanceToHome) {
    distanceToHome =  TinyGPSPlus::distanceBetween(latitude, longtitude, start_latitude, start_longtitude);
    courseToHome =    TinyGPSPlus::courseTo((double)latitude, (double)longtitude, (double)start_latitude, (double)start_longtitude);
    Serial.print("page1.t18.txt=");
    Serial.print('"');
    Serial.print(distanceToHome);
    Serial.print('"');
    writeToNextion();
    animacia();
  }
  if (last_courseToHome != courseToHome) {
    distanceToHome =  TinyGPSPlus::distanceBetween(latitude, longtitude, start_latitude, start_longtitude);
    courseToHome =    TinyGPSPlus::courseTo((double)latitude, (double)longtitude, (double)start_latitude, (double)start_longtitude);
    Serial.print("page1.t17.txt=");
    Serial.print('"');
    Serial.print(courseToHome);
    Serial.print('"');
    writeToNextion();
    animacia();
  }
  //Serial.print("page1.p2.pic=10");
  //writeToNextion();


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
  last_distanceToHome = distanceToHome;
  last_courseToHome = courseToHome;


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
void writeToNextion() {
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}
void sendTo() {
  Communication comp;
  comp.receive();
}
void animacia() {
  unsigned long currentMillisAnimacia = millis();
  if (currentMillisAnimacia - previousMillisAnimacia >= intervalAnimacia) {
    previousMillisAnimacia = currentMillisAnimacia;
    if (AnimaciaState == LOW) {
      Serial.print("page1.p2.pic=9");
      writeToNextion();
      AnimaciaState = HIGH;
    } else {
      Serial.print("page1.p2.pic=8");
      writeToNextion();
      AnimaciaState = LOW;
    }


  }
}

void printHorizont()  {
  pitchToPrint = pitch;
  rollToPrint = roll;
  if (roll >= -50 and roll <= 50)  {
    rollToPrint = map(rollToPrint, -50, 50, 60, 11);
    pitchMode();
    Serial.print("page1.p3.pic=");
    Serial.print(rollToPrint);
    writeToNextion();
    animacia();
  }
  if (roll < -50)  {
    Serial.print("page1.p3.pic=");
    Serial.print(60);
    writeToNextion();
    animacia();
  }
  if (roll > 50)  {
    Serial.print("page1.p3.pic=");
    Serial.print(11);
    writeToNextion();
    animacia();
  }
}

void pitchMode()  {
  if (pitch >= -1 and pitch <= 1)  {
    rollToPrint += 0;
  }
  //61-110
  if (pitch == -2 or pitch == -3)  {
    rollToPrint += 50;
  }
  if (pitch == 2 or pitch == 3)  {
    rollToPrint += 100;
  }
  if (pitch == -4 or pitch == -5)  {
    rollToPrint += 150;
  }
  if (pitch == 4 or pitch == 5)  {
    rollToPrint += 200;
  }
  if (pitch == -6 or pitch == -7)  {
    rollToPrint += 250;
  }
  if (pitch == 6 or pitch == 7)  {
    rollToPrint += 300;
  }
  if (pitch == -8 or pitch == -9)  {
    rollToPrint += 350;
  }
  if (pitch == 8 or pitch == 9)  {
    rollToPrint += 400;
  }
  if (pitch == -10 or pitch == -11)  {
    rollToPrint += 450;
  }
  if (pitch == 10 or pitch == 11)  {
    rollToPrint += 500;
  }
  if (pitch == -12 or pitch == -13)  {
    rollToPrint += 550;
  }
  if (pitch == 12 or pitch == 13)  {
    rollToPrint += 600;
  }
  if (pitch == -14 or pitch == -15)  {
    rollToPrint += 650;
  }
  if (pitch == 14 or pitch == 15)  {
    rollToPrint += 700;
  }
  if (pitch == -16 or pitch == -17)  {
    rollToPrint += 750;
  }
  if (pitch == 16 or pitch == 17)  {
    rollToPrint += 800;
  }
  if (pitch == -18 or pitch == -19)  {
    rollToPrint += 850;
  }
  if (pitch == 18 or pitch == 19)  {
    rollToPrint += 900;
  }
  if (pitch == -20 or pitch == -21)  {
    rollToPrint += 950;
  }
  if (pitch == 20 or pitch == 21)  {
    rollToPrint += 1000;
  }
  if (pitch < -21) {
    rollToPrint += 950;
  }
  if (pitch > 21) {
    rollToPrint += 1000;
  }
}
