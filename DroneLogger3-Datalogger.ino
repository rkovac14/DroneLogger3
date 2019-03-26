#include <SPI.h>
#include <SD.h>

const int chipSelect = 10;
const byte numChars = 200;
char receivedChars[numChars];
char tempChars[numChars];
char messageFromPC[numChars] = {0};
long ultrasonic[4] = {0, 0, 0, 0};
boolean newData = false;

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
int course_offset = 90;
int gps_course;
int number = 0;
class Communication {
  public:
    void initialization() {
      Serial.begin(9600);
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
      while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
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
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {

    while (1);
  }
  Serial.println("card initialized.");
}

void loop() {
  Communication comp;
  comp.receive();
  if (Serial.available())  {
    String dataString = "";
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    dataString = messageFromPC;
    if (dataFile) {
      if(number == 0)  {
        dataFile.println("Id,Data,Time,Latitude,Longtitude,Alt,Name");
      }
      dataFile.print(number);
      dataFile.print(',');
      dataFile.print(gps_day);
      dataFile.print('/');
      dataFile.print(gps_month);
      dataFile.print(',');
      dataFile.print(time_hour);
      dataFile.print(':');
      dataFile.print(time_minute);
      dataFile.print(":00,");
      dataFile.print(latitude,6);
      dataFile.print(',');
      dataFile.print(longtitude,6);
      dataFile.print(',');
      dataFile.print(bmp_altitude);
      dataFile.print(',');
      dataFile.println(speed_kmph);
      dataFile.close();
    }
    number++;
  }
 


}
