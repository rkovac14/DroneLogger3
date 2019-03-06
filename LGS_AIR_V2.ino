/*
   LGS AIR V2
   Marco Pintér
   2018
*/

#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Kalman.h>
#include <MechaQMC5883.h>
SoftwareSerial gyroscope(2, 3); //RX, TX
MechaQMC5883 qmc;
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
//pressure
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
Kalman kalmanX;
Kalman kalmanY;
Adafruit_BMP280 bme;
//Adafruit_BMP280 bme(BMP_CS); // hardware SPI
//Adafruit_BMP280 bme(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
#define RESTRICT_PITCH
//pressure

//Compass
float heading;
float declinationAngle;
float headingDegrees;
float fixedHeadingDegrees;
int smoothHeadingDegrees;
int previousDegree;
int x, y, z;
int azimuth;
int last_azimuth;
//compass

//gyro
int pitch;
int roll;
//gyro

//gps
static const int RXPin = A2, TXPin = A3;
static const uint32_t GPSBaud = 38400;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
unsigned long last = 0UL;
long sats = 0;
float latitude = 0;
float longtitude = 0;
long fix_age = 0;
long gps_month = 0, gps_day = 0, gps_year = 0;
long time_hour = 0, time_minute = 0, time_second = 0;
float alt = 0;
float speed_kmph = 0;
float distanceToLondon = 0;
long gps_course;
//gps
//temperature
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float temperature;
//temperature

//pressure
float bmp_temperature = 0;
float bmp_pressure = 0;
float bmp_altitude = 0;
//pressure

//last
int last_smoothHeadingDegrees;
int last_pitch;
int last_roll;
float last_temperature;
long last_sats = 0;
float last_latitude = 0;
float last_longtitude = 0;
long last_fix_age = 0;
long last_gps_month = 0, last_gps_day = 0, last_gps_year = 0;
long last_time_hour = 0, last_time_minute = 0, last_time_second = 0;
float last_alt = 0;
float last_speed_kmph = 0;
double last_distanceToLondon = 0;
//last

//communication
unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
const long interval = 100;
const long interval2 = 1000;
int but;
int arm;

const byte numChars = 150;
char receivedChars[numChars];
char tempChars[numChars];
char messageFromPC[numChars] = {0};
boolean newData = false;

char receivedCharsGyro[numChars];
char tempCharsGyro[numChars];
char messageFromPCGyro[numChars] = {0};
boolean newDataGyro = false;
//communication
int pitch_final;
int roll_final;
int azimuth_final;

int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;

long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;


double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY;
uint32_t timer;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
float heading1;
float heading2;
class Pressure  {
  public:
    void initialization() {
      Serial.begin(9600);
      if (!bme.begin(0x76)) {
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        while (1);
      }
    }
    void getValuePressure() {
      bmp_temperature = bme.readTemperature();
      bmp_pressure = bme.readPressure();
      bmp_altitude = (bme.readAltitude(1013.25));
    }
};

class Compass    {
  public:
    void serialOutput() {
      Serial.print(" Heading = ");
      Serial.print(heading);
      Serial.print(" Degress = ");
      Serial.print(headingDegrees);
      Serial.print(" Fixed Degress = ");
      Serial.print(fixedHeadingDegrees);
      Serial.print(" Smooth Degress = ");
      Serial.println(smoothHeadingDegrees);
      delay(100);
    }
    void getValueCompass() {
      accX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
      accY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      accZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      gyroX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      gyroY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      gyroZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
      qmc.read(&x, &y, &z, &azimuth);
       /* heading = atan2(y, x);
        float declinationAngle = (4.0 + (26.0 / 60.0));
        heading += declinationAngle;
        if (heading < 0)
        {
        heading += 2 * PI;
        }

        if (heading > 2 * PI)
        {
        heading -= 2 * PI;
        }
        float headingDegrees = heading * 180 / M_PI;
        azimuth_final = headingDegrees;
      */
      heading1 = noTiltCompensate();
      heading2 = tiltCompensate();

      if (heading2 == -1000)
      {
        heading2 = heading1;
      }
      float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
      heading1 += declinationAngle;
      heading2 += declinationAngle;

      // Correct for heading < 0deg and heading > 360deg
      heading1 = correctAngle(heading1);
      heading2 = correctAngle(heading2);

      // Convert to degrees
      heading1 = heading1 * 180 / M_PI;
      heading2 = heading2 * 180 / M_PI;

      // Output
      /*Serial.print(heading1);
      Serial.print(":");
      Serial.println(heading2);*/
    }
    void initialization() {
      Wire.begin();
      Serial.begin(9600);
      qmc.init();

    }
    float noTiltCompensate()
    {
      float heading = atan2(y, x);
      return heading;
    }

    // Tilt compensation
    float tiltCompensate()
    {
      // Pitch & Roll

      float roll;
      float pitch;

      roll = asin(accY);
      pitch = asin(-accX);

      if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78)
      {
        return -1000;
      }

      // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
      float cosRoll = cos(roll);
      float sinRoll = sin(roll);
      float cosPitch = cos(pitch);
      float sinPitch = sin(pitch);

      // Tilt compensation
      float Xh = x * cosPitch + z * sinPitch;
      float Yh = x * sinRoll * sinPitch + y * cosRoll - z * sinRoll * cosPitch;

      float heading = atan2(Yh, Xh);

      return heading;
    }
    float correctAngle(float heading)
    {
      if (heading < 0) {
        heading += 2 * PI;
      }
      if (heading > 2 * PI) {
        heading -= 2 * PI;
      }

      return heading;
    }
};

class Gyro  {
  private:
    int x, y, z;
    const int MPU_addr = 0x68;
    int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
    int gyro[4];
    int minVal = 265;
    int maxVal = 402;
  public:
    void initialization() {
      /*Wire.begin();
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x6B);
        Wire.write(0);
        Wire.endTransmission(true);*/
      gyroscope.begin(115200);
    }
    void getValueGyro() {
      roll  = atan2(accY, accZ) * RAD_TO_DEG;
      double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
      double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
      timer = micros();
      Wire.beginTransmission(MPU_addr);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_addr, 14, true);
      accX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
      accY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      accZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      gyroX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      gyroY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      gyroZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
#ifdef RESTRICT_PITCH // Eq. 25 and 26
      roll  = atan2(accY, accZ) * RAD_TO_DEG;
      pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
      roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
      pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

      double gyroXrate = gyroX / 131.0; // Convert to deg/s
      double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
      // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
      if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        kalmanX.setAngle(roll);
        compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
      } else
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

      if (abs(kalAngleX) > 90)
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
      // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
      if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
        kalmanY.setAngle(pitch);
        compAngleY = pitch;
        kalAngleY = pitch;
        gyroYangle = pitch;
      } else
        kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

      if (abs(kalAngleY) > 90)
        gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

      gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
      gyroYangle += gyroYrate * dt;
      //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
      //gyroYangle += kalmanY.getRate() * dt;

      compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
      compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

      // Reset the gyro angle when it has drifted too much
      if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalAngleX;
      if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalAngleY;

      /*
            if (y > 180)  {
              y -= 360;
            }
      */
      pitch_final = kalAngleX;
      roll_final = kalAngleY;
      if (roll_final < 0)  {
        roll_final = abs(roll_final);
      }
      else  {
        roll_final *= -1;
      }
      temperature = (double)tempRaw / 340.0 + 36.53;
      /*Serial.print(roll); Serial.print("\t");
        Serial.print(gyroXangle); Serial.print("\t");
        Serial.print(compAngleX); Serial.print("\t");
        Serial.print(kalAngleX); Serial.print("\t");

        Serial.print("\t");

        Serial.print(pitch); Serial.print("\t");
        Serial.print(gyroYangle); Serial.print("\t");
        Serial.print(compAngleY); Serial.print("\t");
        Serial.print(kalAngleY); Serial.println("\t");*/
      /*if (pitch > 180) {
        pitch = map(pitch, 181, 360, -180, 0);
        }*/

      /*
        recvWithStartEndMarkers();
        if (newDataGyro == true) {
        strcpy(tempCharsGyro, receivedCharsGyro);
        parseData();
        newDataGyro = false;
        }*/
    }
    void recvWithStartEndMarkers() {
      static boolean recvInProgress = false;
      static byte ndx = 0;
      char startMarker = '<';
      char endMarker = '>';
      char rc;
      while (gyroscope.available() > 0 && newDataGyro == false) {
        rc = gyroscope.read();
        if (recvInProgress == true) {
          if (rc != endMarker) {
            receivedCharsGyro[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
              ndx = numChars - 1;
            }
          }
          else {
            receivedCharsGyro[ndx] = '\0';
            recvInProgress = false;
            ndx = 0;
            newDataGyro = true;
          }
        }
        else if (rc == startMarker) {
          recvInProgress = true;
        }
      }
    }
    void parseData() {
      char * strtokIndx;
      strtokIndx = strtok(tempCharsGyro, ",");
      strcpy(messageFromPCGyro, strtokIndx);
      strtokIndx = strtok(NULL, ",");
      pitch = atoi(strtokIndx);
      strtokIndx = strtok(NULL, ",");
      roll = atoi(strtokIndx);
    }
    void serialOutput() {
      Serial.print("Pitch: ");
      Serial.print(pitch);
      Serial.print(" Roll: ");
      Serial.print("  ");
      Serial.println(roll);
      delay(100);
    }
};

class Gps {
  private:
    static const double LONDON_LAT = 50.8503, LONDON_LON = 4.3517;

  public:
    void initialization(int input) {
      ss.begin(GPSBaud);
      if (input == 0) {
        Serial.println("Initialize GPS");
      }
    }
    void getValueGps()  {
      // Dispatch incoming characters
      while (ss.available() > 0)
        gps.encode(ss.read());


      /*if(gps.satellites.isUpdated())  {
        Serial.print("Sats: ");
        Serial.println(gps.satellites.value());
        sats = gps.satellites.value();
        }*/
      distanceToLondon =
        TinyGPSPlus::distanceBetween(
          latitude,
          longtitude,
          LONDON_LAT,
          LONDON_LON);
      sats = gps.sentencesWithFix();
      alt = gps.altitude.kilometers();
      //fix_age = gps.satellites.age();
      if (gps.location.isUpdated())
      {
        /*Serial.print(F("LOCATION   Fix Age="));
          Serial.print(gps.location.age());
          Serial.print(F("ms Raw Lat="));
          Serial.print(gps.location.rawLat().negative ? "-" : "+");
          Serial.print(gps.location.rawLat().deg);
          Serial.print("[+");
          Serial.print(gps.location.rawLat().billionths);
          Serial.print(F(" billionths],  Raw Long="));
          Serial.print(gps.location.rawLng().negative ? "-" : "+");
          Serial.print(gps.location.rawLng().deg);
          Serial.print("[+");
          Serial.print(gps.location.rawLng().billionths);
          Serial.print(F(" billionths],  Lat="));
          Serial.print(gps.location.lat(), 6);
          Serial.print(F(" Long="));
          Serial.println(gps.location.lng(), 6);*/
        latitude = gps.location.lat();
        longtitude = gps.location.lng();

      }

      else if (gps.date.isUpdated())
      {
        /*Serial.print(F("DATE       Fix Age="));
          Serial.print(gps.date.age());
          Serial.print(F("ms Raw="));
          Serial.print(gps.date.value());
          Serial.print(F(" Year="));
          Serial.print(gps.date.year());
          Serial.print(F(" Month="));
          Serial.print(gps.date.month());
          Serial.print(F(" Day="));
          Serial.println(gps.date.day());*/
        gps_month = gps.date.month();
        gps_day = gps.date.day();
        gps_year = gps.date.year();

      }
      /*else if(gps.course.isUpdated()) {
        gps_course = gps.course.deg();
        }*/

      else if (gps.time.isUpdated())
      {
        /*Serial.print(F("TIME       Fix Age="));
          Serial.print(gps.time.age());
          Serial.print(F("ms Raw="));
          Serial.print(gps.time.value());
          Serial.print(F(" Hour="));
          Serial.print(gps.time.hour());
          Serial.print(F(" Minute="));
          Serial.print(gps.time.minute());
          Serial.print(F(" Second="));
          Serial.print(gps.time.second());
          Serial.print(F(" Hundredths="));
          Serial.println(gps.time.centisecond());*/
        time_hour = gps.time.hour() + 1;
        time_minute = gps.time.minute();
      }

      else if (gps.speed.isUpdated())
      {
        /*Serial.print(F("SPEED      Fix Age="));
          Serial.print(gps.speed.age());
          Serial.print(F("ms Raw="));
          Serial.print(gps.speed.value());
          Serial.print(F(" Knots="));
          Serial.print(gps.speed.knots());
          Serial.print(F(" MPH="));
          Serial.print(gps.speed.mph());
          Serial.print(F(" m/s="));
          Serial.print(gps.speed.mps());
          Serial.print(F(" km/h="));
          Serial.println(gps.speed.kmph());*/
        speed_kmph = gps.speed.kmph();
      }

      else if (gps.course.isUpdated())
      {
        /*Serial.print(F("COURSE     Fix Age="));
          Serial.print(gps.course.age());
          Serial.print(F("ms Raw="));
          Serial.print(gps.course.value());
          Serial.print(F(" Deg="));*/
        gps_course = gps.course.deg();
      }

      else if (gps.altitude.isUpdated())
      {
        /*Serial.print(F("ALTITUDE   Fix Age="));
          Serial.print(gps.altitude.age());
          Serial.print(F("ms Raw="));
          Serial.print(gps.altitude.value());
          Serial.print(F(" Meters="));
          Serial.print(gps.altitude.meters());
          Serial.print(F(" Miles="));
          Serial.print(gps.altitude.miles());
          Serial.print(F(" KM="));
          Serial.print(gps.altitude.kilometers());
          Serial.print(F(" Feet="));
          Serial.println(gps.altitude.feet());*/
        alt = gps.altitude.kilometers();
      }

      else if (gps.satellites.isUpdated())
      {
        /*Serial.print(F("SATELLITES Fix Age="));
          Serial.print(gps.satellites.age());
          Serial.print(F("ms Value="));
          Serial.println(gps.satellites.value());*/
        //sats = gps.satellites.value();
        fix_age = gps.satellites.age();
      }

      else if (gps.hdop.isUpdated())
      {
        Serial.print(F("HDOP       Fix Age="));
        Serial.print(gps.hdop.age());
        Serial.print(F("ms Value="));
        Serial.println(gps.hdop.value());
      }

      else if (millis() - last > 5000)
      {
        Serial.println();
        if (gps.location.isValid())
        {
          static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
          double distanceToLondon =
            TinyGPSPlus::distanceBetween(
              latitude,
              longtitude,
              LONDON_LAT,
              LONDON_LON);
          double courseToLondon =
            TinyGPSPlus::courseTo(
              gps.location.lat(),
              gps.location.lng(),
              LONDON_LAT,
              LONDON_LON);

          Serial.print(F("LONDON     Distance="));
          Serial.print(distanceToLondon / 1000, 6);
          Serial.print(F(" km Course-to="));
          Serial.print(courseToLondon, 6);
          Serial.print(F(" degrees ["));
          Serial.print(TinyGPSPlus::cardinal(courseToLondon));
          Serial.println(F("]"));

        }

        Serial.print(F("DIAGS      Chars="));
        Serial.print(gps.charsProcessed());
        Serial.print(F(" Sentences-with-Fix="));
        Serial.print(gps.sentencesWithFix());
        Serial.print(F(" Failed-checksum="));
        Serial.print(gps.failedChecksum());
        Serial.print(F(" Passed-checksum="));
        Serial.println(gps.passedChecksum());

        if (gps.charsProcessed() < 10)
          Serial.println(F("WARNING: No GPS data.  Check wiring."));

        last = millis();
        Serial.println();
      }
    }

};

class Temperature {
  public:
    void initialization() {
      Wire.begin();
      Wire.beginTransmission(MPU_addr);
      Wire.write(0x6B);  // PWR_MGMT_1 register
      Wire.write(0);     // set to zero (wakes up the MPU-6050)
      Wire.endTransmission(true);
    }
    void getValueTemperature()  {
      Wire.beginTransmission(MPU_addr);
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
      AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
      AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
      temperature = Tmp / 340.00 + 26.53;
    }
};

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
      char * strtokIndx;
      strtokIndx = strtok(tempChars, ",");
      strcpy(messageFromPC, strtokIndx);
      strtokIndx = strtok(NULL, ",");
      but = atoi(strtokIndx);
      strtokIndx = strtok(NULL, ",");
      arm = atoi(strtokIndx);
    }
};
void setup()  {

  gyroscope.begin(115200);
  /*
     note gps
     gps.initialization(0); = no initial serial monitor;
     gps.initialization(1); = initial serial monitor;
  */
  Pressure pres;
  Communication comp;
  comp.initialization();
  Compass compass;
  Gyro gyro;
  Gps gps;
  Temperature temp;
  compass.initialization();
  gyro.initialization();
  gps.initialization(1);
  temp.initialization();
  pres.initialization();
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(10, OUTPUT);
  Serial.begin(9600);
  /*
    setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro                                             //Set digital output 13 high to indicate startup
    //Set the LCD cursor to position to position 0,1
    for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {                 //Run this code 2000 times
      if (cal_int % 125 == 0)                             //Print a dot on the LCD every 125 readings
        read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
      gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
      gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
      gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
      delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
    }
    gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
    gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
    gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offs

    loop_timer = micros();*/
  accX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

void loop() {
  Communication comp;
  comp.receive();
  //compass.serialOutput();
  Gyro gyro;
  gyro.getValueGyro();

  //gyro.serialOutput();
  /*  read_mpu_6050_data();

    gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
    gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
    gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value

    //Gyro angle calculations
    //0.0000611 = 1 / (250Hz / 65.5)
    angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
    angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable

    //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
    angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
    angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

    //Accelerometer angle calculations
    acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector
    //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;    //Calculate the pitch angle
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;    //Calculate the roll angle

    //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
    angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
    angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

    if (set_gyro_angles) {                                               //If the IMU is already started
      angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
      angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
    }
    else {                                                               //At first start
      angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
      angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
      set_gyro_angles = true;                                            //Set the IMU started flag
    }

    //To dampen the pitch and roll angles a complementary filter is used
    angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
    angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
    //Serial.println(angle_roll_output);
    pitch = angle_pitch_output;
    roll = angle_roll_output;
  */
  unsigned long currentMillis = millis();
  digitalWrite(10, HIGH);
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Compass compass;
    compass.getValueCompass();
    /*if ( smoothHeadingDegrees != last_smoothHeadingDegrees or
         pitch != last_pitch or
         roll != last_roll or
         temperature != last_temperature or
         sats != last_sats or
         latitude != last_latitude or
         longtitude != last_longtitude or
         gps_month != last_gps_month or
         gps_day != last_gps_day or
         gps_year != last_gps_year or
         time_hour != last_time_hour or
         time_minute != last_time_minute or
         fix_age != last_fix_age or
         alt != last_alt or
         speed_kmph != last_speed_kmph or
         distanceToLondon != last_distanceToLondon
       ) {*/  //-35
    int course_offset = 0;
    /*if(smoothHeadingDegrees >= 0 and smoothHeadingDegrees <= 70)  {
      course_offset = map(smoothHeadingDegrees,0,45,3,35);
      Serial.println(course_offset);
      }
      else  {
      course_offset = 0;
      }*/
    if (course_offset >= 0) {
      if (azimuth + course_offset < 359)  {
        azimuth += course_offset;
      }
      else  {
        azimuth = azimuth + course_offset - 359;
      }
    }
    else  {
      if (azimuth + course_offset <= 0) {
        azimuth = 359 + (azimuth + course_offset);
      }
      else  {

        azimuth = azimuth + course_offset;
      }
    }

    Serial.print("<DATA,1,");
    Serial.print(heading2);
    Serial.print(",");
    Serial.print(pitch_final);
    Serial.print(",");
    Serial.print(roll_final);
    Serial.println(">");
    
    //}
    /*last_azimuth = azimuth;
      last_pitch = pitch;
      last_roll = roll;
      last_temperature = temperature;
      last_sats = sats;
      last_latitude = latitude;
      last_longtitude = longtitude;
      last_fix_age = fix_age;
      last_gps_month = gps_month, last_gps_day = gps_day, last_gps_year = gps_year;
      last_time_hour = time_hour, last_time_minute = time_minute;
      last_alt = alt;
      last_speed_kmph = speed_kmph;
      last_distanceToLondon = distanceToLondon;*/
    /*Serial.print(but);
      Serial.print("   ");
      Serial.println(arm);*/
  }
  unsigned long currentMillis2 = millis();
  if (currentMillis2 - previousMillis2 >= interval2) {
    previousMillis2 = currentMillis2;
    Gps gps;
    gps.getValueGps();
    Pressure pres;
    pres.getValuePressure();
    Serial.print("<DATA,2,");
    Serial.print(temperature, 1);
    Serial.print(",");
    Serial.print(sats);
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
    Serial.print((int)speed_kmph);
    Serial.print(",");
    Serial.print((int)bmp_altitude);
    Serial.print(",");
    Serial.print(gps_course);
    Serial.println(">");
    
    last_azimuth = azimuth;
    last_pitch = pitch;
    last_roll = roll;
    last_temperature = temperature;
    last_sats = sats;
    last_latitude = latitude;
    last_longtitude = longtitude;
    last_fix_age = fix_age;
    last_gps_month = gps_month, last_gps_day = gps_day, last_gps_year = gps_year;
    last_time_hour = time_hour, last_time_minute = time_minute;
    last_alt = alt;
    last_speed_kmph = speed_kmph;
    last_distanceToLondon = distanceToLondon;
  }
  if (arm == 1)  {
    digitalWrite(3, HIGH);
  }
  else  {
    digitalWrite(3, LOW);
  }
  if (but == 1)  {
    digitalWrite(4, HIGH);
  }
  else  {
    digitalWrite(4, LOW);
  }
  if (but == 2)  {
    digitalWrite(5, HIGH);
  }
  else  {
    digitalWrite(5, LOW);
  }
  if (but == 3)  {
    digitalWrite(2, HIGH);
  }
  else  {
    digitalWrite(2, LOW);
  }
  if (but == 4)  {
    digitalWrite(7, HIGH);
  }
  else  {
    digitalWrite(7, LOW);
  }
  but = 0;
  digitalWrite(10, LOW);


  /*while (micros() - loop_timer < 4000);                                //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
    loop_timer = micros();*/
}
void read_mpu_6050_data() {                                            //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68, 14);                                          //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                                       //Wait until all the bytes are received
  acc_x = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_x variable
  acc_y = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_y variable
  acc_z = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_z variable
  temperature = Wire.read() << 8 | Wire.read();                        //Add the low and high byte to the temperature variable
  gyro_x = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_z variable

}
void setup_mpu_6050_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}





