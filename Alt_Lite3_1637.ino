#include <Arduino.h> // for platformIO

//I2C device found at address 0x3C !  OLED
//I2C device found at address 0x50 !  EEPROM
//I2C device found at address 0x68 !  AXL
//I2C device found at address 0x76 !  BMP
// 950 - 953 - Max Speed
// 947 - 947 - NumRec
// 945 - 946 - Apogee
//A,B,C,D,E,F,G,H,J,L,N,O,P,S,U,Y,a,b,c,d,e,f,h,i,j,l,n,o,q,r,t,u,y,dash,under,equal,empty,degree

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <EEPROM.h>
#include <FM24C256.h>
#include "MPU9250.h"
#include "GyverTM1637.h"

#define CLK 12
#define DIO 11
#define BUTTON 13
#define BUZZER 15

#define MOSFET1 9
#define MOSFET2 10
#define MOSFET3 0

#define DEBUG_OUT true
#define DEBUG_MOSFET false

#define Cycles 600

Adafruit_BMP280 bme;
FM24C256 driveD(0x50);
MPU9250 IMU(Wire, 0x68);
GyverTM1637 disp(CLK, DIO);

float SEALEVELPRESSURE_HPA;
int EEPOS = 0;
int EEXPos;
float Alt1, Alt2;
long int Pressure;
boolean writeit;
boolean powerLost;
int PackSize;
int bx, by, bz;
int bax, bay, baz;
float cx1, cy1, cz1;
double xyz[3];
double ax, ay, az;
int x, y, z;

int bufwrite;
int msgCount;
boolean Fallen;
byte NumRec;
int xCal, yCal, zCal, axCal, ayCal, azCal;
long int Start, Start2, Finish, Finish2, routineTime;
long int FirstTime, SecondTime, oldAltitude, newAltitude, SecondTimeM, FirstTimeM;
boolean MOSFET_1_IS_FIRED, MOSFET_2_IS_FIRED, MOSFET_3_IS_FIRED;

int Maxspeed;
float Altitude;

float Alt_filtered;
float Alts[10];
int Spd_max = 17; //meters per tick (17 -> 340 m/s @ 50ms)

int Apogee;
int Temperature;
float Speed;

byte JournalSize;
byte currentByte;
byte header[4] = {170, 171, 186, 187};
byte command;
unsigned long millisshift;

struct telemetrystruct
{
  int bax, bay, baz;
  int bx, by, bz;
  long int Pressure;
  int Temperature;
  float Altitude;
  float Speed;
};

struct SystemLog
{
  unsigned long timestamp;
  char message[25];
};

struct telemetrystruct telemetry;
struct SystemLog capitansLog;

float AltFilter()
{

  for (int pos = 0; pos < 2; pos++)
  {
    Alts[2 - pos] = Alts[1 - pos];
  }

  Alts[0] = Altitude;

  if (abs(Alts[0] - Alts[1]) > Spd_max)
  {
    Alts[0] = Alts[1] + (Alts[1] - Alts[2]);
  }

  for (int pos = 0; pos < 4; pos++)
  {
    Alts[7 - pos] = Alts[6 - pos];
  }

  Alts[3] = Alts[0];

  Alt_filtered = (Alts[3] + Alts[4] + Alts[5] + Alts[6] + Alts[7]) / 5;

  if (DEBUG_OUT)
  {
    Serial.println( String (millis() - millisshift) + ":Alt_filtered:" + String(Alt_filtered));
  }

  return Alt_filtered;
}

void beeper(int milsec)
{
  digitalWrite(BUZZER, HIGH);
  delay(milsec);
  digitalWrite(BUZZER, LOW);
}

float speedOmeter()
{

  float FloatSpeed;
  Alt2 = bme.readAltitude(SEALEVELPRESSURE_HPA);

  FirstTimeM = millis();

  if (FirstTimeM - SecondTimeM > 100)
  {

    FloatSpeed = (Alt2 - Alt1) / (FirstTimeM - SecondTimeM) * 100000;

    Speed = FloatSpeed / 100;
    Alt1 = Alt2;
    SecondTimeM = millis();

    if (Speed > Maxspeed)
    {
      Maxspeed = Speed;
    }
  }
  return Speed;
}

void toLog(String message)
{

  if (EEPOS < 928)
  {
    int eventSize = sizeof(capitansLog);
    char event[25];
    message.toCharArray(event, 25);
    memcpy(capitansLog.message, event, 25);
    capitansLog.timestamp = millis() - millisshift;
    EEPROM.put(EEPOS, capitansLog);
    EEPOS = EEPOS + eventSize;
    NumRec = EEPOS / eventSize;
    EEPROM.write(947, NumRec);
  }
}

void Writelog()
{
  if (EEXPos < 32500)
  {
    unsigned char *telemetry_bytes;

    telemetry_bytes = (unsigned char *)&telemetry;

    for (bufwrite = 0; bufwrite < PackSize; bufwrite++)
    {
      driveD.write(EEXPos + bufwrite, telemetry_bytes[bufwrite]);
    }
    EEXPos = EEXPos + PackSize;
  }
}

void getdata()
{
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  //                                     AXL SENSORS COLLECTION                                    //
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  IMU.readSensor();

  cx1 = IMU.getAccelX_mss();
  cy1 = IMU.getAccelY_mss();
  cz1 = IMU.getAccelZ_mss();

  cx1 = cx1 * 10;
  cy1 = cy1 * 10;
  cz1 = cz1 * 10;

  bx = cx1;
  by = cy1;
  bz = cz1;

  float fbax, fbay, fbaz;

  fbax = IMU.getGyroX_rads();
  fbay = IMU.getGyroY_rads();
  fbaz = IMU.getGyroZ_rads();

  fbax = fbax * 57.2958;
  fbay = fbay * 57.2958;
  fbaz = fbaz * 57.2958;

  bax = fbax;
  bay = fbay;
  baz = fbaz;

  ///////////////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  //                                     OTHER SENSORS COLLECTION                                  //
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  Pressure = bme.readPressure();

  Altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  Temperature = bme.readTemperature();

  ///////////////////////////////////////////////////////////////////////////////////////////////////

  telemetry.bx = bx;
  telemetry.by = by;
  telemetry.bz = bz;

  telemetry.bax = bax;
  telemetry.bay = bay;
  telemetry.baz = baz;

  telemetry.Pressure = Pressure;
  telemetry.Temperature = Temperature;
  telemetry.Altitude = Altitude;
  telemetry.Speed = speedOmeter();


  if (DEBUG_OUT)
  {
    Serial.println( String (millis() - millisshift) + ":Altitude :" + String(Altitude));
  }



  ///////////////////////////////////////////////////////////////////////////////////////////////////
}

void fallingSense()
{
  int filtered = AltFilter();

  if (!Fallen)
  {

    if ((oldAltitude - newAltitude) > 2)
    {
      Apogee = oldAltitude;
      toLog("Falling detected " + String(Apogee));
      EEPROM.put(945, Apogee);
      Fallen = true;

      if (DEBUG_OUT)
      {
        Serial.println( String (millis() - millisshift) + ":Falling detected at:" + String(Apogee));
      }
    }

    FirstTime = millis();
    if (FirstTime - SecondTime > 500)
    {
      SecondTime = millis();
      oldAltitude = newAltitude;
      newAltitude = filtered;
    }
  }
}

void MOSFET_FIRE(byte Number)
{
  switch (Number)
  {
    case 1:
      disp.clear();
      digitalWrite(MOSFET1, HIGH);
      beeper(300);
      digitalWrite(MOSFET1, LOW);
      MOSFET_1_IS_FIRED = true;
      toLog("MOSFET_1 IS FIRED");
      disp.clear();
      break;

    case 2:
      disp.clear();
      digitalWrite(MOSFET2, HIGH);
      beeper(300);
      digitalWrite(MOSFET2, LOW);
      MOSFET_2_IS_FIRED = true;
      toLog("MOSFET_2 IS FIRED");
      disp.clear();
      break;

    case 3:
      disp.clear();
      digitalWrite(MOSFET3, HIGH);
      beeper(300);
      digitalWrite(MOSFET3, LOW);
      MOSFET_3_IS_FIRED = true;
      toLog("MOSFET_3 IS FIRED");
      disp.clear();
      break;
  }
}

void fromLog()
{
  Serial.println("-----------------------------------------------");
  int eventSize = sizeof(capitansLog);

  NumRec = EEPROM.read(947);

  for (int msgCount = 0; msgCount < (eventSize * NumRec); msgCount = msgCount + eventSize)
  {
    EEPROM.get(msgCount, capitansLog);
    String str(capitansLog.message);
    Serial.print(capitansLog.timestamp);
    Serial.print(":");
    Serial.println(str);
    Serial.println("-----------------------------------------------");
  }
}

void getInfo2()
{
  EEXPos = 0;
  EEPROM.get(945, Apogee);
  Serial.print("Apogee = ");
  Serial.println(Apogee);

  EEPROM.get(950, Maxspeed);
  Serial.print("Max Speed = ");
  Serial.println(Maxspeed);

  PackSize = sizeof(telemetry);
  byte Packet[PackSize];

  Serial.println("  Alt\t Spd\t Prs\t Tmp\t bx\t by\t bz\t gX\t gY\t gZ");
  Serial.println(" ");

  for (int Rec = 0; Rec < Cycles; Rec++)
  {
    for (int intern = 0; intern < PackSize; intern++)
    {
      Packet[intern] = driveD.read(EEXPos + intern);
    }

    memcpy(&telemetry, Packet, sizeof(telemetry));

    bx = telemetry.bx;
    by = telemetry.by;
    bz = telemetry.bz;

    float DT_bx = bx;
    float DT_by = by;
    float DT_bz = bz;

    DT_bx = DT_bx / 10;
    DT_by = DT_by / 10;
    DT_bz = DT_bz / 10;

    bax = telemetry.bax;
    bay = telemetry.bay;
    baz = telemetry.baz;

    Pressure = telemetry.Pressure;
    Temperature = telemetry.Temperature;
    Altitude = telemetry.Altitude;
    Speed = telemetry.Speed;

    Serial.print(Altitude);
    Serial.print("\t");
    Serial.print(Speed);
    Serial.print("\t");
    Serial.print(Pressure);
    Serial.print("\t");
    Serial.print(Temperature);
    Serial.print("\t");
    Serial.print(DT_bx, 2);
    Serial.print("\t");
    Serial.print(DT_by, 2);
    Serial.print("\t");
    Serial.print(DT_bz, 2);
    Serial.print("\t");
    Serial.print(round(bax));
    Serial.print("\t");
    Serial.print(round(bay));
    Serial.print("\t");
    Serial.println(round(baz));

    EEXPos = EEXPos + PackSize;
  }
  Serial.println("-----------------------------------------------------");
  Serial.print(EEXPos);
  Serial.println(" bytes of data");
}

void LANDING_PROCEDURE()
{
  toLog("LANDING_PROCEDURE@" + String(telemetry.Altitude));
  MOSFET_FIRE(1);
}

/* void sendheader(byte command)
  {
  for (byte q = 0; q < 4; q++)
  {
    Serial.write(header[q]);
  }
  Serial.write(command);
  }

  void SendData()
  {
  sendheader(01);

  Serial.write(NumRec);
  Serial.write(highByte(Cycles));
  Serial.write(lowByte(Cycles));
  Serial.flush();
  delay(200);

  sendheader(02);

  /////////////////////////////////////SEND CYCLES////////////////////////////////////////////////
  for (int q = 0; q < Cycles * PackSize; q++)
  {
    byte Sendbyte = driveD.read(q);
    Serial.write(Sendbyte);
  }
  Serial.flush();
  delay(200);
  ////////////////////////////////////////////////////////////////////////////////////////////////

  /////////////////////////////////////SEND JOURNAL///////////////////////////////////////////////
  for (int q = 0; q < NumRec * JournalSize; q++)
  {
    byte Sendbyte = EEPROM.read(q);
    Serial.write(Sendbyte);
  }
  Serial.flush();
  delay(200);
  /////////////////////////////////////SEND DUMP////////////////////////////////////////////////
  for (int q = 945; q < 1024; q++)
  {
    byte Sendbyte = EEPROM.read(q);
    Serial.write(Sendbyte);
  }
  Serial.flush();
  ////////////////////////////////////////////////////////////////////////////////////////////////
  } */

void LOGonOSD()
{
  beeper(10);
  EEPROM.get(945, Apogee);
  EEPROM.get(950, Maxspeed);

  disp.clear();
  disp.displayInt(Apogee);
  delay(6000);
  beeper(10);
  disp.clear();
  disp.displayInt(Maxspeed);
  delay(6000);

  /*
    disp.clear();
    disp.displayInt(433);
    SendData();
  */
}

void test_mosfets()
{

  for (int q = 10; q >= 0; q--)
  {
    disp.displayInt(q);
    delay(1000);
    disp.clear();
  }

  disp.clear();
  disp.displayInt(1);
  delay(2000);
  MOSFET_FIRE(1);

  disp.clear();
  disp.displayInt(2);
  delay(2000);
  MOSFET_FIRE(2);

  disp.clear();
  disp.displayInt(3);
  delay(2000);
  MOSFET_FIRE(3);
  delay(2000);

  disp.clear();
  disp.displayInt(9999);
  delay(2000);
  disp.clear();
}

void setup()
{

  MOSFET_1_IS_FIRED = false;
  MOSFET_2_IS_FIRED = false;
  MOSFET_3_IS_FIRED = false;

  Fallen = false;
  Apogee = 0;
  Maxspeed = 0;

  JournalSize = sizeof(capitansLog);
  PackSize = sizeof(telemetry);

  if (DEBUG_OUT)
  {
    Serial.println( String (millis() - millisshift) + ":Wake up and shine!:");
  }


  disp.clear();
  disp.brightness(7); // яркость, 0 - 7 (минимум - максимум)

  pinMode(MOSFET1, OUTPUT);      //MOSFET#1
  pinMode(MOSFET2, OUTPUT);      //MOSFET#2
  pinMode(MOSFET3, OUTPUT);      //MOSFET#3
  pinMode(BUTTON, INPUT);       //BUTTON PIN
  pinMode(BUZZER, OUTPUT);      //BUZZER
  pinMode(14, OUTPUT);          //GND
  digitalWrite (14, LOW);

  beeper(250);

  Serial.begin(115200);
  Wire.begin();

  if (!bme.begin())
  {
    disp.clear();
    disp.displayInt(280);
    Serial.println("BMP280 NOT FOUND!");
    while (1)
    {
    }
  }

  if (DEBUG_OUT)
  {
    Serial.println( String (millis() - millisshift) + ":BMP280 initialised:");
  }


  int status = IMU.begin();
  if (status < 0)
  {
    Serial.println("IMU NOT FOUND!");
    Serial.print("Status:");
    Serial.println(status);
    disp.clear();
    disp.displayInt(9250);
    while (1)
    {
    }
  }
  if (DEBUG_OUT)
  {
    Serial.println( String (millis() - millisshift) + ":IMU-9250 initialised:");
  }


  for (int q = 0; q < 16; q++)
  {
    byte testbyte = random(255);
    driveD.write(32500 + q, testbyte);

    if (driveD.read(32500 + q) != testbyte)
    {
      Serial.println("EXTERNAL EEPROM ERROR!");
      disp.clear();
      disp.displayInt(9999);
      while (1)
      {
      }
    }
  }

  if (DEBUG_OUT)
  {
    Serial.println( String (millis() - millisshift) + ":FRAM initialised:");
  }

  beeper(250);

  int8_t welcome_banner[] = {
    _H,
    _E,
    _L,
    _L,
    _O,
    _empty,
    _empty,
  };
  disp.clear();
  disp.runningString(welcome_banner, sizeof(welcome_banner), 200);

  SEALEVELPRESSURE_HPA = bme.readPressure() / 100.0;

  // setting the accelerometer full scale range to +/-16G
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);

  if (DEBUG_MOSFET)
    test_mosfets();
}
//------------------------------------------------------------------------------
void loop()
{
  if (digitalRead(BUTTON))
  {
    LOGonOSD();
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  //                         Тут у нас будет часть про ожидание пуска                              //
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  disp.clear();
  disp.displayInt(bme.readAltitude(SEALEVELPRESSURE_HPA));
  delay(1000);
  disp.clear();
  disp.displayInt(bme.readPressure() * 0.00750062);
  delay(1000);

  toLog("Pressure = " + String(bme.readPressure()) + " Pa");



  Serial.print("PackSize:");
  Serial.println(PackSize);




  while (!digitalRead(BUTTON))
  {
    disp.clear();
    disp.displayByte(_S, _E, _N, _D);

    getInfo2();
    fromLog();
    disp.clear();
    delay(500);
    beeper(50);
  }

  disp.clear();
  disp.displayByte(_F, _L, _Y, _empty);
  beeper(6000);

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  //                                     FIRST STAGE                                               //
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  EEXPos = 0;
  Maxspeed = 0;
  Apogee = 0;
  SEALEVELPRESSURE_HPA = bme.readPressure() / 100.0;
  millisshift = millis();
  toLog("Start Logging");

  disp.clear();

  EEPROM.put(945, Apogee);
  EEPROM.put(950, Maxspeed);

  for (int FSTage = 1; FSTage <= Cycles; FSTage++)
  {
    Start2 = millis();

    getdata();      // Получаем данные с датчиков в структуру
    Writelog();     // Пишем данные в EEPROM
    fallingSense(); // Не падаем ли?

    if (Fallen and !MOSFET_1_IS_FIRED)

    {
      LANDING_PROCEDURE();
    }

    delay(42);
    Finish2 = millis();
    routineTime = Finish2 - Start2;

  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  //                                     PRAYING FOR RECOVERY                                      //
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  toLog("Finish Logging");

  EEPROM.put(950, Maxspeed);
  toLog("Maximum Speed " + String(Maxspeed));

  getInfo2();
  fromLog();

  disp.clear();
  disp.displayInt(Apogee);

  while (1)
  {
    beeper(1000);
    delay(1000);
    beeper(1000);
    delay(1000);

    if (digitalRead(BUTTON))
      break;
  }
  while (1)
    ;
}
///////////////////////////////////////////////////////////////////////////////////////////////////
