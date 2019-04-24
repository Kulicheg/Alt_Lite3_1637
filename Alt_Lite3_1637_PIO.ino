//I2C device found at address 0x3C !  OLED
//I2C device found at address 0x50 !  EEPROM
//I2C device found at address 0x58 !  EEPROM
//I2C device found at address 0x68 !  AXL
//I2C device found at address 0x76 !  BMP
// 950 - 953 - Max Speed
// 947 - 947 - NumRec
// 945 - 946 - Apogee
//A,B,C,D,E,F,G,H,J,L,N,O,P,S,U,Y,a,b,c,d,e,f,h,i,j,l,n,o,q,r,t,u,y,dash,under,equal,empty,degree

#include <Arduino.h>
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
#define BUZZER 14

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
float fbx, fby, fbz;
float fbax, fbay, fbaz;
float SpeedDim [3];

int bufwrite;
int msgCount;
int Cycles;
bool Fallen;
byte NumRec;
int xCal, yCal, zCal, axCal, ayCal, azCal;
long int Start, Start2, Finish, Finish2, routineTime;
long int FirstTime, SecondTime, oldAltitude, newAltitude, SecondTimeM, FirstTimeM;
byte MOSFET_1, MOSFET_2, MOSFET_3;
bool MOSFET_1_IS_FIRED, MOSFET_2_IS_FIRED, MOSFET_3_IS_FIRED;

int Maxspeed;
float Altitude;
int Apogee;
int Temperature;
float Speed;
int SpeedAvg;

byte JournalSize;
byte currentByte;
byte header[4] = {170, 171, 186, 187};
byte command;

struct telemetrystruct
{
  float fbax, fbay, fbaz;
  float fbx, fby, fbz;
  long int Pressure;
  int Temperature;
  float Altitude;
  float Speed;
  int SpeedAvg;
};

struct SystemLog
{
  unsigned long timestamp;
  char message[25];
};

struct telemetrystruct telemetry;
struct SystemLog capitansLog;

////////////////////////////////////////////////////////////////////////////////////////////////
//                                   PROCEDURES & FUNCTIONS                                   //
////////////////////////////////////////////////////////////////////////////////////////////////

void sendheader(byte command)
{
  for (byte q = 0; q < 4; q++)
  {
    Serial.write(header[q]);
  }
  Serial.write(command);
}
////////////////////////////////////////////////////////////////////////////////////////////////

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
  //tone (3, 1000, 5);
  for (int q = 0; q < Cycles * PackSize; q++)
  {
    byte Sendbyte = driveD.read(q);
    Serial.write(Sendbyte);
  }
  Serial.flush();
  delay(200);
  ////////////////////////////////////////////////////////////////////////////////////////////////

  /////////////////////////////////////SEND JOURNAL///////////////////////////////////////////////
  //tone (3, 1000, 5);
  for (int q = 0; q < NumRec * JournalSize; q++)
  {
    byte Sendbyte = EEPROM.read(q);
    Serial.write(Sendbyte);
  }
  Serial.flush();
  delay(200);
  /////////////////////////////////////SEND DUMP////////////////////////////////////////////////
  //tone (3, 1000, 5);
  for (int q = 945; q < 1024; q++)
  {
    byte Sendbyte = EEPROM.read(q);
    Serial.write(Sendbyte);
  }
  Serial.flush();
  ////////////////////////////////////////////////////////////////////////////////////////////////
  //tone (3, 500, 5);
}
////////////////////////////////////////////////////////////////////////////////////////////////

float speedOmeter()
{
  Alt2 = bme.readAltitude(SEALEVELPRESSURE_HPA);



  FirstTimeM = millis();

  if (FirstTimeM - SecondTimeM > 98)
  {

    Speed = (Alt2 - Alt1) / (FirstTimeM - SecondTimeM) * 1000;

    Alt1 = Alt2;
    SecondTimeM = millis();

 for (int q = 0; q<2; q++)
{
  SpeedDim [q] = SpeedDim[q+1];
 }
SpeedDim[2] = Speed;

SpeedAvg = round((SpeedDim[0] + SpeedDim[1] + SpeedDim[2]) / 3);

    if (Speed > Maxspeed)
    {
      Maxspeed = Speed;
    }
  }
  return Speed;
}
////////////////////////////////////////////////////////////////////////////////////////////////

void toLog(String message)
{

  if (EEPOS < 928)
  {
    int eventSize = sizeof(capitansLog);
    char event[25];
    message.toCharArray(event, 25);
    memcpy(capitansLog.message, event, 25);
    capitansLog.timestamp = millis();
    EEPROM.put(EEPOS, capitansLog);
    EEPOS = EEPOS + eventSize;
    NumRec = EEPOS / eventSize;
    EEPROM.write(947, NumRec);
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////

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
////////////////////////////////////////////////////////////////////////////////////////////////

void getdata()
{
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  //                                     AXL SENSORS COLLECTION                                    //
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  IMU.readSensor();

  fbx = IMU.getAccelX_mss();
  fby = IMU.getAccelY_mss();
  fbz = IMU.getAccelZ_mss();
  
  fbax = IMU.getGyroX_rads();
  fbay = IMU.getGyroY_rads();
  fbaz = IMU.getGyroZ_rads();

  fbax = fbax * 57.2958;
  fbay = fbay * 57.2958;
  fbaz = fbaz * 57.2958;


  ///////////////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  //                                     OTHER SENSORS COLLECTION                                  //
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  Pressure = bme.readPressure();

  Altitude = round(bme.readAltitude(SEALEVELPRESSURE_HPA));

  Temperature = bme.readTemperature();

  ///////////////////////////////////////////////////////////////////////////////////////////////////

  telemetry.fbx = fbx;
  telemetry.fby = fby;
  telemetry.fbz = fbz;

  telemetry.fbax = fbax;
  telemetry.fbay = fbay;
  telemetry.fbaz = fbaz;

  telemetry.Pressure = Pressure;
  telemetry.Temperature = Temperature;
  telemetry.Altitude = Altitude;
  telemetry.Speed = speedOmeter();
  telemetry.SpeedAvg = SpeedAvg;


  ///////////////////////////////////////////////////////////////////////////////////////////////////
}

void fallingSense()
{
  if (!Fallen)
  {
    if (Speed < 3 and Altitude > 30)
    {
      Fallen = true;
      toLog("LOW_Speed!A=" + String(Altitude) + "S=" + String(Speed));
    }

    if ((oldAltitude - newAltitude) > 2)
    {
      Apogee = oldAltitude;
      toLog("Falling detected " + String(Apogee));
      EEPROM.put(945, Apogee);
      Fallen = true;
    }

    FirstTime = millis();
    if (FirstTime - SecondTime >= 1000)
    {
      SecondTime = millis();
      oldAltitude = newAltitude;
      newAltitude = Altitude;
    }
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////

void MOSFET_FIRE(byte Number)
{
  switch (Number)
  {
  case 1:
    disp.clear();
    tone(BUZZER, 500, 10);
    digitalWrite(MOSFET_1, HIGH);
    delay(250);
    digitalWrite(MOSFET_1, LOW);

    MOSFET_1_IS_FIRED = true;

    toLog("MOSFET_1 IS FIRED");

    break;

  case 2:
    disp.clear();
    tone(BUZZER, 700, 10);
    digitalWrite(MOSFET_2, HIGH);
    delay(250);
    digitalWrite(MOSFET_2, LOW);

    MOSFET_2_IS_FIRED = true;

    toLog("MOSFET_2 IS FIRED");
    break;

  case 3:
    disp.clear();
    tone(BUZZER, 900, 10);
    digitalWrite(MOSFET_3, HIGH);
    delay(250);
    digitalWrite(MOSFET_3, LOW);

    MOSFET_3_IS_FIRED = true;

    toLog("MOSFET_3 IS FIRED");
    break;
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////

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
////////////////////////////////////////////////////////////////////////////////////////////////

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

  Serial.println("  Alt\t Spd\t Prs\t Tmp\t fbx\t fby\t fbz\t gX\t gY\t gZ \t Vavg");
  Serial.println(" ");

  for (int Rec = 0; Rec < Cycles; Rec++)
  {
    for (int intern = 0; intern < PackSize; intern++)
    {
      Packet[intern] = driveD.read(EEXPos + intern);
    }

    memcpy(&telemetry, Packet, sizeof(telemetry));

    fbx = telemetry.fbx;
    fby = telemetry.fby;
    fbz = telemetry.fbz;

    
    fbax = telemetry.fbax;
    fbay = telemetry.fbay;
    fbaz = telemetry.fbaz;

    Pressure = telemetry.Pressure;
    Temperature = telemetry.Temperature;
    Altitude = telemetry.Altitude;
    Speed = telemetry.Speed;
    SpeedAvg = telemetry.SpeedAvg;

    Serial.print(round(Altitude));
    Serial.print("\t");
    Serial.print(round(Speed));
    Serial.print("\t");
    Serial.print(Pressure);
    Serial.print("\t");
    Serial.print(Temperature);
    Serial.print("\t");
    Serial.print(fbx, 2);
    Serial.print("\t");
    Serial.print(fby, 2);
    Serial.print("\t");
    Serial.print(fbz, 2);
    Serial.print("\t");
    Serial.print(round(fbax));
    Serial.print("\t");
    Serial.print(round(fbay));
    Serial.print("\t");
    Serial.println(round(fbaz));
    Serial.print("\t");
    Serial.println(SpeedAvg);

    EEXPos = EEXPos + PackSize;
  }
  Serial.println("-------------------------------------------------------------------------------------");
}
////////////////////////////////////////////////////////////////////////////////////////////////

void LANDING_PROCEDURE()
{
  toLog("LANDING_PROCEDURE (" + String(telemetry.Altitude) + ")");
  MOSFET_FIRE(1);
}
////////////////////////////////////////////////////////////////////////////////////////////////

void LOGonOSD()
{
  tone(BUZZER, 300, 5);
  EEPROM.get(945, Apogee);
  EEPROM.get(950, Maxspeed);

  disp.clear();
  disp.displayInt(Apogee);
  delay(6000);
  tone(BUZZER, 300, 5);
  disp.clear();
  disp.displayInt(Maxspeed);
  delay(6000);

  disp.clear();
  disp.displayInt(433);

  SendData();
}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{

  MOSFET_1_IS_FIRED = false;
  MOSFET_2_IS_FIRED = false;
  MOSFET_3_IS_FIRED = false;

  MOSFET_1 = 15;
  MOSFET_2 = 16;
  MOSFET_3 = 17;

  Fallen = false;
  Cycles = 100;
  Apogee = 0;
  Maxspeed = 0;

  JournalSize = sizeof(capitansLog);
  PackSize = sizeof(telemetry);

  disp.clear();
  disp.brightness(7); // яркость, 0 - 7 (минимум - максимум)

  pinMode(MOSFET_1, OUTPUT);     //MOSFET#1
  pinMode(MOSFET_2, OUTPUT);     //MOSFET#2
  pinMode(MOSFET_3, OUTPUT);     //MOSFET#3
  pinMode(LED_BUILTIN, OUTPUT);  //LED#13
  pinMode(BUTTON, INPUT_PULLUP); // BUTTON PIN

  Serial.begin(115200);
  Wire.begin();

  Serial.print("PackSize = ");
  Serial.println(PackSize);

  if (!bme.begin())
  {
    disp.clear();
    disp.displayInt(280);
    Serial.println("BMP280 NOT FOUND!");
    while (1)
    {
    }
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
  byte welcome_banner[] = {
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

  // setting the accelerometer full scale range to +/-8G
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);
}
//------------------------------------------------------------------------------

void loop()
{
  if (!digitalRead(BUTTON))
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
/*
  while (digitalRead(BUTTON))
  {
    disp.clear();
    disp.displayByte(_S, _E, _N, _D);

    getInfo2();
    fromLog();
    delay(1000);
  }
*/
  disp.clear();
  disp.displayByte(_F, _L, _Y, _empty);
  Serial.println("POEKHALI!");
  delay(2000);

  EEXPos = 0;

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  //                                     FIRST STAGE                                               //
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  Maxspeed = 0;
  toLog("Start Logging");
  disp.clear();

  for (int FSTage = 1; FSTage <= Cycles; FSTage++)
  {
    Start2 = millis();

    getdata();      // Получаем данные с датчиков в структуру
    Writelog();     // Пишем данные в EEPROM
    fallingSense(); // Не падаем ли?

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    tone(BUZZER, 200, 3);

    if (Fallen and !MOSFET_1_IS_FIRED)

    {
      LANDING_PROCEDURE();
    }

    delay(90);
    Finish2 = millis();
    routineTime = Finish2 - Start2;
    disp.displayInt(routineTime);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  //                                     PRAYING FOR RECOVERY                                      //
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  EEPROM.put(950, Maxspeed);
  toLog("Maximum Speed " + String(Maxspeed));

  toLog("Finish Logging");

  getInfo2();
  fromLog();

  disp.clear();
  disp.displayInt(Apogee);

  while (1)
  {
    tone(BUZZER, 1800, 1000);
    disp.displayInt(8888);
    delay(1000);
    tone(BUZZER, 2200, 1000);
    disp.clear();
    delay(1000);

    if (!digitalRead(BUTTON))
      break;
  }
  while (1)
  {
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////
