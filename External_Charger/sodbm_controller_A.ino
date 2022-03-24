#include <SPI.h>
#include <Arduino.h>
#include <due_can.h>
#include <due_wire.h>
#include "variant.h"
#include <Wire_EEPROM.h>
#include <Scheduler.h>
#include <elapsedMillis.h>

elapsedMillis timeElapsed;
elapsedMillis lrx;

#define Serial SerialUSB
template<class T> inline Print &operator <<(Print &obj, T arg) {
  obj.print(arg);  //Sets up Serial streaming Serial<<someshit;
  return obj;
}

class Field {
  public:
    uint16_t msb;
    uint16_t bits;
    uint32_t data;
    char* fieldName;
};

class Msg {
  public:
    char* moduleName;
    uint16_t msgID;
    uint64_t msg;
    Field fields[16];
    uint8_t field_count;
};

class StringTable {
  public:
    Msg messages[32];
};

class EEPROMvariables {
  public:
    uint8_t CANdo;        //Use Serial port if 2, CAN0 if 0 or CAN1 if 1
    char logfile[80];
    uint16_t transmitime = 0;
    boolean logger;
    uint32_t datarate = 500000;
    uint8_t goodEEPROM;
    CAN_FRAME outFrame;

};

StringTable stringTable;
EEPROMvariables myVars;
uint16_t page = 475;
uint16_t dummy;
char cmdBuffer[100];
char logstring[200];
char logstring2[300] = "";
char logstringt[200];
char logstringtl[200];
char logstringt1[200];
char logstringc1[200];
char logstringts[200];
char msgBuff[100];
float Version = 1.10;
int ptrBuffer;
short logcycle = 0;
boolean handlingEvent, debug;
int b1167 = 0x70;
int b142D = 0x02;
int reportedvolts = 0;
int reportedvoltspre = 0;
int tempA = 0;
int tempAint = 0;
int tempB = 0;
int tempBint = 0;
int tempC = 0;
int tempCint = 0;
int VACint = 0;
int AACint = 0;
int contcommand = 0;
float reportedamps = 0;
int VACint1 = 0;
int VACint2 = 0;
int reportvolts1 = 0;
int reportvolts2 = 0;
unsigned int timeout = 1000;

unsigned int incra = 1048575;
int chargevolts = 0;
float chargeamps = 0;
float battvolts = 333;
int chargevoltsint1 = 0;
int chargevoltsint2 = 0;
int chargecontrolint6 = 0;
int chargecontrolint7 = 0;
int chargeampsint = 0;
int battvoltsint = 0;
int reportedampsint = 0;



float acc = 0;
float pilot = 0;
float ampacity = 0;
float VAC = 0;
float AAC = 0;
float Hz = 0;
int sodbmdiagcycle = 1;
boolean sodbmreset = false;
int sodbmtoggle = 1;
int b207A = 0;
int b307A = 0;
int chargingstate=0;
int chargerStatus = 0;

CAN_FRAME inFrame;

void setup() {

  Wire.begin();
  Serial.begin(115200);

  EEPROM.setWPPin(19);
  EEPROM.read(page, myVars);
  if (myVars.goodEEPROM != 200)defaults();
  myVars.logger = false;


  initializeCAN();




  Scheduler.startLoop(ActiveDiagLoop);
  Scheduler.startLoop(StaticSimulatorLoop100ms);
  Scheduler.startLoop(StaticSimulatorLoop50ms);
  Scheduler.startLoop(StaticSimulatorLoop40ms);
  Scheduler.startLoop(OdometerSimulatorLoop);
  Scheduler.startLoop(GearShiftSimulatorLoop);
  Scheduler.startLoop(BECMStatusLoop);
  Scheduler.startLoop(SODBMControlLoop);
  Scheduler.startLoop(BECMVoltageLoop);
  Scheduler.startLoop(SysTimeSimulatorLoop);
  
  



}



void loop()
{
  while (SerialUSB.available()) {
    serialEvent();
  }
  sprintf(logstringtl, "%02X%02X", chargevoltsint1, chargevoltsint2);
  chargevolts = (float)strtol(logstringtl, NULL, 16);
  Serial<<"CV = ";
  Serial<<chargevolts;
  Serial<<" V          CC = ";
  Serial<<chargeampsint;
  Serial<<" A          Batt = ";
  Serial<<battvolts;
  Serial<<" V          AmpsOut = "; 
  Serial<<reportedamps;
  Serial<<" A          State = ";
  Serial<<chargingstate;
  Serial<<"\n"; 
  Serial<<"TempA = ";
  Serial<<tempA;
  Serial<<"     TempB = ";
  Serial<<tempB;
  Serial<<"     Temp C = ";
  Serial<<tempC;
  Serial<<"     Status = ";
  Serial<<chargerStatus;
  Serial<<"\n";
  
      reportedvolts = (reportedvoltspre & 0x1FF);
      reportedvolts = reportedvolts * 2;
      battvolts = reportedvolts/2;
      b207A = (reportedvolts & 0xFF00);
      b207A = b207A >> 8;
      b307A = (reportedvolts & 0x00FF);
    if (timeElapsed > timeout)
    {
      b1167 = 0x70;
      b142D = 0x02;
      chargingstate=0;
      chargeampsint = 0x00;
      chargevoltsint1 = 0x00;
      chargevoltsint2 = 0x00;
      chargecontrolint6 = 0x00;
      chargecontrolint7 = 0x00;
      contcommand = 0;
    }
    
    if (lrx > timeout)
    {
      pilot = 0;
        AAC = 0;
        VAC = 0;
        ampacity = 0;
        Hz = 0;
        tempAint = 0;
        tempA = 0;
        tempBint = 0;
        tempB = 0;
        tempCint = 0;
        tempC = 0;
    }


    myVars.outFrame.id = 0x105;
    myVars.outFrame.data.bytes[0] = reportvolts1;
    myVars.outFrame.data.bytes[1] = reportvolts2;
    myVars.outFrame.data.bytes[2] = reportedampsint;
    myVars.outFrame.data.bytes[3] = 0x00;
    myVars.outFrame.data.bytes[4] = 0x00;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0x00;
    myVars.outFrame.data.bytes[7] = chargerStatus;
    sendCAN(myVars.CANdo);
    delay(40);
    myVars.outFrame.id = 0x109;
    myVars.outFrame.data.bytes[0] = VACint1;
    myVars.outFrame.data.bytes[1] = VACint2;
    myVars.outFrame.data.bytes[2] = AACint;
    myVars.outFrame.data.bytes[3] = 0x00;
    myVars.outFrame.data.bytes[4] = 0x00;
    myVars.outFrame.data.bytes[5] = tempAint;
    myVars.outFrame.data.bytes[6] = tempBint;
    myVars.outFrame.data.bytes[7] = tempCint;
    sendCAN(myVars.CANdo);

  /*
  sprintf(logstring2, "%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;\n",
          batchadspl, hbattchar, hbattcoolin, mcon, hbattcurr, hbattvolt, ETE, hbatttemp, seqf,
          chacurrentf, torquedelivered, brakemode, estchatimef, motorRPM, MPH, throttle, brake, pwrt, acc, hvac, VAC, AAC, ampacity, Hz, mincell, maxcell);
  Serial << logstring2;
*/
  delay(60);
}

void ActiveDiagLoop()
{
  if (sodbmreset == true)
  {
    myVars.outFrame.id = 0x7E2;
    myVars.outFrame.data.bytes[0] = 0x04;
    myVars.outFrame.data.bytes[1] = 0x14;
    myVars.outFrame.data.bytes[2] = 0xFF;
    myVars.outFrame.data.bytes[3] = 0xFF;
    myVars.outFrame.data.bytes[4] = 0xFF;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0x00;
    myVars.outFrame.data.bytes[7] = 0x00;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
  }




  sodbmdiagcycle =  sodbmdiagcycle * sodbmtoggle;

  if (sodbmdiagcycle == 1)
  {
    //SODBM Diagnostic Request for AC Voltage
    myVars.outFrame.id = 0x7E2;
    myVars.outFrame.data.bytes[0] = 0x03;
    myVars.outFrame.data.bytes[1] = 0x22;
    myVars.outFrame.data.bytes[2] = 0x48;
    myVars.outFrame.data.bytes[3] = 0x5E;
    myVars.outFrame.data.bytes[4] = 0x00;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0x00;
    myVars.outFrame.data.bytes[7] = 0x00;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);

  }

  else if (sodbmdiagcycle == 2)
  {
    //SODBM Diagnostic Request for AC Amps
    myVars.outFrame.id = 0x7E2;
    myVars.outFrame.data.bytes[0] = 0x03;
    myVars.outFrame.data.bytes[1] = 0x22;
    myVars.outFrame.data.bytes[2] = 0x48;
    myVars.outFrame.data.bytes[3] = 0x5F;
    myVars.outFrame.data.bytes[4] = 0x00;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0x00;
    myVars.outFrame.data.bytes[7] = 0x00;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);

  }

  else if (sodbmdiagcycle == 3)
  {
    //SODBM Diagnostic Request for Temp A
    myVars.outFrame.id = 0x7E2;
    myVars.outFrame.data.bytes[0] = 0x03;
    myVars.outFrame.data.bytes[1] = 0x22;
    myVars.outFrame.data.bytes[2] = 0xD1;
    myVars.outFrame.data.bytes[3] = 0x17;
    myVars.outFrame.data.bytes[4] = 0x00;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0x00;
    myVars.outFrame.data.bytes[7] = 0x00;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);

  }

  else if (sodbmdiagcycle == 4)
  {
    //SODBM Diagnostic Request for Temp B
    myVars.outFrame.id = 0x7E2;
    myVars.outFrame.data.bytes[0] = 0x03;
    myVars.outFrame.data.bytes[1] = 0x22;
    myVars.outFrame.data.bytes[2] = 0xD0;
    myVars.outFrame.data.bytes[3] = 0x0B;
    myVars.outFrame.data.bytes[4] = 0x00;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0x00;
    myVars.outFrame.data.bytes[7] = 0x00;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);

  }
  
    else if (sodbmdiagcycle == 5)
  {
    //SODBM Diagnostic Request for Temp C
    myVars.outFrame.id = 0x7E2;
    myVars.outFrame.data.bytes[0] = 0x03;
    myVars.outFrame.data.bytes[1] = 0x22;
    myVars.outFrame.data.bytes[2] = 0xD0;
    myVars.outFrame.data.bytes[3] = 0x0C;
    myVars.outFrame.data.bytes[4] = 0x00;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0x00;
    myVars.outFrame.data.bytes[7] = 0x00;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);

  }
  
  else if (sodbmdiagcycle == 6)
  {
    //SODBM Diagnostic Request for Status
    myVars.outFrame.id = 0x7E2;
    myVars.outFrame.data.bytes[0] = 0x03;
    myVars.outFrame.data.bytes[1] = 0x22;
    myVars.outFrame.data.bytes[2] = 0x48;
    myVars.outFrame.data.bytes[3] = 0x4F;
    myVars.outFrame.data.bytes[4] = 0x00;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0x00;
    myVars.outFrame.data.bytes[7] = 0x00;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);

  }
  delay(30);
}

void StaticSimulatorLoop100ms()
{
    //Exterior Temperature Message (100 ms)
    myVars.outFrame.id = 0x178;
    myVars.outFrame.data.bytes[0] = 0xC1;
    myVars.outFrame.data.bytes[1] = 0x2D;
    myVars.outFrame.data.bytes[2] = 0x00;
    myVars.outFrame.data.bytes[3] = 0x00;
    myVars.outFrame.data.bytes[4] = 0x8E;
    myVars.outFrame.data.bytes[5] = 0x4D;
    myVars.outFrame.data.bytes[6] = 0xB6;
    myVars.outFrame.data.bytes[7] = 0x4D;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    
    delay(45);
    
    //Charger's DC/DC converter command Disable (100 ms)
    myVars.outFrame.id = 0x42D;
    myVars.outFrame.data.bytes[0] = 0x00;
    myVars.outFrame.data.bytes[1] = b142D; //0x02 before charge, 0x42 on charge
    myVars.outFrame.data.bytes[2] = 0x4D;
    myVars.outFrame.data.bytes[3] = 0x00;
    myVars.outFrame.data.bytes[4] = 0x46;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0x00;
    myVars.outFrame.data.bytes[7] = 0x00;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(45);
}


void StaticSimulatorLoop50ms()
{
    //DC/DC Converter Low Voltage Set Point (50 ms)
    myVars.outFrame.id = 0x43E;
    myVars.outFrame.data.bytes[0] = 0x00;
    myVars.outFrame.data.bytes[1] = 0xA1;
    myVars.outFrame.data.bytes[2] = 0x00;
    myVars.outFrame.data.bytes[3] = 0x00;
    myVars.outFrame.data.bytes[4] = 0x00;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0x00;
    myVars.outFrame.data.bytes[7] = 0x00;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(13);
    
    //DC/DC Converter Main Status (50 ms)
    myVars.outFrame.id = 0x43D;
    myVars.outFrame.data.bytes[0] = 0x43;
    myVars.outFrame.data.bytes[1] = 0x0F;
    myVars.outFrame.data.bytes[2] = 0x91;
    myVars.outFrame.data.bytes[3] = 0x19;
    myVars.outFrame.data.bytes[4] = 0x08;
    myVars.outFrame.data.bytes[5] = 0x94;
    myVars.outFrame.data.bytes[6] = 0x02;
    myVars.outFrame.data.bytes[7] = 0xA9;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(13);
    
    //Some PCM/ECM BS (50 ms)
    myVars.outFrame.id = 0x43E;
    myVars.outFrame.data.bytes[0] = 0x00;
    myVars.outFrame.data.bytes[1] = 0xA1;
    myVars.outFrame.data.bytes[2] = 0x00;
    myVars.outFrame.data.bytes[3] = 0x00;
    myVars.outFrame.data.bytes[4] = 0x00;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0x00;
    myVars.outFrame.data.bytes[7] = 0x00;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(13);
}


void StaticSimulatorLoop40ms()
{
  //Main ECU Voltage Supply (40 ms)
    myVars.outFrame.id = 0x242;
    myVars.outFrame.data.bytes[0] = 0x04;//static
    myVars.outFrame.data.bytes[1] = 0xD0;//static
    myVars.outFrame.data.bytes[2] = 0x20;//static
    myVars.outFrame.data.bytes[3] = 0x0C;//static
    myVars.outFrame.data.bytes[4] = 0xE9;//static
    myVars.outFrame.data.bytes[5] = 0x28;
    myVars.outFrame.data.bytes[6] = 0x04;
    myVars.outFrame.data.bytes[7] = 0x01;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(35);
    
    myVars.outFrame.id = 0x242;
    myVars.outFrame.data.bytes[0] = 0x04;//static
    myVars.outFrame.data.bytes[1] = 0xD0;//static
    myVars.outFrame.data.bytes[2] = 0x20;//static
    myVars.outFrame.data.bytes[3] = 0x0C;//static
    myVars.outFrame.data.bytes[4] = 0xE9;//static
    myVars.outFrame.data.bytes[5] = 0x28;
    myVars.outFrame.data.bytes[6] = 0x00;
    myVars.outFrame.data.bytes[7] = 0x00;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(35);
    
    myVars.outFrame.id = 0x242;
    myVars.outFrame.data.bytes[0] = 0x04;//static
    myVars.outFrame.data.bytes[1] = 0xD0;//static
    myVars.outFrame.data.bytes[2] = 0x20;//static
    myVars.outFrame.data.bytes[3] = 0x0C;//static
    myVars.outFrame.data.bytes[4] = 0xE9;//static
    myVars.outFrame.data.bytes[5] = 0x2C;
    myVars.outFrame.data.bytes[6] = 0x05;
    myVars.outFrame.data.bytes[7] = 0x01;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(35);
    
    myVars.outFrame.id = 0x242;
    myVars.outFrame.data.bytes[0] = 0x04;//static
    myVars.outFrame.data.bytes[1] = 0xD0;//static
    myVars.outFrame.data.bytes[2] = 0x20;//static
    myVars.outFrame.data.bytes[3] = 0x0C;//static
    myVars.outFrame.data.bytes[4] = 0xE9;//static
    myVars.outFrame.data.bytes[5] = 0x28;
    myVars.outFrame.data.bytes[6] = 0x00;
    myVars.outFrame.data.bytes[7] = 0x00;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(35);
    
    myVars.outFrame.id = 0x242;
    myVars.outFrame.data.bytes[0] = 0x04;//static
    myVars.outFrame.data.bytes[1] = 0xD0;//static
    myVars.outFrame.data.bytes[2] = 0x20;//static
    myVars.outFrame.data.bytes[3] = 0x0C;//static
    myVars.outFrame.data.bytes[4] = 0xE9;//static
    myVars.outFrame.data.bytes[5] = 0x28;
    myVars.outFrame.data.bytes[6] = 0x04;
    myVars.outFrame.data.bytes[7] = 0x01;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(35);
    
    myVars.outFrame.id = 0x242;
    myVars.outFrame.data.bytes[0] = 0x04;//static
    myVars.outFrame.data.bytes[1] = 0xD0;//static
    myVars.outFrame.data.bytes[2] = 0x20;//static
    myVars.outFrame.data.bytes[3] = 0x0C;//static
    myVars.outFrame.data.bytes[4] = 0xE9;//static
    myVars.outFrame.data.bytes[5] = 0x2C;
    myVars.outFrame.data.bytes[6] = 0x02;
    myVars.outFrame.data.bytes[7] = 0x00;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(35);
    
    myVars.outFrame.id = 0x242;
    myVars.outFrame.data.bytes[0] = 0x04;//static
    myVars.outFrame.data.bytes[1] = 0xD0;//static
    myVars.outFrame.data.bytes[2] = 0x20;//static
    myVars.outFrame.data.bytes[3] = 0x0C;//static
    myVars.outFrame.data.bytes[4] = 0xE9;//static
    myVars.outFrame.data.bytes[5] = 0x28;
    myVars.outFrame.data.bytes[6] = 0x00;
    myVars.outFrame.data.bytes[7] = 0x00;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(35);
    
    myVars.outFrame.id = 0x242;
    myVars.outFrame.data.bytes[0] = 0x04;//static
    myVars.outFrame.data.bytes[1] = 0xD0;//static
    myVars.outFrame.data.bytes[2] = 0x20;//static
    myVars.outFrame.data.bytes[3] = 0x0C;//static
    myVars.outFrame.data.bytes[4] = 0xE9;//static
    myVars.outFrame.data.bytes[5] = 0x2C;
    myVars.outFrame.data.bytes[6] = 0x04;
    myVars.outFrame.data.bytes[7] = 0x01;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(35);
    
    myVars.outFrame.id = 0x242;
    myVars.outFrame.data.bytes[0] = 0x04;//static
    myVars.outFrame.data.bytes[1] = 0xD0;//static
    myVars.outFrame.data.bytes[2] = 0x20;//static
    myVars.outFrame.data.bytes[3] = 0x0C;//static
    myVars.outFrame.data.bytes[4] = 0xE9;//static
    myVars.outFrame.data.bytes[5] = 0x28;
    myVars.outFrame.data.bytes[6] = 0x00;
    myVars.outFrame.data.bytes[7] = 0x00;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(35);
    
    myVars.outFrame.id = 0x242;
    myVars.outFrame.data.bytes[0] = 0x04;//static
    myVars.outFrame.data.bytes[1] = 0xD0;//static
    myVars.outFrame.data.bytes[2] = 0x20;//static
    myVars.outFrame.data.bytes[3] = 0x0C;//static
    myVars.outFrame.data.bytes[4] = 0xE9;//static
    myVars.outFrame.data.bytes[5] = 0x28;
    myVars.outFrame.data.bytes[6] = 0x05;
    myVars.outFrame.data.bytes[7] = 0x01;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(35);
}


void OdometerSimulatorLoop()
{
  //Odometer Sequence (100 ms)
    myVars.outFrame.id = 0x430;
    myVars.outFrame.data.bytes[0] = 0x03;
    myVars.outFrame.data.bytes[1] = 0x00;
    myVars.outFrame.data.bytes[2] = 0xB7;
    myVars.outFrame.data.bytes[3] = 0x1A;
    myVars.outFrame.data.bytes[4] = 0x40;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0xC0;
    myVars.outFrame.data.bytes[7] = 0x00;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(95);
    
    myVars.outFrame.id = 0x430;
    myVars.outFrame.data.bytes[0] = 0x03;
    myVars.outFrame.data.bytes[1] = 0x00;
    myVars.outFrame.data.bytes[2] = 0xB7;
    myVars.outFrame.data.bytes[3] = 0x1A;
    myVars.outFrame.data.bytes[4] = 0x40;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0x80;
    myVars.outFrame.data.bytes[7] = 0x04;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(95);
    
    myVars.outFrame.id = 0x430;
    myVars.outFrame.data.bytes[0] = 0x03;
    myVars.outFrame.data.bytes[1] = 0x00;
    myVars.outFrame.data.bytes[2] = 0xB7;
    myVars.outFrame.data.bytes[3] = 0x1A;
    myVars.outFrame.data.bytes[4] = 0x40;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0x80;
    myVars.outFrame.data.bytes[7] = 0x06;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(95);
    
    myVars.outFrame.id = 0x430;
    myVars.outFrame.data.bytes[0] = 0x03;
    myVars.outFrame.data.bytes[1] = 0x00;
    myVars.outFrame.data.bytes[2] = 0xB7;
    myVars.outFrame.data.bytes[3] = 0x1A;
    myVars.outFrame.data.bytes[4] = 0x40;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0xC0;
    myVars.outFrame.data.bytes[7] = 0x00;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(95);
    
    myVars.outFrame.id = 0x430;
    myVars.outFrame.data.bytes[0] = 0x03;
    myVars.outFrame.data.bytes[1] = 0x00;
    myVars.outFrame.data.bytes[2] = 0xB7;
    myVars.outFrame.data.bytes[3] = 0x1A;
    myVars.outFrame.data.bytes[4] = 0x40;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0x80;
    myVars.outFrame.data.bytes[7] = 0x04;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(95);
    
    myVars.outFrame.id = 0x430;
    myVars.outFrame.data.bytes[0] = 0x03;
    myVars.outFrame.data.bytes[1] = 0x00;
    myVars.outFrame.data.bytes[2] = 0xB7;
    myVars.outFrame.data.bytes[3] = 0x1A;
    myVars.outFrame.data.bytes[4] = 0x40;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0x80;
    myVars.outFrame.data.bytes[7] = 0x04;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(95);
}

void GearShiftSimulatorLoop ()
{
    //Gear Shift Simulator Loop (20 ms) 
    myVars.outFrame.id = 0x230;
    myVars.outFrame.data.bytes[0] = 0x01; //Static
    myVars.outFrame.data.bytes[1] = 0x00; //Static
    myVars.outFrame.data.bytes[2] = 0x00;
    myVars.outFrame.data.bytes[3] = 0xFC;
    myVars.outFrame.data.bytes[4] = 0x7B;
    myVars.outFrame.data.bytes[5] = 0x03;
    myVars.outFrame.data.bytes[6] = 0x78;
    myVars.outFrame.data.bytes[7] = 0x6C;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(15);
    
    myVars.outFrame.id = 0x230;
    myVars.outFrame.data.bytes[0] = 0x01; //Static
    myVars.outFrame.data.bytes[1] = 0x00; //Static
    myVars.outFrame.data.bytes[2] = 0x01;
    myVars.outFrame.data.bytes[3] = 0xFB;
    myVars.outFrame.data.bytes[4] = 0x87;
    myVars.outFrame.data.bytes[5] = 0x06;
    myVars.outFrame.data.bytes[6] = 0x68;
    myVars.outFrame.data.bytes[7] = 0x7C;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(15);
    
    myVars.outFrame.id = 0x230;
    myVars.outFrame.data.bytes[0] = 0x01; //Static
    myVars.outFrame.data.bytes[1] = 0x00; //Static
    myVars.outFrame.data.bytes[2] = 0x02;
    myVars.outFrame.data.bytes[3] = 0xFA;
    myVars.outFrame.data.bytes[4] = 0x52;
    myVars.outFrame.data.bytes[5] = 0x03;
    myVars.outFrame.data.bytes[6] = 0x9F;
    myVars.outFrame.data.bytes[7] = 0x8C;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(15);
    
    myVars.outFrame.id = 0x230;
    myVars.outFrame.data.bytes[0] = 0x01; //Static
    myVars.outFrame.data.bytes[1] = 0x00; //Static
    myVars.outFrame.data.bytes[2] = 0x03;
    myVars.outFrame.data.bytes[3] = 0xF9;
    myVars.outFrame.data.bytes[4] = 0x75;
    myVars.outFrame.data.bytes[5] = 0x04;
    myVars.outFrame.data.bytes[6] = 0x7A;
    myVars.outFrame.data.bytes[7] = 0x9C;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(15);
    
    myVars.outFrame.id = 0x230;
    myVars.outFrame.data.bytes[0] = 0x01; //Static
    myVars.outFrame.data.bytes[1] = 0x00; //Static
    myVars.outFrame.data.bytes[2] = 0x04;
    myVars.outFrame.data.bytes[3] = 0xF8;
    myVars.outFrame.data.bytes[4] = 0x6C;
    myVars.outFrame.data.bytes[5] = 0x0C;
    myVars.outFrame.data.bytes[6] = 0x7A;
    myVars.outFrame.data.bytes[7] = 0xAC;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(15);
    
    myVars.outFrame.id = 0x230;
    myVars.outFrame.data.bytes[0] = 0x01; //Static
    myVars.outFrame.data.bytes[1] = 0x00; //Static
    myVars.outFrame.data.bytes[2] = 0x05;
    myVars.outFrame.data.bytes[3] = 0xF7;
    myVars.outFrame.data.bytes[4] = 0xDB;
    myVars.outFrame.data.bytes[5] = 0x03;
    myVars.outFrame.data.bytes[6] = 0x13;
    myVars.outFrame.data.bytes[7] = 0xBC;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(15);
    
    myVars.outFrame.id = 0x230;
    myVars.outFrame.data.bytes[0] = 0x01; //Static
    myVars.outFrame.data.bytes[1] = 0x00; //Static
    myVars.outFrame.data.bytes[2] = 0x06;
    myVars.outFrame.data.bytes[3] = 0xF6;
    myVars.outFrame.data.bytes[4] = 0xED;
    myVars.outFrame.data.bytes[5] = 0x03;
    myVars.outFrame.data.bytes[6] = 0x00;
    myVars.outFrame.data.bytes[7] = 0xCC;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(15);
    
    myVars.outFrame.id = 0x230;
    myVars.outFrame.data.bytes[0] = 0x01; //Static
    myVars.outFrame.data.bytes[1] = 0x00; //Static
    myVars.outFrame.data.bytes[2] = 0x07;
    myVars.outFrame.data.bytes[3] = 0xF5;
    myVars.outFrame.data.bytes[4] = 0x5D;
    myVars.outFrame.data.bytes[5] = 0x06;
    myVars.outFrame.data.bytes[6] = 0x8C;
    myVars.outFrame.data.bytes[7] = 0xDC;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(15);
    
    myVars.outFrame.id = 0x230;
    myVars.outFrame.data.bytes[0] = 0x01; //Static
    myVars.outFrame.data.bytes[1] = 0x00; //Static
    myVars.outFrame.data.bytes[2] = 0x08;
    myVars.outFrame.data.bytes[3] = 0xF4;
    myVars.outFrame.data.bytes[4] = 0x86;
    myVars.outFrame.data.bytes[5] = 0x05;
    myVars.outFrame.data.bytes[6] = 0x63;
    myVars.outFrame.data.bytes[7] = 0xEC;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(15);
    
    myVars.outFrame.id = 0x230;
    myVars.outFrame.data.bytes[0] = 0x01; //Static
    myVars.outFrame.data.bytes[1] = 0x00; //Static
    myVars.outFrame.data.bytes[2] = 0x09;
    myVars.outFrame.data.bytes[3] = 0xF3;
    myVars.outFrame.data.bytes[4] = 0x65;
    myVars.outFrame.data.bytes[5] = 0x03;
    myVars.outFrame.data.bytes[6] = 0x85;
    myVars.outFrame.data.bytes[7] = 0xFC;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(15);
    
    myVars.outFrame.id = 0x230;
    myVars.outFrame.data.bytes[0] = 0x01; //Static
    myVars.outFrame.data.bytes[1] = 0x00; //Static
    myVars.outFrame.data.bytes[2] = 0x0A;
    myVars.outFrame.data.bytes[3] = 0xF2;
    myVars.outFrame.data.bytes[4] = 0x04;
    myVars.outFrame.data.bytes[5] = 0x06;
    myVars.outFrame.data.bytes[6] = 0xF2;
    myVars.outFrame.data.bytes[7] = 0x0C;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(15);
    
    myVars.outFrame.id = 0x230;
    myVars.outFrame.data.bytes[0] = 0x01; //Static
    myVars.outFrame.data.bytes[1] = 0x00; //Static
    myVars.outFrame.data.bytes[2] = 0x0B;
    myVars.outFrame.data.bytes[3] = 0xF1;
    myVars.outFrame.data.bytes[4] = 0x61;
    myVars.outFrame.data.bytes[5] = 0x05;
    myVars.outFrame.data.bytes[6] = 0x95;
    myVars.outFrame.data.bytes[7] = 0x1C;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(15);
    
    myVars.outFrame.id = 0x230;
    myVars.outFrame.data.bytes[0] = 0x01; //Static
    myVars.outFrame.data.bytes[1] = 0x00; //Static
    myVars.outFrame.data.bytes[2] = 0x0C;
    myVars.outFrame.data.bytes[3] = 0xF0;
    myVars.outFrame.data.bytes[4] = 0xAE;
    myVars.outFrame.data.bytes[5] = 0x01;
    myVars.outFrame.data.bytes[6] = 0x4B;
    myVars.outFrame.data.bytes[7] = 0x2C;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(15);
    
    myVars.outFrame.id = 0x230;
    myVars.outFrame.data.bytes[0] = 0x01; //Static
    myVars.outFrame.data.bytes[1] = 0x00; //Static
    myVars.outFrame.data.bytes[2] = 0x0D;
    myVars.outFrame.data.bytes[3] = 0xEF;
    myVars.outFrame.data.bytes[4] = 0x94;
    myVars.outFrame.data.bytes[5] = 0x05;
    myVars.outFrame.data.bytes[6] = 0x60;
    myVars.outFrame.data.bytes[7] = 0x3C;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(15);
    
    myVars.outFrame.id = 0x230;
    myVars.outFrame.data.bytes[0] = 0x01; //Static
    myVars.outFrame.data.bytes[1] = 0x00; //Static
    myVars.outFrame.data.bytes[2] = 0x0E;
    myVars.outFrame.data.bytes[3] = 0xEE;
    myVars.outFrame.data.bytes[4] = 0x22;
    myVars.outFrame.data.bytes[5] = 0x07;
    myVars.outFrame.data.bytes[6] = 0xCF;
    myVars.outFrame.data.bytes[7] = 0x4C;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(15);
    
    myVars.outFrame.id = 0x230;
    myVars.outFrame.data.bytes[0] = 0x01; //Static
    myVars.outFrame.data.bytes[1] = 0x00; //Static
    myVars.outFrame.data.bytes[2] = 0x0F;
    myVars.outFrame.data.bytes[3] = 0xED;
    myVars.outFrame.data.bytes[4] = 0x16;
    myVars.outFrame.data.bytes[5] = 0x04;
    myVars.outFrame.data.bytes[6] = 0xDD;
    myVars.outFrame.data.bytes[7] = 0x5C;
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(15);
}

void BECMStatusLoop ()
{
    //Sets the charging status
    myVars.outFrame.id = 0x24C;
    myVars.outFrame.data.bytes[0] = 0x3A; //Coolant temp (x - 50) * 9 / 5 + 32;
    myVars.outFrame.data.bytes[1] = 0xA0; //static
    myVars.outFrame.data.bytes[2] = 0x80; //static
    myVars.outFrame.data.bytes[3] = 0x01; //static
    myVars.outFrame.data.bytes[4] = 0x77; //static
    myVars.outFrame.data.bytes[5] = 0x00; //static
    myVars.outFrame.data.bytes[6] = 0x4C; //static
    myVars.outFrame.data.bytes[7] = chargingstate; // 0 = "Not Ready", 1 = "Charge Wait", 2 = "Ready", 3 = "Charging", 4 = "Charge Complete", 5 = "Fault"
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(90);
}

void SODBMControlLoop ()
{
  
  //Voltage and Current Commands (50 ms)
    myVars.outFrame.id = 0x2E4;
    myVars.outFrame.data.bytes[0] = 0x05; //static
    myVars.outFrame.data.bytes[1] = 0x9F; //static
    myVars.outFrame.data.bytes[2] = 0x00; //static 
    myVars.outFrame.data.bytes[3] = chargeampsint; //Current / 10
    myVars.outFrame.data.bytes[4] = chargevoltsint1; //Voltage
    myVars.outFrame.data.bytes[5] = chargevoltsint2; //Voltage
    myVars.outFrame.data.bytes[6] = chargecontrolint6; //something 
    myVars.outFrame.data.bytes[7] = chargecontrolint7; // 0 = "Charge Contactors Open", 3 = "Charge Contactors Closed"
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(40);
}

void BECMVoltageLoop ()
{
  
  
  
    //Reports Battery Voltage
    myVars.outFrame.id = 0x07A;
    myVars.outFrame.data.bytes[0] = 0xBA; //Static
    myVars.outFrame.data.bytes[1] = 0x99; //static
    myVars.outFrame.data.bytes[2] = b207A; //Voltage/2
    myVars.outFrame.data.bytes[3] = b307A; //Voltage/2
    myVars.outFrame.data.bytes[4] = 0xB9; //static
    myVars.outFrame.data.bytes[5] = 0x78; //static
    myVars.outFrame.data.bytes[6] = 0x00; //static
    myVars.outFrame.data.bytes[7] = 0x00; //static
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(3);
    
        //PCM 0x0167 
    myVars.outFrame.id = 0x167;
    myVars.outFrame.data.bytes[0] = b1167; 
    myVars.outFrame.data.bytes[1] = 0x80; //static
    myVars.outFrame.data.bytes[2] = 0x00; //Static
    myVars.outFrame.data.bytes[3] = 0x01; //Static
    myVars.outFrame.data.bytes[4] = 0xFF; //static
    myVars.outFrame.data.bytes[5] = 0xE0; //static
    myVars.outFrame.data.bytes[6] = 0x00; //static
    myVars.outFrame.data.bytes[7] = 0x00; //static
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(3);
    
}

void SysTimeSimulatorLoop ()
{
  //SysTimeSimulator
    myVars.outFrame.id = 0x40A;
    myVars.outFrame.data.bytes[0] = 0xC0; //Static
    myVars.outFrame.data.bytes[1] = 0x00; //static
    myVars.outFrame.data.bytes[2] = 0x3E; //Static
    myVars.outFrame.data.bytes[3] = 0x41; //Static
    myVars.outFrame.data.bytes[4] = 0x13; //static
    myVars.outFrame.data.bytes[5] = 0xA5; //static
    myVars.outFrame.data.bytes[6] = 0xCE; //static
    myVars.outFrame.data.bytes[7] = 0x39; //static
    myVars.outFrame.length = 8;
    myVars.outFrame.fid = 0;
    myVars.outFrame.rtr = 1;
    myVars.outFrame.priority = 0;
    myVars.outFrame.extended = false;
    Can0.sendFrame(myVars.outFrame);
    delay(10);
}





void serialEventRun(void)
{
  if (Serial.available())serialEvent(); //If Serial event interrupt is on USB port, go check for keyboard input

}

void serialEvent() {
  int incoming;

  incoming = Serial.read();
  if (incoming == -1) { //false alarm....
    return;
  }

  if (incoming == 10 || incoming == 13) { //command done. Parse it.
    handleConsoleCmd();
    ptrBuffer = 0; //reset line counter once the line has been processed
  } else {
    cmdBuffer[ptrBuffer++] = (unsigned char) incoming;
    if (ptrBuffer > 79)
      ptrBuffer = 79;
  }
}

void handleConsoleCmd() {
  handlingEvent = true;


  if (ptrBuffer < 5 ) { //command is a single ascii character
    handleShortCmd();
  } else { //if cmd over 1 char then assume (for now) that it is a config line
    handleConfigCmd();
  }

  handlingEvent = false;
}

void handleConfigCmd() {
  int i;
  int newValue;

  if (ptrBuffer < 6) return; //4 digit command, =, value is at least 6 characters
  cmdBuffer[ptrBuffer] = 0; //make sure to null terminate
  String cmdString = String();
  unsigned char whichEntry = '0';
  i = 0;

  while (cmdBuffer[i] != '=' && i < ptrBuffer)
  {
    cmdString.concat(String(cmdBuffer[i++]));
  }
  i++; //skip the =
  if (i >= ptrBuffer)
  {
    Serial << ("Command needs a value..ie FILE=myfile.log");
    return; //or, we could use this to display the parameter instead of setting
  }

  // strtol() is able to parse also hex values (e.g. a string "0xCAFE"), useful for enable/disable by device id
  newValue = strtol((char *) (cmdBuffer + i), NULL, 0);
  cmdString.toUpperCase();


  if (cmdString == String("VOLTS") )
  {

  chargevolts = newValue; 
  chargevoltsint1 = (chargevolts & 0xFF00);
  chargevoltsint2 = (chargevolts & 0x00FF);
  chargevoltsint1 = chargevoltsint1 >> 8;



  }
  else if (cmdString == String("AMPS") )
  {
  chargeampsint=newValue;
  }
  
  else if (cmdString == String("MAMPS"))
  {
    chargeampsint=newValue;
    
    
  }


}

void handleShortCmd()
{
  uint8_t val;

  switch (cmdBuffer[0])
  {

    case '1':
      b1167 = 0x41;
      break;
    case '2':
      chargingstate = 1;
      break;
    case '3':
      chargingstate=2;
      chargeampsint = 0x01;
      //chargevoltsint1 = 0x01;
      //chargevoltsint2 = 0x05C;      
      chargevoltsint1 = 0x01;
      chargevoltsint2 = 0x68;
      chargecontrolint6 = 0x40;
      chargecontrolint7 = 0x00;
      
      break;
    case '4':
      b142D = 0x42;
      chargingstate=3;
      chargeampsint = 0x01;
      //chargevoltsint1 = 0x01;
      //chargevoltsint2 = 0x68;
      chargevoltsint1 = 0x01;
      chargevoltsint2 = 0x68;
      chargecontrolint6 = 0x40;
      chargecontrolint7 = 0x4E;
      break;
    case '5':
      chargingstate=4;
      chargeampsint = 0x00;
      chargevoltsint1 = 0x00;
      chargevoltsint2 = 0x00;
      chargecontrolint6 = 0x00;
      chargecontrolint7 = 0x00;
      break;

    case '6':

      if (sodbmtoggle == 0)
      {
        sodbmtoggle = 1;
        sodbmdiagcycle = 1;
      }
      else
      {
        sodbmtoggle = 0;
        pilot = 0;
        AAC = 0;
        VAC = 0;
        ampacity = 0;
        Hz = 0;
        tempAint = 0;
        tempA = 0;
        tempBint = 0;
        tempB = 0;
        tempCint = 0;
        tempC = 0;
        
      }
      sodbmdiagcycle =  sodbmdiagcycle * sodbmtoggle;


      break;

    case '7':
    if (sodbmreset == false)sodbmreset = true;
    else if (sodbmreset == true)sodbmreset = false;

     
      break;

    case '9':
      incra = 0;
      break;

    case '0':
      b1167 = 0x70;
      b142D = 0x02;
      chargingstate=0;
      chargeampsint = 0x00;
      chargevoltsint1 = 0x00;
      chargevoltsint2 = 0x00;
      chargecontrolint6 = 0x00;
      chargecontrolint7 = 0x00;
      break;



  }
  cmdBuffer[0] = 0;
}



void initializeCAN()
{
  pinMode(48, OUTPUT);
  
  Can1.enable();
  if (Can1.begin(myVars.datarate, 48))
  {
    Can1.setNumTXBoxes(2);
    Can1.setRXFilter(0x101, 0x7FF, false);
    //Can1.setRXFilter(0x109, 0x7FF, false);

    //Can1.setRXFilter(1,0,0, false);
    //Can1.setRXFilter(2,0,0, true);
    Can1.setGeneralCallback(handleFrame1);
    Can1.disable_tx_repeat();


    Serial << "\n\nUsing CAN1 - initialization completed at " << myVars.datarate << " \n";
  }
  else Serial.println("\nCAN1 initialization (sync) ERROR\n");


  pinMode(50, OUTPUT);
  Can0.enable();
  if (Can0.begin(myVars.datarate, 50))
  {
    
    //Can0.setRXFilter(0x700, 0x700, false);
    /*
    Can0.setRXFilter(0x07A, 0x7F0, false);
    Can0.setRXFilter(0x1E4, 0x7FF, false);
    //Can0.setRXFilter(0x07D, 0x7F7, false);
    //Can0.setRXFilter(0x075, 0x7FF, false);
    Can0.setRXFilter(0x160, 0x5F2, false);
    //Can0.setRXFilter(0x165, 0x7FF, false);
    //Can0.setRXFilter(0x204, 0x7FF, false);
    Can0.setRXFilter(0x117, 0x7FF, false);
    */
    Can0.setNumTXBoxes(6);
    Can0.setRXFilter(0X2EC, 0x7FF, false);
    Can0.setRXFilter(0x7EA, 0x7FF, false);
    

    
    Can0.setGeneralCallback(handleFrame);
    Serial << "\n\nUsing CAN0 - initialization completed at " << myVars.datarate << " \n";
  }
  else Serial.println("\nCAN0 initialization (sync) ERROR\n");

}

void handleFrame(CAN_FRAME * frame)

{
  
  
//Serial<<"Got one\n";
  if (frame->id == 0X7EA)
  {
    if (sodbmdiagcycle == 1)
    {
      //AC Voltage Input
      sprintf(logstringt, "%02X%02X", frame->data.bytes[4], frame->data.bytes[5]);
      VACint = (int)strtol(logstringt, NULL, 16);
      VAC = (float)strtol(logstringt, NULL, 16) / 100;
      sprintf(logstringt, "%02X", frame->data.bytes[4]);
      VACint1 = (int)strtol(logstringt, NULL, 16);
      sprintf(logstringt, "%02X", frame->data.bytes[5]);
      VACint2 = (int)strtol(logstringt, NULL, 16);
      sodbmdiagcycle = 2;
    }
    else if (sodbmdiagcycle == 2)
    {
      //AC Current Input
      sprintf(logstringt, "%02X", frame->data.bytes[4]);
      AACint = (int)strtol(logstringt, NULL, 16);
      AAC = (float)strtol(logstringt, NULL, 16);
      sodbmdiagcycle = 3;
    }
    else if (sodbmdiagcycle == 3)
    {
      //SODBM Temp A
      sprintf(logstringt, "%02X", frame->data.bytes[4]);
      tempAint = (int)strtol(logstringt, NULL, 16);
      tempA = tempAint - 40;
      
      sodbmdiagcycle = 4;
    }

    else if (sodbmdiagcycle == 4)
    {
      //SODBM Temp B
      sprintf(logstringt, "%02X", frame->data.bytes[4]);
      tempBint = (int)strtol(logstringt, NULL, 16);
      tempB = tempBint - 40;

      sodbmdiagcycle = 5;
    }
    
    else if (sodbmdiagcycle == 5)
    {
      //SODBM Temp C
      sprintf(logstringt, "%02X", frame->data.bytes[4]);
      tempCint = (int)strtol(logstringt, NULL, 16);
      tempC = tempCint-40;
      sodbmdiagcycle = 6;
    }
    
    else if (sodbmdiagcycle == 6)
    {
      //SODBM Status
      sprintf(logstringt, "%02X", frame->data.bytes[4]);
      chargerStatus = (int)strtol(logstringt, NULL, 16);
      sodbmdiagcycle = 1;
    }
    sodbmdiagcycle = sodbmdiagcycle * sodbmtoggle;   

    }
    
    else if (frame->id == 0X2EC)
    {
      lrx = 0;
      //SODBM Reported Voltage
      sprintf(logstringt, "%02X%02X", frame->data.bytes[4], frame->data.bytes[5]);
      reportedvoltspre  = (int) strtol(logstringt, NULL, 16);
      reportedvolts = (reportedvoltspre & 0x1FF);
      reportvolts1 = (reportedvolts & 0xFF00);
      reportvolts1 = reportvolts1 >> 8;
      reportvolts2 = (reportedvolts & 0x00FF);
      reportedvolts = reportedvolts * 2;
      b207A = (reportedvolts & 0xFF00);
      b207A = b207A >> 8;
      b307A = (reportedvolts & 0x00FF);
      
      //SODBM Reported Amps
      sprintf(logstringt, "%02X", frame->data.bytes[7]);
      reportedamps  = (float) strtol(logstringt, NULL, 16)/10;
      reportedampsint = (int) strtol(logstringt, NULL, 16);
      
    }
  
  





}

void handleFrame1(CAN_FRAME * frame1)

{
  //Serial<<"got it\n";
if (frame1->id == 0x0101)
{
      sprintf(logstringt, "%02X", frame1->data.bytes[0]);
      contcommand  = (int) strtol(logstringt, NULL, 16);
      
      sprintf(logstringt, "%02X", frame1->data.bytes[1]);
      chargevoltsint1 = (int) strtol(logstringt, NULL, 16);
      
      sprintf(logstringt, "%02X", frame1->data.bytes[2]);
      chargevoltsint2 = (int) strtol(logstringt, NULL, 16);
      
      sprintf(logstringt, "%02X", frame1->data.bytes[3]);
      chargeampsint = (int) strtol(logstringt, NULL, 16);
      
      sprintf(logstringt, "%02X", frame1->data.bytes[7]);
      sodbmreset = (int) strtol(logstringt, NULL, 16);
      
      if (contcommand == 0)
      {
      b1167 = 0x70;
      b142D = 0x02;
      chargingstate=0;
      chargeampsint = 0x00;
      chargevoltsint1 = 0x00;
      chargevoltsint2 = 0x00;
      chargecontrolint6 = 0x00;
      chargecontrolint7 = 0x00;
      }
      else if (contcommand == 1)
      {
        b1167 = 0x41;
      }
      else if (contcommand == 2)
      {
        chargingstate = 1;
      }
      else if (contcommand == 3)
      {
        chargingstate=2;
        chargecontrolint6 = 0x40;
        chargecontrolint7 = 0x00;
      }
      else if (contcommand == 4)
      {
        b142D = 0x42;
        chargingstate=3;
        chargecontrolint6 = 0x40;
        chargecontrolint7 = 0x4E;
      }
      else if (contcommand == 5)
      {
        chargingstate=4;
        chargeampsint = 0x00;
        chargevoltsint1 = 0x00;
        chargevoltsint2 = 0x00;
        chargecontrolint6 = 0x00;
        chargecontrolint7 = 0x00;
      }
        
 timeElapsed = 0;     
}


}



void sendCAN(int which)

{
 /* 
        char buffer[140];

        sprintf(buffer,"Sent msgID 0x%03X; %02X; %02X; %02X; %02X; %02X; %02X; %02X; %02X;\n", myVars.outFrame.id,
        myVars.outFrame.data.bytes[0], myVars.outFrame.data.bytes[1], myVars.outFrame.data.bytes[2],
        myVars.outFrame.data.bytes[3], myVars.outFrame.data.bytes[4], myVars.outFrame.data.bytes[5],
        myVars.outFrame.data.bytes[6], myVars.outFrame.data.bytes[7]);
        Serial<<buffer;
        Serial<<"\n";
     */
  //int time=millis();
  //Serial<<time;
  //Serial<<"\n";


  myVars.outFrame.length = 8;
  myVars.outFrame.fid = 0;
  myVars.outFrame.rtr = 1;
  myVars.outFrame.priority = 0;
  myVars.outFrame.extended = false;
  //int time=micros();
  //Serial<<time<<"\n"<<buffer<<"\n";

  //Serial<<"I'm running\n";

  Can1.sendFrame(myVars.outFrame);    //Mail it




}


void defaults()
{
  myVars.CANdo = 2;
  myVars.transmitime = 500;
  myVars.logger = false;
  myVars.outFrame.length = 8;  // Data payload 8 bytes
  myVars.outFrame.rtr = 1;  // Data payload 8 bytes
  myVars.goodEEPROM = 200;
  myVars.datarate = 500000;



}
