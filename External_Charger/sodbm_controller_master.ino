#include <SPI.h>
#include <Arduino.h>
#include <due_can.h>
#include <due_wire.h>
#include "variant.h"
#include <Wire_EEPROM.h>
#include <Scheduler.h>
#include <elapsedMillis.h>

elapsedMillis timeElapsed;
elapsedMillis v100lrx;
elapsedMillis v101lrx;
elapsedMillis v102lrx;

elapsedMillis mod1lrx;
elapsedMillis mod2lrx;
elapsedMillis mod3lrx;
elapsedMillis mod4lrx;
elapsedMillis mod5lrx;

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
char logstringveh[200];
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
boolean coolingfansmanual = false;
int tempincra = 0;
float contcommandfloat = 0;
float reportedamps = 0;
float mod1DCamps = 0;
float mod2DCamps = 0;
float mod3DCamps = 0;
float mod4DCamps = 0;
float mod5DCamps = 0;
float quickchargeamps = 0;
float mod1DCvolts = 0;
float mod2DCvolts = 0;
float mod3DCvolts = 0;
float mod4DCvolts = 0;
float mod5DCvolts = 0;
float mod1tempA = 0;
float mod1tempB = 0;
float mod1tempC = 0;
float mod2tempA = 0;
float mod2tempB = 0;
float mod2tempC = 0;
float mod3tempA = 0;
float mod3tempB = 0;
float mod3tempC = 0;
float mod4tempA = 0;
float mod4tempB = 0;
float mod4tempC = 0;
float mod5tempA = 0;
float mod5tempB = 0;
float mod5tempC = 0;
float mod1ACamps = 0;
float mod2ACamps = 0;
float mod3ACamps = 0;
float mod4ACamps = 0;
float mod5ACamps = 0;
float mod1ACvolts = 0;
float mod2ACvolts = 0;
float mod3ACvolts = 0;
float mod4ACvolts = 0;
float mod5ACvolts = 0;
boolean sodbmReset = false;
boolean moduleReset = true;
int resetincra = 0;
boolean chademoenable = false;
boolean coolingfansAenable = false;
boolean coolingfansBenable = false;
float coolingfansAenablefloat = 0;
float coolingfansBenablefloat = 0;
boolean coolingfansAauto = false;
boolean coolingfansBauto = false;




int j = 38;
boolean vehper = false;
int d1 = 22;
int d2 = 24;
int connectorpin = 26;
int contactor = 28;
int controllers = 30;
int chargingmodules = 32;
int coolingfansA = 34;
int coolingfansB = 36;


int vehmaxbattvolt = 0;
int SOCconstant = 0;
int maxchargetime10s = 0;
int maxchargetime1min = 0;
int estchargetime1min = 0;
float estchargetime1minfloat = 0;
int chargeseqcontrolnum = 0;
int vehtargetbattvolt = 0;
int vehcurrentreq = 0;
int vehfaultflag = 0;
boolean vfbattovolt = false;
boolean vfbattuvolt = false;
boolean vfbattcd = false;
boolean vfbatthtemp = false;
boolean vfbattvoltdev = false;
int vehstatusflag = 0;
boolean vehcharenable = false;
boolean vehpark = false;
boolean vehchafault = false;
boolean vehcontactor = true;
boolean vehnormstop = false;
int SOCdisplay = 0;
float SOCdisplayfloat = 0;
int chaseq = 0;
int incra = 0;
int chaloopstart = 0;
int incraDelta = 0;
unsigned int timeout = 1000;

boolean chastartstop = false;
float chastartstopfloat = 0;
boolean chacan = false;

int charavailoutputvolts = 0;
int charavailoutputvolts1 = 0;
int charavailoutputvolts2 = 0;
int charavailoutputcurr = 0;
int threshvolts = 0;
int threshvolts1 = 0;
int threshvolts2 = 0;
int charoutputvolts = 0;
int charoutputvolts1 = 0;
int charoutputvolts2 = 0;
int charoutputcurr = 0;
int charflagout = 0;
int charremaintime10s = 0;
int charremaintime1min = 0;
boolean charstatus = false;
boolean charmalfunct = false;
boolean charlock = false;
boolean charbattincomp = false;
boolean charsysmalfunct = false;
boolean charstopcontrol = false;
float sodbmResetfloat = 0;
float moduleResetfloat = 0;







int chargevolts = 0;
float chargevoltsfloat = 0;
float chargeamps = 0;
float battvolts = 333;
int chargevoltsint1 = 0;
int chargevoltsint2 = 0;
int chargecontrolint6 = 0;
int chargecontrolint7 = 0;
int chargeampsint = 0;
float chargeampsintfloat = 0;
int chargeampsint1 = 0;
int chargeampsint2 = 0;
int chargeampsint3 = 0;
int chargeampsint4 = 0;
int chargeampsint5 = 0;
int battvoltsint = 0;
int mod1chargerStatus = 0;
int mod2chargerStatus = 0;
int mod3chargerStatus = 0;
int mod4chargerStatus = 0;
int mod5chargerStatus = 0;
float mod1chargerStatusfloat = 0;
float mod2chargerStatusfloat = 0;
float mod3chargerStatusfloat = 0;
float mod4chargerStatusfloat = 0;
float mod5chargerStatusfloat = 0;



float acc = 0;
float pilot = 0;
float ampacity = 0;
float VAC = 0;
float AAC = 0;
float Hz = 0;
int sodbmdiagcycle = 1;
int sodbmtoggle = 1;
int b207A = 0;
int b307A = 0;
int chargingstate = 0;

CAN_FRAME inFrame;

void setup() {

  Wire.begin();
  Serial.begin(115200);

  EEPROM.setWPPin(19);
  EEPROM.read(page, myVars);
  if (myVars.goodEEPROM != 200)defaults();
  myVars.logger = false;


  initializeCAN();

  pinMode(j, INPUT_PULLUP);
  pinMode(d1, OUTPUT);
  pinMode(d2, OUTPUT);
  pinMode(connectorpin, OUTPUT);
  pinMode(contactor, OUTPUT);
  pinMode(controllers, OUTPUT);
  pinMode(chargingmodules, OUTPUT);
  pinMode(coolingfansA, OUTPUT);
  pinMode(coolingfansB, OUTPUT);

  digitalWrite(d1, HIGH);
  digitalWrite(d2, HIGH);
  digitalWrite(connectorpin, HIGH);
  digitalWrite(contactor, HIGH);
  digitalWrite(controllers, HIGH);
  digitalWrite(chargingmodules, HIGH);
  digitalWrite(coolingfansA, HIGH);
  digitalWrite(coolingfansB, HIGH);




  Scheduler.startLoop(ChademoLoop);
  Scheduler.startLoop(ControllerStartup);
  Scheduler.startLoop(temperatureLoop);

}

void ControllerStartup()
{
  if (moduleReset == true)
  {
    digitalWrite(controllers, HIGH);
    digitalWrite(chargingmodules, HIGH);
    delay(2000);
    digitalWrite(controllers, LOW);
    delay(2000);
    digitalWrite(chargingmodules, LOW);
    moduleReset = false;
  }
  delay(100);
}

void temperatureLoop ()
{
  if (tempincra > 50)
  {
    tempincra = 0;
    if (mod1tempA < 90  && mod2tempA < 90 && mod3tempA < 90 &&  mod4tempA < 90 &&  mod5tempA < 90 && mod5tempB < 90)
    {
      coolingfansAauto = false;
      coolingfansBauto = false;
    }
    else if (mod1tempA > 94 || mod2tempA > 94 || mod3tempA > 94 || mod4tempA > 94 || mod5tempA > 94 || mod1tempB > 94 || mod2tempB > 94 || mod3tempB > 94 || mod4tempB > 94 || mod5tempB > 94)
    {
      coolingfansAauto = true;
      coolingfansBauto = true;

    }
    else if (mod1tempA > 89 || mod2tempA > 89 || mod3tempA > 89 || mod4tempA > 89 || mod5tempA > 89 || mod1tempB > 89 || mod2tempB > 89 || mod3tempB > 89 || mod4tempB > 89 || mod5tempB > 89)
    {
      coolingfansAauto = true;
      coolingfansBauto = false;
    }

  }
  if (coolingfansAauto == true)
  {
    coolingfansAenable = true;
  }

  if (coolingfansBauto == true)
  {
    coolingfansBenable = true;
  }

  if (coolingfansmanual == false)
  {
    if (coolingfansAauto == false)
    {
      coolingfansAenable = false;
    }

    if (coolingfansBauto == false)
    {
      coolingfansBenable = false;
    }
  }

  if (coolingfansAenable == true)
  {
    digitalWrite(coolingfansA, LOW);
  }
  else if (coolingfansAenable == false)
  {
    digitalWrite(coolingfansA, HIGH);
  }
  if (coolingfansBenable == true)
  {
    digitalWrite(coolingfansB, LOW);
  }
  else if (coolingfansBenable == false)
  {
    digitalWrite(coolingfansB, HIGH);
  }
  tempincra++;

  delay(100);
}



void loop()
{
  while (SerialUSB.available()) {
    serialEvent();
  }
  sprintf(logstringtl, "%02X%02X", chargevoltsint1, chargevoltsint2);
  chargevolts = (float)strtol(logstringtl, NULL, 16);
  chargevoltsfloat = chargevolts;

  if (mod1lrx > timeout)
  {
    mod1DCvolts  = 0;
    mod1DCamps = 0;
    mod1ACvolts  = 0;
    mod1ACamps = 0;
    mod1tempA = 0;
    mod1tempB = 0;
    mod1tempC = 0;
    mod1chargerStatus = 999;
  }
  if (mod2lrx > timeout)
  {
    mod2DCvolts  = 0;
    mod2DCamps = 0;
    mod2ACvolts  = 0;
    mod2ACamps = 0;
    mod2tempA = 0;
    mod2tempB = 0;
    mod2tempC = 0;
    mod2chargerStatus = 999;
  }

  if (mod3lrx > timeout)
  {
    mod3DCvolts  = 0;
    mod3DCamps = 0;
    mod3ACvolts  = 0;
    mod3ACamps = 0;
    mod3tempA = 0;
    mod3tempB = 0;
    mod3tempC = 0;
    mod3chargerStatus = 999;
  }

  if (mod4lrx > timeout)
  {
    mod4DCvolts  = 0;
    mod4DCamps = 0;
    mod4ACvolts  = 0;
    mod4ACamps = 0;
    mod4tempA = 0;
    mod4tempB = 0;
    mod4tempC = 0;
    mod4chargerStatus = 999;
  }

  if (mod5lrx > timeout)
  {
    mod5DCvolts  = 0;
    mod5DCamps = 0;
    mod5ACvolts  = 0;
    mod5ACamps = 0;
    mod5tempA = 0;
    mod5tempB = 0;
    mod5tempC = 0;
    mod5chargerStatus = 999;
  }

  chargeampsint1 = chargeampsint * 10 / 5;
  chargeampsint2 = chargeampsint * 10 / 5;
  chargeampsint3 = chargeampsint * 10 / 5;
  chargeampsint4 = chargeampsint * 10 / 5;
  chargeampsint5 = chargeampsint * 10 / 5;

  if (sodbmReset == true)resetincra++;
  if (resetincra > 10)
  {
    sodbmReset = false;
    resetincra = 0;
  }
  reportedvolts = (reportedvoltspre & 0x1FF);
  reportedvolts = reportedvolts * 2;
  battvolts = reportedvolts / 2;
  b207A = (reportedvolts & 0xFF00);
  b207A = b207A >> 8;
  b307A = (reportedvolts & 0x00FF);

  myVars.outFrame.id = 0x101;
  myVars.outFrame.data.bytes[0] = contcommand;
  myVars.outFrame.data.bytes[1] = chargevoltsint1;
  myVars.outFrame.data.bytes[2] = chargevoltsint2;
  myVars.outFrame.data.bytes[3] = chargeampsint1;
  myVars.outFrame.data.bytes[4] = 0x00;
  myVars.outFrame.data.bytes[5] = 0x00;
  myVars.outFrame.data.bytes[6] = 0x00;
  myVars.outFrame.data.bytes[7] = sodbmReset;
  sendCAN(myVars.CANdo);
  delay(20);
  myVars.outFrame.id = 0x102;
  myVars.outFrame.data.bytes[0] = contcommand;
  myVars.outFrame.data.bytes[1] = chargevoltsint1;
  myVars.outFrame.data.bytes[2] = chargevoltsint2;
  myVars.outFrame.data.bytes[3] = chargeampsint2;
  myVars.outFrame.data.bytes[4] = 0x00;
  myVars.outFrame.data.bytes[5] = 0x00;
  myVars.outFrame.data.bytes[6] = 0x00;
  myVars.outFrame.data.bytes[7] = sodbmReset;
  sendCAN(myVars.CANdo);
  delay(20);
  myVars.outFrame.id = 0x103;
  myVars.outFrame.data.bytes[0] = contcommand;
  myVars.outFrame.data.bytes[1] = chargevoltsint1;
  myVars.outFrame.data.bytes[2] = chargevoltsint2;
  myVars.outFrame.data.bytes[3] = chargeampsint3;
  myVars.outFrame.data.bytes[4] = 0x00;
  myVars.outFrame.data.bytes[5] = 0x00;
  myVars.outFrame.data.bytes[6] = 0x00;
  myVars.outFrame.data.bytes[7] = sodbmReset;
  sendCAN(myVars.CANdo);
  delay(20);
  myVars.outFrame.id = 0x104;
  myVars.outFrame.data.bytes[0] = contcommand;
  myVars.outFrame.data.bytes[1] = chargevoltsint1;
  myVars.outFrame.data.bytes[2] = chargevoltsint2;
  myVars.outFrame.data.bytes[3] = chargeampsint4;
  myVars.outFrame.data.bytes[4] = 0x00;
  myVars.outFrame.data.bytes[5] = 0x00;
  myVars.outFrame.data.bytes[6] = 0x00;
  myVars.outFrame.data.bytes[7] = sodbmReset;
  sendCAN(myVars.CANdo);
  delay(20);
  myVars.outFrame.id = 0x10D;
  myVars.outFrame.data.bytes[0] = contcommand;
  myVars.outFrame.data.bytes[1] = chargevoltsint1;
  myVars.outFrame.data.bytes[2] = chargevoltsint2;
  myVars.outFrame.data.bytes[3] = chargeampsint5;
  myVars.outFrame.data.bytes[4] = 0x00;
  myVars.outFrame.data.bytes[5] = 0x00;
  myVars.outFrame.data.bytes[6] = 0x00;
  myVars.outFrame.data.bytes[7] = sodbmReset;
  sendCAN(myVars.CANdo);
  delay(20);
  quickchargeamps = float( mod1DCamps + mod2DCamps + mod3DCamps + mod4DCamps + mod5DCamps);
  contcommandfloat = float(contcommand);
  chargeampsintfloat = float(chargeampsint);
  chastartstopfloat = float(chastartstop);
  SOCdisplayfloat = float(SOCdisplay);
  estchargetime1minfloat = float(estchargetime1min);
  mod1chargerStatusfloat = float(mod1chargerStatus);
  mod2chargerStatusfloat = float(mod2chargerStatus);
  mod3chargerStatusfloat = float(mod3chargerStatus);
  mod4chargerStatusfloat = float(mod4chargerStatus);
  mod5chargerStatusfloat = float(mod5chargerStatus);
  sodbmResetfloat = float(sodbmReset);
  moduleResetfloat = float(moduleReset);
  coolingfansAenablefloat = float(coolingfansAenable);
  coolingfansBenablefloat = float(coolingfansBenable);
  sprintf(logstring2, "%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;\n",
          contcommandfloat, chargevoltsfloat, chargeampsintfloat, quickchargeamps, mod1DCvolts, mod1DCamps, mod1ACvolts, mod1ACamps, mod1tempA, mod1tempB, mod1tempC, mod2DCvolts, mod2DCamps, mod2ACvolts, mod2ACamps, mod2tempA, mod2tempB, mod2tempC, mod3DCvolts, mod3DCamps, mod3ACvolts, mod3ACamps, mod3tempA, mod3tempB, mod3tempC, mod4DCvolts, mod4DCamps, mod4ACvolts, mod4ACamps, mod4tempA, mod4tempB, mod4tempC, chastartstopfloat, SOCdisplayfloat, estchargetime1minfloat, mod1chargerStatusfloat, mod2chargerStatusfloat, mod3chargerStatusfloat, mod4chargerStatusfloat, sodbmResetfloat, mod5DCvolts, mod5DCamps, mod5ACvolts, mod5ACamps, mod5tempA, mod5tempB, mod5tempC, mod5chargerStatusfloat, moduleResetfloat, coolingfansAenablefloat, coolingfansBenablefloat);
  Serial << logstring2;


  //delay(100);
}

void ChademoLoop()
{
  //Read analog vehicle permission signal
  vehper = digitalRead(j);
  vehper = !vehper;
  if (chademoenable == true)
  {
    if (chastartstop == true)
    {
      if (chaloopstart == 0)
      {
        chargeampsint = 0x00;
        chargevoltsint1 = 0x00;
        chargevoltsint2 = 0x00;
        contcommand = 0;
        digitalWrite(contactor, HIGH);
        delay(2000);

        contcommand = 1;
        digitalWrite(contactor, HIGH);
        chargeampsint = 0;
        delay(5000);

        contcommand = 2;
        digitalWrite(contactor, HIGH);
        delay(500);

        contcommand = 3;
        chargevolts = 360;
        chargevoltsint1 = (chargevolts & 0xFF00);
        chargevoltsint2 = (chargevolts & 0x00FF);
        chargevoltsint1 = chargevoltsint1 >> 8;
        digitalWrite(contactor, LOW);
        delay(1000);

        contcommand = 4;
        chargevolts = 360;
        chargevoltsint1 = (chargevolts & 0xFF00);
        chargevoltsint2 = (chargevolts & 0x00FF);
        chargevoltsint1 = chargevoltsint1 >> 8;
        digitalWrite(contactor, LOW);
        delay(2000);
        chaloopstart++;
      }
      if (v102lrx > timeout && chacan == true && chaseq != 0) chastartstop = false;
      chacan = true;
      switch (chaseq)
      {
        case 0:
          digitalWrite(d1, LOW);
          if (incra > 100)
          {
            chastartstop = false;
            incra = 0;
            incraDelta = 0;
          }
          charavailoutputvolts = 370;
          charavailoutputcurr = 100;
          threshvolts = 364;
          charflagout = 0x20;
          if (vehper == true)
          {
            if (incraDelta <= incra)
            {
              if (vehcharenable == true)
              {
                chaseq = 1;
                incra = 0;
                incraDelta = 0;
              }
            }
            else
            {
              chastartstop = false;
              chaseq = 0;
            }
            incraDelta++;
          }



          else
          {
            incra++;
            incraDelta = incra - 13;
          }

          break;

        case 1:
          digitalWrite(connectorpin, LOW);
          digitalWrite(d2, LOW);

          charflagout = 0x05;
          chargeampsint = vehcurrentreq;
          if (vehper == false || chaloopstart == false)
          {
            chargeampsint = 0;

            if (quickchargeamps < 1)
            {
              digitalWrite(d1, HIGH);
              digitalWrite(d2, HIGH);
              digitalWrite(connectorpin, HIGH);
              charflagout = 0x20;
              chaseq = 0;
              chastartstop = false;
              contcommand = 5;
              digitalWrite(contactor, HIGH);
            }
          }


          break;
      }
    }
    else
    {
      chargeampsint = 0;
      if (quickchargeamps < 1)
      {
        digitalWrite(d1, HIGH);
        digitalWrite(d2, HIGH);

        chargeampsint = 0;
        digitalWrite(contactor, HIGH);
        contcommand = 5;
      }
      if (quickchargeamps < 1 && (mod1DCvolts + mod2DCvolts + mod3DCvolts + mod4DCvolts + mod5DCvolts) < 6)
      {
        digitalWrite(connectorpin, HIGH);
        charflagout = 0x20;
        chaseq = 0;
        chastartstop = false;
        chacan = false;
        chargeampsint = 0x00;
        chargevoltsint1 = 0x00;
        chargevoltsint2 = 0x00;
        contcommand = 0;
        digitalWrite(contactor, HIGH);
      }

      incraDelta = 0;
      chaloopstart = 0;

    }



    //Setup splits for multi byte CAN
    //Split charavailoutputvolts
    charavailoutputvolts1 = (charavailoutputvolts & 0xFF00);
    charavailoutputvolts1 = charavailoutputvolts1 >> 8;
    charavailoutputvolts2 = (charavailoutputvolts & 0x00FF);

    //Split threshvolts
    threshvolts1 = (threshvolts & 0xFF00);
    threshvolts1 = threshvolts1 >> 8;
    threshvolts2 = (threshvolts & 0x00FF);

    //Split charoutputvolts
    charoutputvolts = (int)(mod1DCvolts + mod2DCvolts + mod3DCvolts + mod4DCvolts + mod5DCvolts) / 5;
    charoutputvolts1 = (charoutputvolts & 0xFF00);
    charoutputvolts1 = charoutputvolts1 >> 8;
    charoutputvolts2 = (charoutputvolts & 0x00FF);

    //charoutputcurr
    charoutputcurr = (int) quickchargeamps;




    if (chacan == true)
    {
      //Transmit CAN

      myVars.outFrame.id = 0x108;
      myVars.outFrame.data.bytes[0] = 0x00; //Welding support identifier (not supporting)
      myVars.outFrame.data.bytes[1] = charavailoutputvolts1; //Available output voltage 1
      myVars.outFrame.data.bytes[2] = charavailoutputvolts2; //Available output voltage 2
      myVars.outFrame.data.bytes[3] = charavailoutputcurr; //Available output current
      myVars.outFrame.data.bytes[4] = threshvolts1; //Threshold voltage 1
      myVars.outFrame.data.bytes[5] = threshvolts2; //Threshold voltage 2
      myVars.outFrame.data.bytes[6] = 0x00;
      myVars.outFrame.data.bytes[7] = 0x00;
      myVars.outFrame.length = 8;
      myVars.outFrame.fid = 0;
      myVars.outFrame.rtr = 1;
      myVars.outFrame.priority = 0;
      myVars.outFrame.extended = false;
      Can0.sendFrame(myVars.outFrame);
      delay(50);

      myVars.outFrame.id = 0x109;
      myVars.outFrame.data.bytes[0] = 0x01; //Charging sequence control number
      myVars.outFrame.data.bytes[1] = charoutputvolts1; //Present output voltage 1
      myVars.outFrame.data.bytes[2] = charoutputvolts2; //Present output voltage 2
      myVars.outFrame.data.bytes[3] = charoutputcurr; //Present output current
      myVars.outFrame.data.bytes[4] = 0x00;
      myVars.outFrame.data.bytes[5] = charflagout; //Status/fault flag (charger)
      myVars.outFrame.data.bytes[6] = charremaintime10s; //Remaining charging time (by 10 s)
      myVars.outFrame.data.bytes[7] = charremaintime1min; //Remaining charging time (by 1 min)
      myVars.outFrame.length = 8;
      myVars.outFrame.fid = 0;
      myVars.outFrame.rtr = 1;
      myVars.outFrame.priority = 0;
      myVars.outFrame.extended = false;
      Can0.sendFrame(myVars.outFrame);
      delay(50);
    }
    else delay(100);
  }
  else delay(100);
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
    chargeampsint = newValue;
  }


}

void handleShortCmd()
{
  uint8_t val;

  switch (cmdBuffer[0])
  {

    case '1':
      chademoenable = false;
      contcommand = 1;
      digitalWrite(contactor, HIGH);
      break;
    case '2':
      chademoenable = false;
      contcommand = 2;
      digitalWrite(contactor, HIGH);
      break;
    case '3':
      chademoenable = false;
      contcommand = 3;
      chargevolts = 360;
      chargevoltsint1 = (chargevolts & 0xFF00);
      chargevoltsint2 = (chargevolts & 0x00FF);
      chargevoltsint1 = chargevoltsint1 >> 8;
      digitalWrite(contactor, LOW);

      break;
    case '4':
      chademoenable = false;
      contcommand = 4;
      chargevolts = 360;
      chargevoltsint1 = (chargevolts & 0xFF00);
      chargevoltsint2 = (chargevolts & 0x00FF);
      chargevoltsint1 = chargevoltsint1 >> 8;
      chargeampsint = 1;
      digitalWrite(contactor, LOW);
      break;
    case '5':
      chademoenable = false;
      contcommand = 5;
      chargeampsint = 0;
      digitalWrite(contactor, HIGH);
      break;

    case '6':
      chademoenable = true;
      chastartstop = true; //Start
      incra = 0;
      incraDelta = 0;
      chaloopstart = 0;
      break;

    case '7':
      chademoenable = true;
      chastartstop = false; //Stop
      chargeampsint = 0;
      incra = 0;
      chaloopstart = 0;
      break;

    case '9':
      chademoenable = false;
      sodbmReset = true;
      break;

    case '0':
      chademoenable = false;
      chargeampsint = 0x00;
      chargevoltsint1 = 0x00;
      chargevoltsint2 = 0x00;
      contcommand = 0;
      digitalWrite(contactor, HIGH);
      break;
    case 'r':
      chademoenable = false;
      chargeampsint = 0x00;
      chargevoltsint1 = 0x00;
      chargevoltsint2 = 0x00;
      contcommand = 0;
      digitalWrite(contactor, HIGH);
      chademoenable = true;
      chastartstop = false; //Stop
      chargeampsint = 0;
      incra = 0;
      chaloopstart = 0;
      moduleReset = true;
      break;
    case 'a':
      if (coolingfansAenable == true)
      {
        coolingfansmanual = false;
        coolingfansAenable = false;
      }
      else
      {
        coolingfansmanual = true;
        coolingfansAenable = true;
      }
      break;
    case 'b':
      if (coolingfansBenable == true)
      {
        coolingfansmanual = false;
        coolingfansBenable = false;
      }
      else
      {
        coolingfansmanual = true;
        coolingfansBenable = true;
      }

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

    //Can1.setRXFilter(0x105, 0x7F0, false);
    //Can1.setRXFilter(0x10E, 0x7FF, false);
    //Can1.setRXFilter(0x10F, 0x7FF, false);
    Can1.setNumTXBoxes(3);
    Can1.setRXFilter(0x105, 0x7F3, false); //Rx for 0x105 and 0x109 (Module A)
    Can1.setRXFilter(0x106, 0x7F3, false); //Rx for 0x106 and 0x10A (Module B)
    Can1.setRXFilter(0x107, 0x7F3, false); //Rx for 0x107 and 0x10B (Module C)
    Can1.setRXFilter(0x108, 0x7FB, false); //Rx for 0x108 and 0x10C (Module D)
    Can1.setRXFilter(0x10E, 0x7FE, false); //Rx for 0x10E and 0x10F (Module E)

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
    //Can0.setRXFilter(0X2EC, 0x7FF, false);
    //Can0.setRXFilter(0x7EA, 0x7FF, false);
    Can0.setNumTXBoxes(2);

    Can0.setRXFilter(0x100, 0x7FF, false);
    Can0.setRXFilter(0x101, 0x7FF, false);
    Can0.setRXFilter(0x102, 0x7FF, false);




    Can0.setGeneralCallback(handleFrame);
    Serial << "\n\nUsing CAN0 - initialization completed at " << myVars.datarate << " \n";
  }
  else Serial.println("\nCAN0 initialization (sync) ERROR\n");

}

void handleFrame(CAN_FRAME * frame)

{


  switch (frame->id)
  {
    case 0x100:

      v100lrx = 0;

      //b4,b5 Max Battery Voltage
      sprintf(logstringveh, "%02X%02X", frame->data.bytes[4], frame->data.bytes[5]);
      vehmaxbattvolt = (int)strtol(logstringveh, NULL, 16);

      //b6 Constant of charging rate indication (for display)
      sprintf(logstringveh, "%02X", frame->data.bytes[6]);
      SOCconstant = (int)strtol(logstringveh, NULL, 16);

      break;


    case 0x101:

      v101lrx = 0;

      //b1 Max charging time (by 10s)
      sprintf(logstringveh, "%02X", frame->data.bytes[1]);
      maxchargetime10s = (int)strtol(logstringveh, NULL, 16);

      //b2 Max charging time (by 1 min)
      sprintf(logstringveh, "%02X", frame->data.bytes[2]);
      maxchargetime1min = (int)strtol(logstringveh, NULL, 16);

      //b3 Estimated charging time (by 1 min)
      sprintf(logstringveh, "%02X", frame->data.bytes[3]);
      estchargetime1min = (int)strtol(logstringveh, NULL, 16);

      break;


    case 0x102:

      v102lrx = 0;
      //b0 Charging sequence control number
      sprintf(logstringveh, "%02X", frame->data.bytes[0]);
      chargeseqcontrolnum = (int)strtol(logstringveh, NULL, 16);

      //b1,b2 Target battery voltage
      sprintf(logstringveh, "%02X%02X", frame->data.bytes[1], frame->data.bytes[2]);
      vehtargetbattvolt = (int)strtol(logstringveh, NULL, 16);

      //b3 Charging current request
      sprintf(logstringveh, "%02X", frame->data.bytes[3]);
      vehcurrentreq = (int)strtol(logstringveh, NULL, 16);

      //b4 Fault flag
      sprintf(logstringveh, "%02X", frame->data.bytes[4]);
      vehfaultflag = (int)strtol(logstringveh, NULL, 16);

      //Battery Overvolt 0:Normal 1:Fault
      if ((vehfaultflag & 0x01) == 0x01) vfbattovolt = true;
      else vfbattovolt = false;

      //Battery Undervolt 0:Normal 1:Fault
      if ((vehfaultflag & 0x02) == 0x02) vfbattuvolt = true;
      else vfbattuvolt = false;

      //Battery current deviation error 0:Normal 1:Fault
      if ((vehfaultflag & 0x04) == 0x04) vfbattcd = true;
      else vfbattcd = false;

      //Battery high temp error 0:Normal 1:Fault
      if ((vehfaultflag & 0x08) == 0x08) vfbatthtemp = true;
      else vfbatthtemp = false;

      //Battery voltage deviation error 0:Normal 1:Fault
      if ((vehfaultflag & 0x10) == 0x10) vfbattvoltdev = true;
      else vfbattvoltdev = false;

      //b5 Status flag
      sprintf(logstringveh, "%02X", frame->data.bytes[5]);
      vehstatusflag = (int)strtol(logstringveh, NULL, 16);

      //Vehicle charging enabled 0:disabled 1:enabled
      if ((vehstatusflag & 0x01) == 0x01) vehcharenable = true;
      else vehcharenable = false;

      //Vehicle shift position 0:Parking position 1:Other position
      if ((vehstatusflag & 0x02) == 0x02) vehpark = false;
      else vehpark = true;

      //Charging system fault 0:Normal 1:Fault
      if ((vehstatusflag & 0x04) == 0x04) vehchafault = true;
      else vehchafault = false;

      //Vehicle status 0:EV contactor closed or welding being detected 1:EV contactor open or welding detection finished
      if ((vehstatusflag & 0x08) == 0x08) vehcontactor = false;
      else vehcontactor = true;

      // Normal stop request before charging 0:No Request 1:Stop Request
      if ((vehstatusflag & 0x10) == 0x10) vehnormstop = true;
      else vehnormstop = false;

      //b6 Charging rate (for display)
      sprintf(logstringveh, "%02X", frame->data.bytes[6]);
      SOCdisplay = (int)strtol(logstringveh, NULL, 16);

      break;



  }







}

void handleFrame1(CAN_FRAME * frame1)

{
  //Serial<<"got it\n";
  if (frame1->id == 0x105)
  {
    mod1lrx = 0;
    sprintf(logstringt, "%02X%02X", frame1->data.bytes[0], frame1->data.bytes[1]);
    mod1DCvolts  = (float) strtol(logstringt, NULL, 16);
    /*
      Serial<<"\nmod1DCvolts = ";
      Serial<<mod1DCvolts;
      Serial<<" V     logstringt = ";
      Serial<<logstringt;
      Serial<<"\n";
    */
    sprintf(logstringt, "%02X", frame1->data.bytes[2]);
    mod1DCamps = (float) strtol(logstringt, NULL, 16) / 10;

    sprintf(logstringt, "%02X", frame1->data.bytes[7]);
    mod1chargerStatus = (int) strtol(logstringt, NULL, 16);

  }

  else if (frame1->id == 0x106)
  {
    mod2lrx = 0;
    sprintf(logstringt, "%02X%02X", frame1->data.bytes[0], frame1->data.bytes[1]);
    mod2DCvolts  = (float) strtol(logstringt, NULL, 16);

    sprintf(logstringt, "%02X", frame1->data.bytes[2]);
    mod2DCamps = (float) strtol(logstringt, NULL, 16) / 10;

    sprintf(logstringt, "%02X", frame1->data.bytes[7]);
    mod2chargerStatus = (int) strtol(logstringt, NULL, 16);

  }

  else if (frame1->id == 0x107)
  {
    mod3lrx = 0;
    sprintf(logstringt, "%02X%02X", frame1->data.bytes[0], frame1->data.bytes[1]);
    mod3DCvolts  = (float) strtol(logstringt, NULL, 16);

    sprintf(logstringt, "%02X", frame1->data.bytes[2]);
    mod3DCamps = (float) strtol(logstringt, NULL, 16) / 10;

    sprintf(logstringt, "%02X", frame1->data.bytes[7]);
    mod3chargerStatus = (int) strtol(logstringt, NULL, 16);
  }

  else if (frame1->id == 0x108)
  {

    mod4lrx = 0;
    sprintf(logstringt, "%02X%02X", frame1->data.bytes[0], frame1->data.bytes[1]);
    mod4DCvolts  = (float) strtol(logstringt, NULL, 16);

    sprintf(logstringt, "%02X", frame1->data.bytes[2]);
    mod4DCamps = (float) strtol(logstringt, NULL, 16) / 10;

    sprintf(logstringt, "%02X", frame1->data.bytes[7]);
    mod4chargerStatus = (int) strtol(logstringt, NULL, 16);
  }

  else if (frame1->id == 0x109)
  {
    mod1lrx = 0;
    sprintf(logstringt, "%02X%02X", frame1->data.bytes[0], frame1->data.bytes[1]);
    mod1ACvolts  = (float) strtol(logstringt, NULL, 16);

    sprintf(logstringt, "%02X", frame1->data.bytes[2]);
    mod1ACamps = (float) strtol(logstringt, NULL, 16);

    sprintf(logstringt, "%02X", frame1->data.bytes[5]);
    mod1tempA = (float)(strtol(logstringt, NULL, 16) - 40) * 9 / 5 + 32;

    sprintf(logstringt, "%02X", frame1->data.bytes[6]);
    mod1tempB = (float)(strtol(logstringt, NULL, 16) - 40) * 9 / 5 + 32;

    sprintf(logstringt, "%02X", frame1->data.bytes[7]);
    mod1tempC = (float)(strtol(logstringt, NULL, 16) - 40) * 9 / 5 + 32;
  }

  else if (frame1->id == 0x10A)
  {
    mod2lrx = 0;
    sprintf(logstringt, "%02X%02X", frame1->data.bytes[0], frame1->data.bytes[1]);
    mod2ACvolts  = (float) strtol(logstringt, NULL, 16);

    sprintf(logstringt, "%02X", frame1->data.bytes[2]);
    mod2ACamps = (float) strtol(logstringt, NULL, 16);

    sprintf(logstringt, "%02X", frame1->data.bytes[5]);
    mod2tempA = (float)(strtol(logstringt, NULL, 16) - 40) * 9 / 5 + 32;

    sprintf(logstringt, "%02X", frame1->data.bytes[6]);
    mod2tempB = (float)(strtol(logstringt, NULL, 16) - 40) * 9 / 5 + 32;

    sprintf(logstringt, "%02X", frame1->data.bytes[7]);
    mod2tempC = (float)(strtol(logstringt, NULL, 16) - 40) * 9 / 5 + 32;

  }

  else if (frame1->id == 0x10B)
  {
    mod3lrx = 0;
    sprintf(logstringt, "%02X%02X", frame1->data.bytes[0], frame1->data.bytes[1]);
    mod3ACvolts  = (float) strtol(logstringt, NULL, 16);

    sprintf(logstringt, "%02X", frame1->data.bytes[2]);
    mod3ACamps = (float) strtol(logstringt, NULL, 16);

    sprintf(logstringt, "%02X", frame1->data.bytes[5]);
    mod3tempA = (float)(strtol(logstringt, NULL, 16) - 40) * 9 / 5 + 32;

    sprintf(logstringt, "%02X", frame1->data.bytes[6]);
    mod3tempB = (float)(strtol(logstringt, NULL, 16) - 40) * 9 / 5 + 32;

    sprintf(logstringt, "%02X", frame1->data.bytes[7]);
    mod3tempC = (float)(strtol(logstringt, NULL, 16) - 40) * 9 / 5 + 32;
  }
  else if (frame1->id == 0x10C)
  {

    mod4lrx = 0;
    sprintf(logstringt, "%02X%02X", frame1->data.bytes[0], frame1->data.bytes[1]);
    mod4ACvolts  = (float) strtol(logstringt, NULL, 16);

    sprintf(logstringt, "%02X", frame1->data.bytes[2]);
    mod4ACamps = (float) strtol(logstringt, NULL, 16);

    sprintf(logstringt, "%02X", frame1->data.bytes[5]);
    mod4tempA = (float)(strtol(logstringt, NULL, 16) - 40) * 9 / 5 + 32;

    sprintf(logstringt, "%02X", frame1->data.bytes[6]);
    mod4tempB = (float)(strtol(logstringt, NULL, 16) - 40) * 9 / 5 + 32;

    sprintf(logstringt, "%02X", frame1->data.bytes[7]);
    mod4tempC = (float)(strtol(logstringt, NULL, 16) - 40) * 9 / 5 + 32;
  }
  else if (frame1->id == 0x10E)
  {
//Serial<<"\n\n RX 10E\n\n";
    mod5lrx = 0;
    sprintf(logstringt, "%02X%02X", frame1->data.bytes[0], frame1->data.bytes[1]);
    mod5DCvolts  = (float) strtol(logstringt, NULL, 16);

    sprintf(logstringt, "%02X", frame1->data.bytes[2]);
    mod5DCamps = (float) strtol(logstringt, NULL, 16) / 10;

    sprintf(logstringt, "%02X", frame1->data.bytes[7]);
    mod5chargerStatus = (int) strtol(logstringt, NULL, 16);
  }
  else if (frame1->id == 0x10F)
  {
    //Serial<<"\n\nRX 10F\n\n";

    mod5lrx = 0;
    sprintf(logstringt, "%02X%02X", frame1->data.bytes[0], frame1->data.bytes[1]);
    mod5ACvolts  = (float) strtol(logstringt, NULL, 16);

    sprintf(logstringt, "%02X", frame1->data.bytes[2]);
    mod5ACamps = (float) strtol(logstringt, NULL, 16);

    sprintf(logstringt, "%02X", frame1->data.bytes[5]);
    mod5tempA = (float)(strtol(logstringt, NULL, 16) - 40) * 9 / 5 + 32;

    sprintf(logstringt, "%02X", frame1->data.bytes[6]);
    mod5tempB = (float)(strtol(logstringt, NULL, 16) - 40) * 9 / 5 + 32;

    sprintf(logstringt, "%02X", frame1->data.bytes[7]);
    mod5tempC = (float)(strtol(logstringt, NULL, 16) - 40) * 9 / 5 + 32;
  }


}



void sendCAN(int which)

{
  /*
    char buffer[140];

    sprintf(buffer, "Sent msgID 0x%03X; %02X; %02X; %02X; %02X; %02X; %02X; %02X; %02X;\n", myVars.outFrame.id,
            myVars.outFrame.data.bytes[0], myVars.outFrame.data.bytes[1], myVars.outFrame.data.bytes[2],
            myVars.outFrame.data.bytes[3], myVars.outFrame.data.bytes[4], myVars.outFrame.data.bytes[5],
            myVars.outFrame.data.bytes[6], myVars.outFrame.data.bytes[7]);
    Serial << buffer;
    Serial << "\n";

    //int time=millis();
    //Serial<<time;
    //Serial<<"\n";
  */

  myVars.outFrame.length = 8;
  myVars.outFrame.fid = 0;
  myVars.outFrame.rtr = 0;
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
  myVars.outFrame.rtr = 0;  // Data payload 8 bytes
  myVars.goodEEPROM = 200;
  myVars.datarate = 500000;
  myVars.outFrame.extended = false;



}
