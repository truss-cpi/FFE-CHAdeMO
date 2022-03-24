/*
	Original code release by Sefs of the MyFocusElectric forum.
	Likely to contain example code that may or may not be under similar license.
	Use at your own risk
	Cleanup and comment expansion by truss-cpi
*/
#include <SPI.h>
/*
	Serial Peripheral Interface library
	SPISettings
    begin()
    end()
    beginTransaction()
    endTransaction()
    setBitOrder()
    setClockDivider()
    setDataMode()
    transfer()
    usingInterrupt()
    Due Extended SPI usage
*/
#include <Arduino.h> // Arduino IDE built-in to perform all the underlying housekeeping stuff
#include <due_can.h> // "Implements interrupt driven communication and supports both object oriented and non-OOP callbacks"
#include <due_wire.h> // "This library allows you to communicate with I2C / TWI devices" - Different versions for different target HW, 2-Wire Interface
/*
    begin()
    requestFrom()
    beginTransmission()
    endTransmission()
    write()
    available()
    read()
    SetClock()
    onReceive()
    onRequest() 
*/
#include "variant.h" // Names the various pins on different board variants, allowing addressing by designations other than pin number (RX1, DAC0, etc)
#include <Wire_EEPROM.h> // I2C based routines to support EEPROM on the Due. Requires the due_wire library. Defines EEPROM class for EEPROM actions via i2c communication
/*
	readByte()	uint8_t readByte(uint32_t address);
	writeByte()	void writeByte(uint32_t address, uint8_t valu);
	serWPPin()	void setWPPin(uint8_t pin);
	EEPROM.write()	template <class T> int write(int ee, const T& value)
	EEPROM.read()	template <class T> int read(int ee, T& value)
*/
#include <Scheduler.h> // The Scheduler library enables the Arduino Due, Zero, and MKR1000 to run multiple functions at the same time (ARM processors ftw). This allows tasks to happen without interrupting each other.

#define Serial SerialUSB // #define constantName value
/*
	if(Serial)
	available()
	availableForWrite()
	begin()
	end()
	find()
	findUntil()
	flush()
	parseFloat()
	parseInt()
	peek()
	print()
	println()
	read()
	readBytes()
	readBytesUntil()
	readString()
	readStringUntil()
	setTimeout()
	write()
	serialEvent()
*/
template<class T> inline Print &operator <<(Print &obj, T arg) { // no-cost stream operator as described at http://arduiniana.org/libraries/streaming/
  obj.print(arg);  // Sets up Serial streaming Serial<<someshit;
  return obj;
}

class Field {
  public: // Define variables in public classes to help with organization
    uint16_t msb;
    uint16_t bits;
    uint32_t data;
    char* fieldName;
};

class Msg {
  public: // Define variables in public classes to help with organization
    char* moduleName;
    uint16_t msgID;
    uint64_t msg;
    Field fields[16];
    uint8_t field_count;
};

class StringTable {
  public: // Define variables in public classes to help with organization
    Msg messages[32];
};

class EEPROMvariables {
  public: // Define variables in public classes to help with organization
    uint8_t CANdo; // Use Serial port if 2, CAN0 if 0 or CAN1 if 1
    char logfile[80];
    uint16_t transmitime = 0;
    boolean logger;
    uint32_t datarate = 500000;
    uint8_t goodEEPROM;
    CAN_FRAME outFrame;
};

StringTable stringTable; // Declare an array of strings
EEPROMvariables myVars; // Declare an instance/array of variables within EEPROM with the name myVars
uint16_t page = 475;
uint16_t dummy;
char cmdBuffer[100];
char logstring[200];
char logstring2[300] = "";
char logstringt[200];
char logstringt1[200]; // 200 index char array
char logstringc1[200];
char logstringts[200];
char msgBuff[100];
float Version = 1.10;
int ptrBuffer;
short logcycle = 0;
boolean handlingEvent, debug;
int seq = 0;
unsigned long lastrx = 0;
unsigned long lastactiverx = 0;
unsigned long howlongagorx = 0;
unsigned long howlonglastactiverx = 0;
int chacurrent = 0;
int chargestatus = 0;
float chacurrentf = 0;
float estchatimef = 0;
float seqf = 0;
float mincell = 0;
float maxcell = 0;
float deltacell = 0;
float batchadspl = 0;
float brakemode = 0;
float frictionbrake = 0;
float torquedelivered = 0;
int torquedeliveredint = 0;
float motorRPM = 0;
float MPH = 0;
float hbattchar = 0;
float brake = 0;
float acc = 0;
float pilot = 0;
float ampacity = 0;
float VAC = 0;
float AAC = 0;
float Hz = 0;
int sodbmdiagcycle = 0;
int sodbmtoggle = 0;
int becmdiagcycle = 0;
int becmtoggle = 1;
int socint = 0;
int maxchacurrent = 0;
int maxchatime = 0;
int zerocurrentwait = 0;
int estchatime = 0;
int maxchavoltage = 0;
int chastatusfault = 0;
int chastop = 0;
int vehstop = 0;
int brakeon = 0;
int chaenable = 0;
int compat = 0;
int incra = 0;
int wayup = 0;
int seq1start = 0;
int seq2start = 0;
int f = 29;
int g = 31;
int k = 25;
int e = 27;
int d1 = 0;
int d2 = 0;
float hbattcoolin = 0;
float mcon = 0;
float hbattcurr = 0;
float currentandmcon = 0;
float ETE = 0;
float hbatttemp = 0;
float hbattvolt = 0;
float throttle = 0;
float pwrt = 0;
float hvac = 0;
CAN_FRAME inFrame; // Define a CAN_FRAME structure named inFrame

void setup() {
  Wire.begin(); /* Initiate the Wire library and join the I2C bus as a master.
  The 7-bit slave address is optional. If not specified, join the bus as a master.  */
  Serial.begin(115200); // Sets the data rate in bits per second (baud) for serial data transmission.

  EEPROM.setWPPin(19); // 12c on Serial RX 1 pin
  EEPROM.read(page, myVars); // Read out myVars from EEPROM starting at location 475
  if (myVars.goodEEPROM != 200)defaults(); // Reset to defaults if the goodEEPROM value has changed from it's default of 200
  myVars.logger = false; // turn off logging

  initializeCAN(); // Run the initializeCAN function

  pinMode(k, OUTPUT);	// Pin 25 (Variable k) - triggers 5V relay board input
  pinMode(e, OUTPUT);	// Pin 27 (Variable e) - triggers 5V relay board input
  pinMode(f, INPUT);	// Pin 29 (Variable f) - reads output from optoisolators that are driven from the CH +B pin of the CHAdeMO port.
  pinMode(g, INPUT);	// Pin 31 (Variable g) - reads output from optoisolators that are driven from the CH +B pin of the CHAdeMO port.

  Scheduler.startLoop(ActiveDiagLoop); // Start the ActiveDiagLoop as a "background" process
}

void loop() {
  while (SerialUSB.available()) {
    serialEvent();
  }
  d1 = digitalRead(f); // read the state of pin 29 and store it in d1
  d2 = digitalRead(g); // read the state of pin 31 and store it in d2
/*
CHAdeMO CH +B feeds into one side of the 12V relay coil, a voltage divider circuit that splits into both optoisolator inputs which can feed through to these digital pins.
The input ground connections are switched by relays controlled with the digital output pins of this device. However the relay used operates with an Active low behavior
So when our digital outputs are high, the floating inputs to the optoisolators will register as 0s on the inputs.
*/
  /*
  Serial<<"d1 = ";
  Serial<<d1;
  Serial<<"\n";
  Serial<<"d2 = ";
  Serial<<d2;
  Serial<<"\n";
  */

  if (d1 == 0 && d2 == 0) { // if both inputs from the optoisolators are 0s, clear all charging variables
    seq = 0;
    seq1start = 0;
    seq2start = 0;
    chacurrent = 0;
    digitalWrite(k, HIGH); // and set the digital outputs to high, opening the 5V relays
    digitalWrite(e, HIGH);
  }
  else if (d1 == 1 && d2 == 0 && seq1start == 0) {
    seq = 1;
    seq1start = 1;
  }
  else if (d1 == 1 && d2 == 1 && seq2start == 0) {
    seq = 3;
    seq2start = 1;
    digitalWrite(e, LOW);
  }
  else if (d1 == 0 && d2 == 1) { // could be combined with the first "if" based on d1 alone
    seq = 0;
    seq1start = 0;
    seq2start = 0;
    digitalWrite(k, HIGH);
    digitalWrite(e, HIGH);
    chacurrent = 0;
  }

  if (mcon == 0) { // if main contactor state is 0, seq must be 0
    seq = 0;
  }

  if (seq == 2 || seq == 3 || seq == 4) {
    estchatime = float (((-0.4 * maxchacurrent) + 77)) * (100 - socint) / 100 + 1; // if seq 2 to 4, estimate charging time
  }
  if (seq == 0) {
    incra = 0; // clear reusable incremental counter
    //Can1.disable();
    digitalWrite(k, HIGH); // close 5V relays
    digitalWrite(e, HIGH);
    chacurrent = 0;
    maxchacurrent = 0;
    vehstop = 0;
    maxchatime = 60;
    estchatime = 0;
  }

  if (seq == 1) {
    //Can1.enable();
    //Compatibility Checks

    myVars.outFrame.id = 0x100;
    myVars.outFrame.data.bytes[0] = 0x00; // ZERO Byte
    myVars.outFrame.data.bytes[1] = 0x00; // ZERO Byte
    myVars.outFrame.data.bytes[2] = 0x00; // ZERO Byte
    myVars.outFrame.data.bytes[3] = 0x00; // ZERO Byte
    myVars.outFrame.data.bytes[4] = 0x6C; // max battery voltage
    myVars.outFrame.data.bytes[5] = 0x01; // max battery voltage
    myVars.outFrame.data.bytes[6] = 0x64; // constant for SOC indication
    myVars.outFrame.data.bytes[7] = 0x00; // ZERO Byte
    sendCAN(myVars.CANdo); // myVars.CANdo is defaulted to 2, which should send thie message to serial *but* sendCAN() does nothing with the value passed and just sends to CAN1 (CHAdeMO)
    delay(1);

    myVars.outFrame.id = 0x101;
    myVars.outFrame.data.bytes[0] = 0x00; // ZERO Byte
    myVars.outFrame.data.bytes[1] = 0xFF; // max charging time by 10s
    myVars.outFrame.data.bytes[2] = maxchatime; // max charging time by 1 min
    myVars.outFrame.data.bytes[3] = estchatime; // estimated charging time
    myVars.outFrame.data.bytes[4] = 0x00; // ZERO Byte
    myVars.outFrame.data.bytes[5] = 0x00; // ZERO Byte
    myVars.outFrame.data.bytes[6] = 0x00; // ZERO Byte
    myVars.outFrame.data.bytes[7] = 0x00; // ZERO Byte
    sendCAN(myVars.CANdo);
    delay(1);

    myVars.outFrame.id = 0x102;
    myVars.outFrame.data.bytes[0] = 0x01; // version number
    myVars.outFrame.data.bytes[1] = 0x68; // target battery voltage
    myVars.outFrame.data.bytes[2] = 0x01; // target battery voltage
    myVars.outFrame.data.bytes[3] = 0x00; // requested current
    myVars.outFrame.data.bytes[4] = 0x00; // fault flag
    myVars.outFrame.data.bytes[5] = 0x08; // status flag (charging disabled, contact open)
    myVars.outFrame.data.bytes[6] = socint; // displayed SOC
    myVars.outFrame.data.bytes[7] = 0x00; // ZERO Byte
    sendCAN(myVars.CANdo);
    delay(1);

    //Compatability Checker

    if (compat == 1) { // compat defaults to 0, set to 1 with CHAdeMO CAN Frame 0x108
      incra++;
    }
    else if (compat == 2) { // I can only guess this was a way to stop the proecss to check the handshake, no code sets this value to anything but 0 or 1
      seq = 0;
    }

    if (incra > 5 && incra < 10) {
      digitalWrite(k, LOW);

    }
    else if (incra > 15) {
      seq = 2;
      incra = 0;
    }
  }
  else if (seq == 2) {
    myVars.outFrame.id = 0x100;
    myVars.outFrame.data.bytes[0] = 0x00;
    myVars.outFrame.data.bytes[1] = 0x00;
    myVars.outFrame.data.bytes[2] = 0x00;
    myVars.outFrame.data.bytes[3] = 0x00;
    myVars.outFrame.data.bytes[4] = 0x6C; // max battery voltage
    myVars.outFrame.data.bytes[5] = 0x01; // max battery voltage
    myVars.outFrame.data.bytes[6] = 0x64; // constant for SOC indication
    myVars.outFrame.data.bytes[7] = 0x00;
    sendCAN(myVars.CANdo);
    delay(1);

    myVars.outFrame.id = 0x101;
    myVars.outFrame.data.bytes[0] = 0x00;
    myVars.outFrame.data.bytes[1] = 0xFF; //max charging time by 10s
    myVars.outFrame.data.bytes[2] = maxchatime; //max charging time by 1 min
    myVars.outFrame.data.bytes[3] = estchatime; //estimated charging time
    myVars.outFrame.data.bytes[4] = 0x00;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0x00;
    myVars.outFrame.data.bytes[7] = 0x00;
    sendCAN(myVars.CANdo);
    delay(1);

    //Vehicle permission given, start isolation checks, EV contactor open still
    myVars.outFrame.id = 0x102;
    myVars.outFrame.data.bytes[0] = 0x01; // version number
    myVars.outFrame.data.bytes[1] = 0x68; // target battery voltage
    myVars.outFrame.data.bytes[2] = 0x01; // target battery voltage
    myVars.outFrame.data.bytes[3] = 0x00; // requested current
    myVars.outFrame.data.bytes[4] = 0x00; // fault flag
    myVars.outFrame.data.bytes[5] = 0x09; // status flag (charging enabled, contact open)
    myVars.outFrame.data.bytes[6] = socint; // displayed SOC
    myVars.outFrame.data.bytes[7] = 0x00;
    sendCAN(myVars.CANdo);
    delay(1);
    incra = 0;
  }

  else if (seq == 3) {
    myVars.outFrame.id = 0x100;
    myVars.outFrame.data.bytes[0] = 0x00;
    myVars.outFrame.data.bytes[1] = 0x00;
    myVars.outFrame.data.bytes[2] = 0x00;
    myVars.outFrame.data.bytes[3] = 0x00;
    myVars.outFrame.data.bytes[4] = 0x6C; // max battery voltage
    myVars.outFrame.data.bytes[5] = 0x01; // max battery voltage
    myVars.outFrame.data.bytes[6] = 0x64; // constat for SOC indication
    myVars.outFrame.data.bytes[7] = 0x00;
    sendCAN(myVars.CANdo);
    delay(1);

    myVars.outFrame.id = 0x101;
    myVars.outFrame.data.bytes[0] = 0x00;
    myVars.outFrame.data.bytes[1] = 0xFF; // max charging time by 10s
    myVars.outFrame.data.bytes[2] = maxchatime; // max charging time by 1 min
    myVars.outFrame.data.bytes[3] = estchatime; // estimated charging time
    myVars.outFrame.data.bytes[4] = 0x00;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0x00;
    myVars.outFrame.data.bytes[7] = 0x00;
    sendCAN(myVars.CANdo);
    delay(1);

    //Vehicle permission given, EV contactor has closed, wait to request current until 0x109.5.1 and indicate closed relay
    myVars.outFrame.id = 0x102;
    myVars.outFrame.data.bytes[0] = 0x01; // version number
    myVars.outFrame.data.bytes[1] = 0x68; // target battery voltage
    myVars.outFrame.data.bytes[2] = 0x01; // target battery voltage
    myVars.outFrame.data.bytes[3] = 0x00; // requested current
    myVars.outFrame.data.bytes[4] = 0x00; // fault flag
    myVars.outFrame.data.bytes[5] = 0x01; // status flag (charging enabled, contact closed)
    myVars.outFrame.data.bytes[6] = socint; // displayed SOC
    myVars.outFrame.data.bytes[7] = 0x00;
    sendCAN(myVars.CANdo);
    delay(1);
    incra++;
    if (incra > 10) {
      seq = 4;
      incra = 0;
      chacurrent = 1;
      wayup = 0;
      zerocurrentwait = 0;
    }
  }

  else if (seq == 4) {
    if (batchadspl > 99.5) {
      vehstop = 1;
    }
    myVars.outFrame.id = 0x100;
    myVars.outFrame.data.bytes[0] = 0x00;
    myVars.outFrame.data.bytes[1] = 0x00;
    myVars.outFrame.data.bytes[2] = 0x00;
    myVars.outFrame.data.bytes[3] = 0x00;
    myVars.outFrame.data.bytes[4] = 0x6C; // max battery voltage
    myVars.outFrame.data.bytes[5] = 0x01; // max battery voltage
    myVars.outFrame.data.bytes[6] = 0x64; // constat for SOC indication
    myVars.outFrame.data.bytes[7] = 0x00;
    sendCAN(myVars.CANdo);
    delay(1);

    myVars.outFrame.id = 0x101;
    myVars.outFrame.data.bytes[0] = 0x00;
    myVars.outFrame.data.bytes[1] = 0xFF; // max charging time by 10s
    myVars.outFrame.data.bytes[2] = maxchatime; // max charging time by 1 min
    myVars.outFrame.data.bytes[3] = estchatime; // estimated charging time
    myVars.outFrame.data.bytes[4] = 0x00;
    myVars.outFrame.data.bytes[5] = 0x00;
    myVars.outFrame.data.bytes[6] = 0x00;
    myVars.outFrame.data.bytes[7] = 0x00;
    sendCAN(myVars.CANdo);
    delay(1);

    //Vehicle permission given, EV contactor has closed, start requesting and indicate closed relay
    myVars.outFrame.id = 0x102;
    myVars.outFrame.data.bytes[0] = 0x01; // version number
    myVars.outFrame.data.bytes[1] = 0x68; // target battery voltage
    myVars.outFrame.data.bytes[2] = 0x01; // target battery voltage
    myVars.outFrame.data.bytes[3] = chacurrent; // requested current
    myVars.outFrame.data.bytes[4] = 0x00; // fault flag
    if ((chastop == 1 && incra > 20) || vehstop == 1) {
      myVars.outFrame.data.bytes[5] = 0x00; // status flag (charging enabled, contact closed)
      zerocurrentwait++;
    }
    else {

      myVars.outFrame.data.bytes[5] = 0x01; // status flag (charging enabled, contact closed)
      zerocurrentwait = 0;

    }
    myVars.outFrame.data.bytes[6] = socint; // displayed SOC
    myVars.outFrame.data.bytes[7] = 0x00;
    sendCAN(myVars.CANdo);
    delay(1);
    incra++;
    if (wayup == 0 && chaenable == 1) {
      if (chacurrent < maxchacurrent) {
        chacurrent = chacurrent + 2;
        if (chacurrent > maxchacurrent) {
          chacurrent = maxchacurrent;
        }
      }
      else {
        wayup = 1;
      }
    }

    if ((chastop == 1 && incra > 20) || vehstop == 1 ) {
      chacurrent = chacurrent - 20;
      if (chacurrent < 1) {
        chacurrent = 0;
      }
      wayup = 1;
      if (chacurrent == 0 && zerocurrentwait > 18) {
        seq = 5;
        chacurrent = 0;
        digitalWrite(k, HIGH);
        //digitalwrite(e,HIGH);
      }
    }
    if (hbattvolt >= 359 || maxcell > 4.15) {
      wayup = 1;
      if (chacurrent > 0) {
        chacurrent = chacurrent - 1;
      }
      /*else {
        chacurrent = 0;
        seq = 5;
        //digitalwrite(e,HIGH);
      }*/
    }
    incra = 0;

  }

  else if (seq == 5) {
    chacurrent = 0;
    //Permission Revoked, EV contactor has opened
    myVars.outFrame.id = 0x102;
    myVars.outFrame.data.bytes[0] = 0x01;//version number
    myVars.outFrame.data.bytes[1] = 0x68;//target battery voltage
    myVars.outFrame.data.bytes[2] = 0x01;//target battery voltage
    myVars.outFrame.data.bytes[3] = 0x00;//requested current
    myVars.outFrame.data.bytes[4] = 0x00;//fault flag
    if (chaenable == 0) {
      digitalWrite(e, HIGH);
      myVars.outFrame.data.bytes[5] = 0x08;//status flag (charging enabled, contact closed)
      incra++;
    }
    else if (chaenable == 1) {
      myVars.outFrame.data.bytes[5] = 0x00;//status flag (charging enabled, contact closed)
    }
    myVars.outFrame.data.bytes[5] = 0x00;//status flag (charging enabled, contact closed)
    myVars.outFrame.data.bytes[6] = socint;//displayed SOC
    myVars.outFrame.data.bytes[7] = 0x00;
    sendCAN(myVars.CANdo);

    if (incra > 25) {
      seq = 0;
      //Can1.disable();
    }
  }
  //Brake mode
  if (frictionbrake > 0 && torquedelivered < -1) {
    //Blended
    brakemode = 2;
  }
  else if (frictionbrake > 0 && torquedelivered > -1) {
    //Friction
    brakemode = 3;
  }
  else if (frictionbrake == 0 && torquedelivered < -1) {
    //Regen
    brakemode = 1;
  }
  else if (frictionbrake == 0 && torquedelivered > -1) {
    //No Brake
    brakemode = 0;
  }

  chacurrentf = (float) chacurrent;
  seqf = (float) seq;

  estchatimef = (float) estchatime;
  sprintf(logstring2, "%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;\n",
          batchadspl,
		  hbattchar,
		  hbattcoolin,
		  mcon,
		  hbattcurr,
		  hbattvolt,
		  ETE,
		  hbatttemp,
		  seqf,
          chacurrentf,
		  torquedelivered,
		  brakemode,
		  estchatimef,
		  motorRPM,
		  MPH,
		  throttle,
		  brake,
		  pwrt,
		  acc,
		  hvac,
		  VAC,
		  AAC,
		  ampacity,
		  Hz,
		  mincell,
		  maxcell);
  //Serial << logstring2;

  howlongagorx = (unsigned long) millis() - lastrx;
  howlonglastactiverx = (unsigned long) millis() - lastactiverx;

  if (howlongagorx > 500) {
    mincell = 0;
    maxcell = 0;
    deltacell = 0;
    batchadspl = 0;
    brakemode = 0;
    frictionbrake = 0;
    torquedelivered = 0;
    motorRPM = 0;
    MPH = 0;
    hbattchar = 0;
    becmtoggle = 0;
    hbattcoolin = 0;
    mcon = 0;
    hbattcurr = 0;
    currentandmcon = 0;
    ETE = 0;
    hbatttemp = 0;
    hbattvolt = 0;
    throttle = 0;
    pwrt = 0;
    hvac = 0;
  }
  if (howlonglastactiverx > 500) {
    mincell = 0;
    maxcell = 0;
    deltacell = 0;
    hbattvolt = 0;
  }
  if (seq == 1) {
    delay(97);
  }
  else {
    delay(100);
  }

}

void ActiveDiagLoop() {

  if (MPH > 0) { // If we're moving, we're not charging. Clear these values:
    sodbmtoggle = 0;
    pilot = 0;
    AAC = 0;
    VAC = 0;
    ampacity = 0;
    Hz = 0;
  }
  becmdiagcycle = becmdiagcycle * becmtoggle; // Is diagcycle being overridden by the toggle this time?
  
  if (becmdiagcycle == 0) { // BECM Diagnostic Request for Delta Cell Voltage
    myVars.outFrame.id = 0x7E4;
    myVars.outFrame.data.bytes[0] = 0x03;
    myVars.outFrame.data.bytes[1] = 0x22;
    myVars.outFrame.data.bytes[2] = 0x48;
    myVars.outFrame.data.bytes[3] = 0x40;
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
  else if (becmdiagcycle == 1) { //BECM Diagnostic Request for Min Cell Voltage
    myVars.outFrame.id = 0x7E4;
    myVars.outFrame.data.bytes[0] = 0x03;
    myVars.outFrame.data.bytes[1] = 0x22;
    myVars.outFrame.data.bytes[2] = 0x48;
    myVars.outFrame.data.bytes[3] = 0x3F;
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
  else if (becmdiagcycle == 2) { //BECM Diagnostic Request for Voltage
    myVars.outFrame.id = 0x7E4;
    myVars.outFrame.data.bytes[0] = 0x03;
    myVars.outFrame.data.bytes[1] = 0x22;
    myVars.outFrame.data.bytes[2] = 0x48;
    myVars.outFrame.data.bytes[3] = 0x0D;
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

  if (sodbmtoggle == 1) { // If SOBDM toggle is enabled, wait a bit before running down the logic ladder
    delay(20);
  }

  sodbmdiagcycle =  sodbmdiagcycle * sodbmtoggle; // Is diagcycle being overridden by the toggle this time?

  if (sodbmdiagcycle == 1) { //SODBM Diagnostic Request for AC Voltage
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

  else if (sodbmdiagcycle == 2) { //SODBM Diagnostic Request for AC Amps
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

  else if (sodbmdiagcycle == 3) { //SODBM Diagnostic Request for Pilot Signal
    myVars.outFrame.id = 0x7E2;
    myVars.outFrame.data.bytes[0] = 0x03;
    myVars.outFrame.data.bytes[1] = 0x22;
    myVars.outFrame.data.bytes[2] = 0x48;
    myVars.outFrame.data.bytes[3] = 0x61;
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

  else if (sodbmdiagcycle == 4) { //SODBM Diagnostic Request for AC Hz
    myVars.outFrame.id = 0x7E2;
    myVars.outFrame.data.bytes[0] = 0x03;
    myVars.outFrame.data.bytes[1] = 0x22;
    myVars.outFrame.data.bytes[2] = 0x48;
    myVars.outFrame.data.bytes[3] = 0x60;
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
  delay(30); // wait a bit before starting all over again
}

void serialEventRun(void) {
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

  if (ptrBuffer < 6) return; // 4 digit command, =, value is at least 6 characters
  cmdBuffer[ptrBuffer] = 0; // make sure to null terminate
  String cmdString = String();
  unsigned char whichEntry = '0';
  i = 0;

  while (cmdBuffer[i] != '=' && i < ptrBuffer)  {
    cmdString.concat(String(cmdBuffer[i++])); // read out characters from the command string until the = char is reached
  }
  i++; // skip the =
  if (i >= ptrBuffer)  { // is there's noting left to process...
    Serial << ("Command needs a value..ie FILE=myfile.log"); // complain about it on the console
    return; // or, we could use this to display the parameter instead of setting
  }

  // strtol() is able to parse also hex values (e.g. a string "0xCAFE"), useful for enable/disable by device id
  newValue = strtol((char *) (cmdBuffer + i), NULL, 0); // read the rest of the string/chars until the null terminator is reached
  cmdString.toUpperCase(); // and uppercase the command for consistency
  if (cmdString == String("XXXX") )   { // if the command started with 4 uppercase X's
    if (newValue > maxchacurrent && seq == 4)    { // If a value *greater than* max charging current is specified while the sequence is "4", just set charging current to max
      wayup = 1;
      chacurrent = maxchacurrent;
    }
    else if (newValue < 0 && seq == 4)    { // if a value less than 0 is specified while the sequence is 4, just set charging current to zero and not that negative number
      wayup = 1;
      chacurrent = 0;
    }
    else if (newValue < maxchacurrent && newValue > 0 && seq == 4)    { // if new value is positive, less than max charging current and sequence is 4, set charging current to that new value.
      wayup = 1;
      chacurrent = newValue;
    }
  }
  else if (cmdString == String("xxxx") )  { // Seems to be the exact same logic but looking for an input value that shouldn't be possible
    if (newValue > maxchacurrent && seq == 4)    {
      wayup = 1;
      chacurrent = maxchacurrent;
    }
    else if (newValue < 0 && seq == 4)    {
      wayup = 1;
      chacurrent = 0;
    }
    else if (newValue < maxchacurrent && newValue > 0 && seq == 4)    {
      wayup = 1;
      chacurrent = newValue;
    }
  }
}

void handleShortCmd() { // single character commands on serial to force various charging states - most disabled by commenting.
  uint8_t val;

  switch (cmdBuffer[0])
  {

    case '1':
      seq = 1;
      //Can1.enable();
      //digitalWrite(k,HIGH);
      //Serial<<"Now in PRECHARGE PHASE SEQ=1\n";
      break;
    case '2':
      seq = 2;
      //digitalwrite(e,HIGH);
	  //Serial<<"Now in ISO CHECK MODE SEQ=2\n";
      break;
    case '3':
      seq = 3;
      //Serial<<"Now in CONTACT CLOSED MODE SEQ=3\n";
      break;
    case '4':
      seq = 4;
      chacurrent = 1;
      //Serial<<"Now in CHARGING MODE SEQ=4\n";
      break;
    case '5':
      seq = 5;
      //Serial<<"Now in ENDING CHARGE SEQ=5\n";
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
      }
      sodbmdiagcycle =  sodbmdiagcycle * sodbmtoggle;

      break;

    case '7':

      if (becmtoggle == 0)
      {
        becmtoggle = 1;
        becmdiagcycle = 0;
      }
      else
      {
        becmtoggle = 0;
        mincell = 0;
        maxcell = 0;
        becmdiagcycle = 0;

      }
      becmdiagcycle =  becmdiagcycle * becmtoggle;

      break;

    case '9':
      vehstop = 1;
      break;

    case '0':
      seq = 0;

      //initializeCAN();
      //Can1.disable();
      //digitalWrite(k,LOW);
      //digitalwrite(e,LOW);
      //Serial<<"Now in NO COMMS mode. OFF SEQ=0\n";
      break;

  }
  cmdBuffer[0] = 0;
}

void initializeCAN() {
  pinMode(48, OUTPUT); // Set Pin 48 to output, to enable CAN1
  Can1.enable(); // Enable CAN1 RX and TX. TTL data turned into CANHI/CANLO differential signaliing by the EVAL board. CAN1 is used for CHAdeMO comms.
  if (Can1.begin(myVars.datarate, 48)) { // If CAN1 starts successfully on Pin 48 with the saved datarate, set these message filters.
    Can1.setRXFilter(0x108, 0x7FF, false); // We want to receive non-extended messages whose IDs exactly match 0x108 (0x00001101100, CHAdeMO max current)
    Can1.setRXFilter(0x109, 0x7FF, false); // and 0x109 (0x00001101101, Status/Fault flag)...
//	Can1.setRXFilter(1,0,0, false);
//	Can1.setRXFilter(2,0,0, true);
    Can1.setGeneralCallback(handleFrame1); // "Set a callback that will be used for any mailbox that doesn't have a registered callback"
    Can1.disable_tx_repeat(); // ...Disable Tx Repeat for CAN1

    Serial << "\n\nUsing CAN1 - initialization completed at " << myVars.datarate << " \n"; // Tell us on USBSerial that things worked
  }
  else Serial.println("\nCAN1 initialization (sync) ERROR\n"); // Tell us things failed

  pinMode(50, OUTPUT); // Set Pin 50 to output to Enable CAN0
  Can0.enable(); // Enable CAN0 RX and TX. Vehicle side interface that uses the SN65HVD234 to turn binary RX/TX to differential CANHI/CANLO output.
  if (Can0.begin(myVars.datarate, 50)) { // If CAN0 starts successfully on Pin 48 with the saved datarate, set these message filters.
    Can0.setRXFilter(0x24C, 0x7B7, false); // ID 01001001100, Mask 11110110111 - SoC display, HBATT_CHAR, Battery coolant inlet temp
    Can0.setRXFilter(0x07A, 0x7F0, false); // ID 00001111010, Mask 11111110000 - HV Current, main contactor voltage
    Can0.setRXFilter(0x1E4, 0x7FF, false); // ID 00111100100, Mask 11111111111 - Energy to Empty, Batt temp
//	Can0.setRXFilter(0x07D, 0x7F7, false); // Friction Brake Check, Brake pedal
//	Can0.setRXFilter(0x075, 0x7FF, false); // Torque, RPM
    Can0.setRXFilter(0x160, 0x5F2, false); // ID 00101100000, Mask 10111110010 - MPH
//	Can0.setRXFilter(0x165, 0x7FF, false); // Brake on
//	Can0.setRXFilter(0x204, 0x7FF, false); // Throttle
    Can0.setRXFilter(0x117, 0x7FF, false); // ID 00100010111, Mask 11111111111 - "PWRT kW"
//	Can0.setRXFilter(0x368, 0x, false); // ACC kW, HVAC kW
    Can0.setRXFilter(0X7EA, 0x7F9, false); // ID 11111101010, Mask 11111111001 - AC Voltage Input, AC Current Input, AC Current Input, AC Hz Input
//	Can0.setRXFilter(0x7EC, 0x, false); // Min Cell Voltage, Cell Variation, HBatt Volt

    Can0.setGeneralCallback(handleFrame);
    Serial << "\n\nUsing CAN0 - initialization completed at " << myVars.datarate << " \n"; // Tell us on USBSerial that things worked
  }
  else Serial.println("\nCAN0 initialization (sync) ERROR\n"); // Tell us things failed
}

void handleFrame(CAN_FRAME * frame) { // CAN Frames from the car
  lastrx = millis();

  if (frame->id == 0x24C)  {
//	BAT_CHA_DSPL
    sprintf(logstringt, "%02X", frame->data.bytes[6]); // Write the 6th frame data byte to logstringt as a 2 digit hex value
    batchadspl = (float)strtol(logstringt, NULL, 16) / 2; // store half the base 10 equivalent of logstringt in batchadspl as a float
    socint = (int) batchadspl; // truncate decimal portion to store the integer "percentage" of SoC (77%, not 0.77)

//	H_BATT_CHAR
    sprintf(logstringt, "%02X%02X", frame->data.bytes[3], frame->data.bytes[4]); // Write the 3rd and 4th frame data bytes to logstringt as a 4 digit hex value
    hbattchar = (float)strtol(logstringt, NULL, 16) / 10; // store 1/10th the base 10 equivalent of logstringt in batchadspl as a base 10 float

//	Battery Coolant Inlet Temperature
    sprintf(logstringt, "%02X", frame->data.bytes[0]); // Write the 0th frame data byte to logstringt as a 2 digit hex value
    hbattcoolin = (float)(strtol(logstringt, NULL, 16) - 50) * 9 / 5 + 32; // Store converted battery coolant inlet temp as float farenhight value
  }
  else if (frame->id == 0x07A)  {
//	Current and Main Contactor
    sprintf(logstringt, "%02X%02X", frame->data.bytes[0], frame->data.bytes[1]); // Write the first two data bytes to logstringt as a 4 digit hex value
    currentandmcon = (float)strtol(logstringt, NULL, 16); // convert to base 10 float

    if (currentandmcon >= 32768)    { // Main Contactor status == MSB, hbattcurr proportional to 15 LSBs
      mcon = 1;
      hbattcurr = (currentandmcon - 32768) * 0.0500000000029104 - 750.050000071525; // (stored battery current value is ~20x larger and offset by ~15000 !? - check order of operations and bit order )
    }
    else if (currentandmcon < 32768)    {
      mcon = 0;
      hbattcurr = currentandmcon * 0.0500000000029104 - 750.050000071525;
    }
//	Voltage
    sprintf(logstringt, "%02X%02X", frame->data.bytes[2], frame->data.bytes[3]); // bytes 2 and 3 store double the base 10 voltage value as HEX
    hbattvolt = (float)strtol(logstringt, NULL, 16) * 0.5;
  }
  else if (frame->id == 0x1E4)  {
//	ETE
    sprintf(logstringt, "%02X%02X", frame->data.bytes[0], frame->data.bytes[1]); // bytes 0 and 1 store 20x the base 10 ETE value as HEX
    ETE = (float)strtol(logstringt, NULL, 16) / 20;

//	Battery Temperature
    sprintf(logstringt, "%02X", frame->data.bytes[6]); // byte 6 stores HBATTTEMP hex value proportional to farenheiht base 10 float value
    hbatttemp = (float)(strtol(logstringt, NULL, 16) / 2 - 10) * 9 / 5 + 32;
  }
  else if (frame->id == 0x07D)  {
//	Friction Brake Checker
    sprintf(logstringt, "%02X%02X", frame->data.bytes[4], frame->data.bytes[5]);
    frictionbrake = (float)strtol(logstringt, NULL, 16) - 24576; // subtract ( 0110000000000000 ), we only care about the 14 LSBs

//	Brake Pedal
    sprintf(logstringt, "%02X%02X", frame->data.bytes[0], frame->data.bytes[1]); // extract the first 2 bytes as 4 hex characters
    sprintf(logstringts, "%c%c%c", logstringt[1], logstringt[2], logstringt[3]); // read the last 3 hex characters from that extraction
    brake = (float)strtol(logstringts, NULL, 16) * 0.14792899408284; // convert those 3 hex characters into their base 10 float equivalent and multiply them by a constant
  }
  else if (frame->id == 0x075)  {
//	Torque Delivered
    sprintf(logstringt, "%02X%02X", frame->data.bytes[2], frame->data.bytes[3]); // extract the 2nd and 3rd bytes as 4 hex characters
    torquedeliveredint = (int)strtol(logstringt, NULL, 16); // convert them to a base 10 integer ans store it
    torquedeliveredint = (torquedeliveredint & 16383); // Bitwise AND the stored integer against 16383 (0011111111111111) to null the 2 MSBs
    torquedelivered = (0.1 * torquedeliveredint - 1000) * 0.737562149; // transform to final value

//	RPM
    sprintf(logstringt, "%02X%02X", frame->data.bytes[6], frame->data.bytes[7]); // extract 6th and 7th bytes as 4 hex characters
    motorRPM = ((float)strtol(logstringt, NULL, 16) - 45268 )*2; // transform to final float value with... some manner of black magic? ( -1011000011010100 ?)
  }
  else if (frame->id == 0x160)  {
//	MPH
    sprintf(logstringt, "%02X%02X", frame->data.bytes[2], frame->data.bytes[3]); // extract the 2nd and 3rd bytes as 4 hex characters
    MPH = (float)strtol(logstringt, NULL, 16); // turn them into a base 10 float (KPH * 100)
    MPH = (MPH / 100) * 0.621371; // MPH
  }
  else if (frame->id == 0x165)  {
//	Brake On
    sprintf(logstringt, "%02X", frame->data.bytes[4]); // extract 4th byte as 2 hex characters
    brakeon = (int)strtol(logstringt, NULL, 16); // convert to integer - 0 to 255
    if (brakeon == 16)    { // if extracted hex was 0x10
      vehstop = 1; // vehicle is stopped
    }
  }
  else if (frame->id == 0x204)  {
//	Throttle
    sprintf(logstringt, "%02X%02X", frame->data.bytes[0], frame->data.bytes[1]); // extract the first 2 bytes as 4 hex characters
    sprintf(logstringts, "%c%c%c", logstringt[1], logstringt[2], logstringt[3]); // read the last 3 hex characters from that extraction
    throttle = (float)strtol(logstringts, NULL, 16) / 10; // convert those 3 hex characters into their base 10 float equivalent and divide them by a constant
  }
  else if (frame->id == 0x117)  {
//	PWRT kW
    sprintf(logstringt, "%02X%02X", frame->data.bytes[4], frame->data.bytes[5]); // extract 4th and 5th bytes as 4 hex characters
    pwrt = (float)strtol(logstringt, NULL, 16) - 41215; // transform to final float value with... some manner of black magic? ( -1010000011111111 ?)
  }
  else if (frame->id == 0x368)  {
//	ACC kW
    sprintf(logstringt, "%02X%02X", frame->data.bytes[4], frame->data.bytes[5]); // extract 4th and 5th bytes as 4 hex characters
    acc = (float)strtol(logstringt, NULL, 16) / 100; // Convert to float and divide by 100 for (AC?) Current
    acc = acc * hbattvolt / 1000; // Multiply by batt voltage to get watts and divide by 1000 for kW

//	HVAC kW
    sprintf(logstringt, "%02X%02X", frame->data.bytes[0], frame->data.bytes[1]); // extract the first 2 bytes as 4 hex characters
    sprintf(logstringts, "%c%c%c", logstringt[1], logstringt[2], logstringt[3]); // read the last 3 hex characters from that extraction
    hvac = (float)strtol(logstringts, NULL, 16) * 0.005; // convert those 3 hex characters into their base 10 float equivalent and multiply them by a constant
  }
  else if (frame->id == 0X7EA)  {
    if (sodbmdiagcycle == 1)    {
//	AC Voltage Input
      sprintf(logstringt, "%02X%02X", frame->data.bytes[4], frame->data.bytes[5]); // extract 4th and 5th bytes as 4 hex characters
      VAC = (float)strtol(logstringt, NULL, 16) / 100; // convert those hex characters into their base 10 float equivalent and divide them by a constant
      sodbmdiagcycle = 2; // increment sobdmdiagcycle to next diagcycle value and exit the conditional ladder
    }
    else if (sodbmdiagcycle == 2)    {
//	AC Current Input
      sprintf(logstringt, "%02X", frame->data.bytes[4]); // extract 4th byte as 2 hex characters
      AAC = (float)strtol(logstringt, NULL, 16); // convert those hex characters into their base 10 float equivalent
      sodbmdiagcycle = 3; // increment sobdmdiagcycle to next diagcycle value and exit the conditional ladder
    }
    else if (sodbmdiagcycle == 3)    {
//	AC Current Input
      sprintf(logstringt, "%02X", frame->data.bytes[4]); // extract 4th byte as 2 hex characters
      pilot = (float)strtol(logstringt, NULL, 16) / 2; // convert those hex characters into their base 10 float equivalent and divide them by a constant for pilot duty cycle
      if (pilot >= 10 && pilot <= 85)      { // if Pilot duty cycle between 10% and 85%
        ampacity = (float) pilot * 0.6; // available current is 6/10 the integer pilot value
      }
      else if (pilot > 85 && pilot <= 96)      { // more of the same
        ampacity = (float) ((pilot - 64) * 2.5);
      }
      sodbmdiagcycle = 4; // increment sobdmdiagcycle to next diagcycle value and exit the conditional ladder
    }
    else if (sodbmdiagcycle == 4)    {
//	AC Hz Input
      sprintf(logstringt, "%02X", frame->data.bytes[4]); // extract 4th byte as 2 hex characters
      Hz = (float)strtol(logstringt, NULL, 16) / 2;

      sodbmdiagcycle = 1; // reset sobdmdiagcycle diagcycle value and exit the conditional ladder
    }
    sodbmdiagcycle = sodbmdiagcycle * sodbmtoggle; // if sobdm is toggled off, diagcycle becomes 0 and won't enter the ladder until toggle changes.
  }
  else if (frame->id == 0X7EC)  {
    lastactiverx = millis(); // update the timestamp for the last 0X7EC frame
    if (becmdiagcycle == 0)    {
//	Min Cell Voltage
      sprintf(logstringt, "%02X%02X", frame->data.bytes[4], frame->data.bytes[5]); // Extract the 4th and 5th bytes as 4 hex characters...
      mincell = (float)strtol(logstringt, NULL, 16) / 1000; // and convert them to the base 10 float equivalent divided by 1000 for the minimum cell voltage
      becmdiagcycle = 1; // Toggle becmdiagcycle to next diagcycle value and exit the conditional ladder
    }
    else if (becmdiagcycle == 1)    {
//	Cell Variation
      sprintf(logstringt, "%02X", frame->data.bytes[4]); // Extract the 4th and 5th bytes as 4 hex characters...
      deltacell = (float)strtol(logstringt, NULL, 16) / 100; // and convert them to the base 10 float equivalent divided by 100 for the difference between the lowest cell voltage and highest
      becmdiagcycle = 0; // Toggle becmdiagcycle to next diagcycle value and exit the conditional ladder
      maxcell = (float) deltacell + mincell + 0.005; // Calculate the explicit value of the highest cell voltage

    }
/*  else if (becmdiagcycle == 2)    {
      sprintf(logstringt, "%02X%02X", frame->data.bytes[4], frame->data.bytes[5]);
      hbattvolt = (float)strtol(logstringt, NULL, 16) / 100;
      becmdiagcycle = 0;
    }*/
  }
}

void handleFrame1(CAN_FRAME * frame1) { // CAN Frames from the CHAdeMO connection

  if (frame1->id == 0x108) { // Max Charging Current Avaialable
    sprintf(logstringt1, "%02X", frame1->data.bytes[3]); // Write the 3rd byte of frame data to logstringt1 as a 2 char hex number
    maxchacurrent = (int)strtol(logstringt1, NULL, 16); // Read logstringt1 as a base 16 number and store it as an int in maxchacurrent
    compat = 1;

    if (seq == 1)    {
      maxchatime = float (((-0.5385 * maxchacurrent) + 104)) * (100 - socint) / 100 + 6; // calculate max remaining charge time
      estchatime = float (((-0.4 * maxchacurrent) + 77)) * (100 - socint) / 100 + 1; // estimate remaining charge time
    }
  }
  else if (frame1->id == 0x109) { // Status/Fault Flag
    sprintf(logstringt1, "%02X", frame1->data.bytes[5]); // Write the 5th frame data byte to logstringt1 as a 2 char hex number
    chastatusfault = (int)strtol(logstringt1, NULL, 16); // Read logstringt1 as a base 16 number and store it as an inti in chastatusfault

    if (chastatusfault >= 32)    { // Stop charging if status/fault >= 32 (6th or higher bit = 1)
      chastop = 1;
    }
    else    {
      chastop = 0;
    }
    if ((chastatusfault % 2) == 0)    { // Enable charging = LSB
      chaenable = 0;
    }
    else    {
      chaenable = 1;
    }
/*	Serial<<"cha enable = ";
    Serial<<chaenable;
    Serial<<"     chastop = ";
    Serial<<chastop;
    Serial<<"\n";*/
  }

/*	sprintf(logstringc1,"Rcvd msgID; %03X; %02X; %02X; %02X; %02X; %02X; %02X; %02X; %02X;\n", frame1->id, frame1->data.bytes[0],
     frame1->data.bytes[1],frame1->data.bytes[2], frame1->data.bytes[3], frame1->data.bytes[4], frame1->data.bytes[5], frame1->data.bytes[6],
     frame1->data.bytes[7]);
     Serial << logstringc1;*/
}

void sendCAN(int which) { // Comments are lies, "which" doesn't change the interface on which the frame sends
/*		char buffer[140];

        sprintf(buffer,"Sent msgID 0x%03X; %02X; %02X; %02X; %02X; %02X; %02X; %02X; %02X;\n", myVars.outFrame.id,
        myVars.outFrame.data.bytes[0], myVars.outFrame.data.bytes[1], myVars.outFrame.data.bytes[2],
        myVars.outFrame.data.bytes[3], myVars.outFrame.data.bytes[4], myVars.outFrame.data.bytes[5],
        myVars.outFrame.data.bytes[6], myVars.outFrame.data.bytes[7]);
        Serial<<buffer;
        Serial<<"\n";
*/    
//	int time=millis();
//	Serial<<time;
//	Serial<<"\n";

	myVars.outFrame.length = 8;
	myVars.outFrame.fid = 0;
	myVars.outFrame.rtr = 1;
	myVars.outFrame.priority = 0;
	myVars.outFrame.extended = false;
//	int time=micros();
//	Serial<<time<<"\n"<<buffer<<"\n";

//	Serial<<"I'm running\n";

  Can1.sendFrame(myVars.outFrame);    // Mail it
}

void defaults() {
  myVars.CANdo = 2; // Use Serial port if 2, CAN0 if 0 or CAN1 if 1
  myVars.transmitime = 500;
  myVars.logger = false;
  myVars.outFrame.length = 8;  // Data payload 8 bytes
  myVars.outFrame.rtr = 0;  // Data payload 8 bytes
  myVars.goodEEPROM = 200;
  myVars.datarate = 500000;
}
