#include <SPI.h>
byte address = 0x00;
int CS= 53;

//////////////***********Loop SPI Code***********//////////////////
    /*for (int i = 0; i <= 128; i++)
    {
       digitalPotWrite(i);
       delay(10); 
    }
    delay(500);
    for (int i = 128; i >= 0; i--)
    {
       digitalPotWrite(i);
       delay(500);
    }
    
    int i = 25;
    digitalPotWrite(i);
    delay(1000);*/

int digitalPotWrite(int value)
{
digitalWrite(CS, LOW);
SPI.transfer(address);
SPI.transfer(value);
digitalWrite(CS, HIGH);
}

double channel[8];
//X8R outputs to DPIN inputs on arduino
const int CH1 = 47;  //Boom up&down
const int CH2 = 45;  //Boom swivel
const int CH3 = 43;  //End Boom
const int CH4 = 41;  //ING & End Valve small left Switch
const int CH5 = 39;  //Choke and Starter motor left MOM Switch
const int CH6 = 37;  //Speed Dial
const int CH7 = 35;  //Lights
const int CH8 = 33;  //LockoutSW and Choke disable

//OPTPUTS TO RELAYS
const int raiseBoom = 2;
const int lowerBoom = 3;
const int swingBoomR = 4;
const int swingBoomL = 5;
const int endBoomIn = 6;
const int endBoomOut = 7;
const int ING = 8;
const int Valve = 9;
const int Choke = 10;
const int Starter = 11;
const int LightRelay = 12; // output to light relay
//output to linear actuator relay
const int DigPot1 = 13;  //DigtialPot enable  //move act to throttle up
const int DigPot2 = 16;  //DigtialPot enable  //move act to throttle down


//const int sensorPin = A0;  //Input from throttle pot
const int levelSwitch = 23;

//actuator info
int goalPosition = 0;
int CurrentPosition = 0;
boolean Extending = false;
boolean Retracting = false;
int Speed = 0;
int SpeedLock = 0;
int LastSpeed = 0;

const int lowerus = 1250;
const int higherus = 1750;

const int shutdownhigher = 1650;
const int shutdownlower = 1350;
const int SWUpper = 1700;
const int SWLower = 1300;

int boomovingState = 0;  //0 netural   1 is up  2 is down
int boomswingState = 0;  //0 netural   1 is right  2 is left
int boomendState = 0;  //0 netural   1 is out  2 is in
int INGState = 0;   //0 off  1 is on
int StartState = 0;
int ChokeState = 0;
int ChokeDisable = 0;
int ValveState = 0;
int LightState = 0;
int tankFull = 0;
int alreadyDone = 0;

void setup() {
  pinMode (CS, OUTPUT);
  SPI.begin();
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);
  pinMode(CH7, INPUT);
  pinMode(CH8, INPUT);

  pinMode(raiseBoom, OUTPUT);
  pinMode(lowerBoom, OUTPUT);
  pinMode(swingBoomR, OUTPUT);
  pinMode(swingBoomL, OUTPUT);
  pinMode(endBoomIn, OUTPUT);
  pinMode(endBoomOut, OUTPUT);
  pinMode(ING, OUTPUT);
  pinMode(Valve, OUTPUT);
  pinMode(Choke, OUTPUT);
  pinMode(Starter, OUTPUT);
  pinMode(DigPot1, OUTPUT);
  pinMode(DigPot2, OUTPUT);
  pinMode(LightRelay, OUTPUT);

  //pinMode(sensorPin, INPUT);
  pinMode(levelSwitch, INPUT);

  //preset the relays to high
  digitalWrite(raiseBoom, HIGH);
  digitalWrite(lowerBoom, HIGH);
  digitalWrite(swingBoomR, HIGH);
  digitalWrite(swingBoomL, HIGH);
  digitalWrite(endBoomIn, HIGH);
  digitalWrite(endBoomOut, HIGH);
  digitalWrite(ING, HIGH);
  digitalWrite(Valve, HIGH);
  digitalWrite(Choke, HIGH);
  digitalWrite(Starter, HIGH);
  digitalWrite(DigPot1, LOW);  ///set to LOW here???
  digitalWrite(DigPot2, LOW);
  digitalWrite(LightRelay, HIGH);


  //preset >goalPosition< to current position is tis a good idea?? or Delete??
  //CurrentPosition = analogRead(sensorPin);
  tankFull = digitalRead(levelSwitch);

  //pinMode(13, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  //delay(1000);
  channel[1] = pulseIn(CH1, HIGH);
  channel[2] = pulseIn(CH2, HIGH);
  channel[3] = pulseIn(CH3, HIGH);
  channel[4] = pulseIn(CH4, HIGH);
  channel[5] = pulseIn(CH5, HIGH);
  channel[6] = pulseIn(CH6, HIGH);
  channel[7] = pulseIn(CH7, HIGH);
  channel[0] = pulseIn(CH8, HIGH);
  /*Serial.print("Chan1::  ");
  Serial.println(channel[1]);
  Serial.print("Chan2::  ");
  Serial.println(channel[2]);
  Serial.print("Chan3::  ");
  Serial.println(channel[3]);
  Serial.print("Chan4::  ");
  Serial.println(channel[4]);
  Serial.print("Chan5::  ");
  Serial.println(channel[5]);
  Serial.print("Chan6::  ");
  Serial.println(channel[6]);
  Serial.print("Chan7::  ");
  Serial.println(channel[7]);
  Serial.print("Chan8::  ");
  Serial.println(channel[0]);
  delay(1000);*/


  //Channel 3 Startup Lockout
  if (alreadyDone == 0) {
    digitalPotWrite(2);
    while (channel[3] < shutdownhigher) {
      channel[3] = pulseIn(CH3, HIGH);
      alreadyDone = 1;
    }
  }
  
  
  //Channel 8 LockOut Code Loop stays right here
  // NEED TO ADD CODE TO KILL BOOM MOVING, STARTER AND CHOKE CODES  any movment!!!

  while (channel[0] < 1500) {
    channel[0] = pulseIn(CH8, HIGH);
    //Serial.println("LOCKDOWN");
  }
  //Channel 8 Choke Disable Code
  if (channel[0] > 1950) {
    ChokeDisable = 1;
  }
  else {
    ChokeDisable = 0;
  }
  //Serial.println(ChokeDisable);

  //CHANNEL 1 CODE  //0 netural   1 is up  2 is down //shutdownhigher = 1650 shutdownlower = 1350
  if (boomovingState == 1) {
    if (channel[1] < shutdownhigher ) {
      digitalWrite(raiseBoom, HIGH);
      boomovingState = 0;
      //Serial.println("stopraiseing boom");
    }
  }
  if (boomovingState == 2) {
    if (channel[1] > shutdownlower ) {
      digitalWrite(lowerBoom, HIGH);
      boomovingState = 0;
      //Serial.println("stop lowering boom");
    }
  }
  if (boomovingState == 0) {
    if (channel[1] < lowerus) {
      digitalWrite(lowerBoom, LOW);
      boomovingState = 2;
      //Serial.println("lowerBoom");
    }
  }
  if (boomovingState == 0) {
    if (channel[1] > higherus) {
      digitalWrite(raiseBoom, LOW);
      boomovingState = 1;
      //Serial.println("raiseBoom");
    }
  }

  //CHANNEL 2 CODE  //0 netural   1 is right  2000us  2 is left 988us  shutdownhigher = 1650 shutdownlower = 1350;
  if (boomswingState == 1) {
    if (channel[2] < shutdownhigher) {   //1500 < 1650
      digitalWrite(swingBoomR, HIGH);
      boomswingState = 0;
      //Serial.println("stop swing Right");
    }
  }
  if (boomswingState == 2) {
    if (channel[2] > shutdownlower) {   //1500 > 1350
      digitalWrite(swingBoomL, HIGH);
      boomswingState = 0;
      //Serial.println("stop swing Left");
    }
  }
  if (boomswingState == 0) {
    if (channel[2] < lowerus) {
      digitalWrite(swingBoomL, LOW);
      boomswingState = 2;
      //Serial.println("swingBoomLeft");
    }
  }
  if (boomswingState == 0) {
    if (channel[2] > higherus) {
      digitalWrite(swingBoomR, LOW);
      boomswingState = 1;
      //Serial.println("SwingBoomRigh");
    }
  }


  //CHANNEL 3 CODE  //0 netural   1 is out  2 is in //shutdownhigher = 1650 shutdownlower = 1350
  if (boomendState == 1) {
    if (channel[3] < shutdownhigher) {
      digitalWrite(endBoomOut, HIGH);
      boomendState = 0;
      //Serial.println("stopEndBoomOut");
    }
  }
  if (boomendState == 2) {
    if (channel[3] > shutdownlower) {
      digitalWrite(endBoomIn, HIGH);
      boomendState = 0;
      //Serial.println("stopEndBoomIn");
    }
  }
  if (boomendState == 0) {
    if (channel[3] < lowerus) {
      digitalWrite(endBoomIn, LOW);
      boomendState = 2;
      //Serial.println("endBoomIn");
    }
  }
  if (boomendState == 0) {
    if (channel[3] > higherus) {
      digitalWrite(endBoomOut, LOW);
      boomendState = 1;
      //Serial.println("endBoomOut");
    }
  }


  //CHANNEL 4 CODE  //0 off  1 is on  //SWUpper = 1700 SWLower = 1300
  if (INGState == 0) {
    if (channel[4] < SWUpper && SpeedLock == 0) { // like this?? 1500 < 1700
      digitalWrite(ING, LOW);
      INGState = 1;
      //Serial.println("ING is ON!!");
    }
  }


  if (INGState == 1) {
    if (channel[4] > SWUpper) {   // like this??  2000  > 1700 this code also used in tankFull down below
      digitalWrite(ING, HIGH);
      INGState = 0;
      //Serial.println("ING is OFF!!");
    }
  }

  if (ValveState == 0) {
    if (channel[4] < SWLower) {   // like this??  988  < 1300
      digitalWrite(Valve, LOW);
      ValveState = 1;
      //Serial.println("VALVE is OPEN!!");
    }
  }

  if (ValveState == 1 && Speed == 0) { //need to add delay in case speed is rapdily decreased and valve tries to close before pump idle
    if (channel[4] > SWLower) {   // like this??  1500  > 1300
      digitalWrite(Valve, HIGH);
      ValveState = 0;
      //Serial.println("VALVE is Closed!!");
    }
  }


  //CHANNEL 5 CODE  Choke delay Starter motor //0 off  1 is on  //SWUpper = 1700 SWLower = 1300
  //ADD CODE TO DISABLE START IF SPEED IS GREATER THAN IDLE
  if (StartState == 0) {
    if (channel[5] > SWUpper && INGState == 1 && Speed == 0) { // like this?? 2000 > 1700
      if (ChokeDisable == 0) {
        //digitalWrite(Choke, LOW);
        //ChokeState = 1;
        //Serial.println("Choke is ON!!");
      }
      //delay(500);
      digitalWrite(Starter, LOW);
      StartState = 1;
      //Serial.println("Starter is ON!!");
    }
  }


  if (StartState == 1) {
    if (channel[5] < SWLower || INGState == 0) {   // like this??  988 < 1300
      digitalWrite(Starter, HIGH);
      StartState = 0;
      digitalPotWrite(2);
      //Serial.println("Starter is OFF!!");
      //digitalWrite(Choke, HIGH);
      //ChokeState = 0;
      //Serial.println("Choke is OFF!!");
    }
  }


  //CHANNEL 6 CODE  Pump Speed Dial Idle==1000 IB==  1A== 1==1200  2==1500  3==1800 4==2000
  //Speed input tracking to be tracked digtially or with pot SW on actuator??
  //Speed == 1,2,3,4 need to be disabled if Valve is == 0

  //Idle Code  Speed 0
  if (channel[6] < 1020 && Speed != 0 && SpeedLock == 0) {
    //Serial.println("Idle");
    Speed = 0;
    digitalPotWrite(0);
  }

  //Quater Code
  if (channel[6] > 1120 && channel[6] < 1350 && Speed != 1 && ValveState == 1 && SpeedLock == 0) { //1200 > 1120   1200 < 1350
    //Serial.println("1/4 Speed");
    Speed = 1;
    digitalPotWrite(5);
    delay(2000);
    digitalPotWrite(12);
    delay(1000);
    digitalPotWrite(22);
  }

  //Half Code
  if (channel[6] > 1400 && channel[6] < 1630 && Speed != 2 && ValveState == 1 && SpeedLock == 0) { //1200 > 1120   1200 < 1350
    //Serial.println("1/2 Speed");
    Speed = 2;
    digitalPotWrite(60);
  }

  //Three-Quarter Code
  if (channel[6] > 1670 && channel[6] < 1880 && Speed != 3 && ValveState == 1 && SpeedLock == 0) { //1200 > 1120   1200 < 1350
    //Serial.println("3/4 Speed");
    Speed = 3;
    digitalPotWrite(95);
  }

  //Full Throttle Code
  if (channel[6] > 1920 && Speed != 4 && ValveState == 1 && SpeedLock == 0) { //1200 > 1120   1200 < 1350
    //Serial.println("1 Speed");
    Speed = 4;
    digitalPotWrite(127);
  }

  //Cordinate Speed Position with actuator GoalPosition
  goalPosition = map(Speed, 0, 4, 0, 128);    //orignal was Speed, 0, 4, 280, 745
  //Serial.print("MappedNumber   ");
  //Serial.println(goalPosition);

  /*if (LastSpeed == 0 && Speed == 1) {
     digitalPotWrite(5);
     delay(2000);
     digitalPotWrite(12);
     delay(2000);
     digitalPotWrite(22);
     LastSpeed = Speed;
  }
  if (Speed != LastSpeed) {
    // set new goal position
    LastSpeed = Speed;
    digitalPotWrite(goalPosition);
  }*/


  //Read actuator position on throttle
  //CurrentPosition = analogRead(sensorPin);
  //Serial.println(CurrentPosition);

  //trailer levelswitch to kill pump
  tankFull = digitalRead(levelSwitch);
  //Serial.print("TankFULL Status");
  //Serial.println(tankFull);
  if (tankFull == 1) {
    digitalPotWrite(1);
    Speed = 0;
    SpeedLock = 1;
  }
  //continued code of tank full
  if (tankFull == 1 && INGState == 1) {
    delay(4000);             //wait for pump to idle
    digitalWrite(ING, HIGH); //shutdown the pump
    INGState = 0;            //Keep track of ignition state
  }
  if (channel[4] > SWUpper && SpeedLock == 1) {   // like this??  2000  > 1700
    SpeedLock = 0;
    //Serial.print(SpeedLock);
    //Serial.println("  Speed Lockout Reset");
  }


  //Channel 7 Code LIGHTS  SWUpper = 1700; SWLower = 1300
  if (LightState == 0 && channel[7] > SWUpper) {
    digitalWrite(LightRelay, LOW);
    LightState = 1;
    //Serial.println("lIGHTSA on!!");
  }

  if (LightState == 1 && channel[7] < SWLower) {
    digitalWrite(LightRelay, HIGH);
    LightState = 0;
    //Serial.println("lIGHTSA off!!");
  }
}
