//Installed on Uno in juction box 4/10/2021 this is the connection between hetronic out (wire#12 0-5V) and 5K digtial pot to control iGX 390 RPM

#include <SPI.h>
byte address = 0x00;
int CS = 10;
//int hetronicIN = A1;
int currentPosition;
int goalSpeed;
int lastSpeed;
byte PMW_PIN = 3;
int pmwValue;

int digitalPotWrite(int value) {
  digitalWrite(CS, LOW);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(CS, HIGH);
}

void setup() {
  pinMode (CS, OUTPUT);
  //pinMode (hetronicIN , INPUT);
  pinMode (PMW_PIN, INPUT);
  SPI.begin();
  Serial.begin(9600);
}

void loop() {
  //currentPosition = analogRead(hetronicIN);
  pmwValue = pulseIn(PMW_PIN, HIGH);
  Serial.print("PMW: ");
  Serial.println(pmwValue);
  //goalSpeed = map(currentPosition, 0, 1023, 0, 4);
  if(pmwValue < 500  &&                    goalSpeed != 0) {goalSpeed = 0; digitalPotWrite(0); Serial.print("SpeedChanged: "); Serial.println(goalSpeed);}
  if(pmwValue > 600  && pmwValue < 1200 && goalSpeed != 1) {goalSpeed = 1; digitalPotWrite(5); delay(2000); digitalPotWrite(12); delay(2000); digitalPotWrite(22); }
  if(pmwValue > 1600 && pmwValue < 2200 && goalSpeed != 2) {goalSpeed = 2; digitalPotWrite(60); Serial.print("SpeedChanged: "); Serial.println(goalSpeed);}
  if(pmwValue > 2600 && pmwValue < 3200 && goalSpeed != 3) {goalSpeed = 3; digitalPotWrite(95); Serial.print("SpeedChanged: "); Serial.println(goalSpeed);}
  if(pmwValue > 3600 &&                    goalSpeed != 4) {goalSpeed = 4; digitalPotWrite(127); Serial.print("SpeedChanged: "); Serial.println(goalSpeed);}
  /*if (goalSpeed != lastSpeed) {
    lastSpeed = goalSpeed;
    Serial.print("SpeedChanged: ");
    Serial.println(goalSpeed);
  }*/
  Serial.print("CurrentPostion: ");
  Serial.println(currentPosition);
  Serial.print("goalSpeed: ");
  Serial.println(goalSpeed);
  delay(1000);
}



/*
550-640 - off

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



  //trailer levelswitch to kill pump
  tankFull = digitalRead(levelSwitch);
  //Serial.print("TankFULL Status");
  //Serial.println(tankFull);
  if (tankFull == 1) {
    digitalPotWrite(1);
    Speed = 0;
    SpeedLock = 1;
  }

*/
