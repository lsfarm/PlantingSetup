#include <SPI.h>
byte address = 0x00;
int CS = 10;
int hetronicIN = A5;
int currentPosition;
int goalSpeed;
int lastSpeed;

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



void setup() {
  pinMode (CS, OUTPUT);
  pinMode (hetronicIN , INPUT);
  SPI.begin();
  Serial.begin(9600);
}

void loop() {
  currentPosition = analogRead(hetronicIN);
  goalSpeed = map(currentPosition, 0, 1023, 0, 4);
  if(goalSpeed != lastSpeed) {
    int potMap = map(goalSpeed, 0, 4, 0, 127);
    digitalPotWrite(potMap);
  }
  //Serial.println(currentPosition);
}



/*

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
