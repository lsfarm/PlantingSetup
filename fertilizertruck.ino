/////////************* **********/////////
//          Blynk Assignments           //
/////////************* **********/////////
/*
V0  - Time stamps
V1-9 tank leel stuff
V5  - motorKill>>to delete
V10-20 tank level stuff

V20 - sendtoblynkenable
v21 - getBatVoltage();
V23 - Relay 8(LPU Sennor) power toggle
V28 - megaPower >> disontinue??
V29 - motorKill >> discontinue??
//////////********** Blynk **********//////////
#include <blynk.h>
char auth[] = "DRwlkQMXGrxDVNGbPujY3IFNAjwJeX6p";  //DRwlkQMXGrxDVNGbPujY3IFNAjwJeX6p
BlynkTimer timer;
int timerNA = 99;
int batVoltMeterDelay = timerNA;
bool sendtoblynkenable = 1;

#define LPUSig      A1 // HARDWARE CONNECTIONS: Boron A1 --> LPU Ground > Sensor Signal Line
#define batIN12V    A2

// 8 channel relay
#define ON  0
#define OFF 1
bool DisSensorPowerState = 1; // off is ON
bool megapowerstate = 1;
bool motorkillstate = 1;
bool motorkilllockout = 0;
long motorkillreset = 60000;  //60000

//////////********** Things Speak **********//////////
#include <ThingSpeak.h>

TCPClient client;

unsigned long myChannelNumber = 943747;    /*Thingspeak channel id*/
const char * myWriteAPIKey = "8YZ6XYZX2LDR9A2C";/*Channel's write API key*/

//////////********** GPS Shield **********//////////
/*PARTS:
    Particle Boron: < https://www.sparkfun.com/products/15069 >
    Adafruit Ultimate GPS FeatherWing:  < https://www.adafruit.com/product/3133 >
    UBEC Step-Down Converter:  < https://www.adafruit.com/product/1385 >
  HARDWARE CONNECTIONS:
    Boron GND --> GPS GND
    Boron 3.3V --> GPS 3.3V
    Boron D10 --> GPS TX
    Boron D9 --> GPS RX
*/

// Add the Adafruit GPS library
#include <Adafruit_GPS.h>

// Name the Serial port
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

//uint32_t timer = millis();
//uint32_t timer2 = millis();

//////////**********// End GPS Shield //**********//////////

//////////********** My Variables **********////////// 
float currentlatitude = 36.438092;
float currentlongitude = -102.947385;

bool movementDetected = true;

int tankLevel = 999;
float cuLevel = 6480;
int calibratedlevel = 0.0;

float batteryVolts = 20;
int rawbatvolt = 22;

#define Relay1          D0
#define motorkillrelay  D1
#define megapowerrelay  D2
#define Relay2          D3
#define Relay5          D4
#define R6_battChecker  D5  //battery checker power
#define Relay7          D6  //Modbus Power
#define Rel8DisPower    D7  //LPU power

//////////**********// End My Variables //**********////////// 

//////////********** Blynk Stuff **********//////////
BLYNK_WRITE(V20)  {
    sendtoblynkenable = param.asInt(); // assigning incoming value from pin to a variable
}
BLYNK_WRITE(V21) {  //Push Button that prints current time back to V2
    int button1 = param.asInt();
    if (button1) {
        getBatVoltage();
    }
}
BLYNK_WRITE(V22)  {
    DisSensorPowerState = param.asInt();
    digitalWrite(Rel8DisPower, DisSensorPowerState);
}
BLYNK_WRITE(V28)  { //to be discontinued
    megapowerstate = param.asInt();
    digitalWrite(megapowerrelay, megapowerstate);
    digitalWrite(Relay7, megapowerstate);
}
BLYNK_WRITE(V29)  { //to be discontinued
    int motorkillstate = param.asInt();
    if (motorkillstate == 0 && motorkilllockout == 0)  {  //&& motorkilllockout == 0
       digitalWrite(motorkillrelay, motorkillstate);
       int delayedOff = timer.setTimeout(3000, MOTOR_KILLoff); 
       int lockout = timer.setTimeout(motorkillreset, resetlockout);
       motorkilllockout = 1;
    }
    
}

void setup() {
  Time.zone(-5);
  ThingSpeak.begin(client);
  Blynk.begin(auth);
  //timer.setInterval(5000L, readLPUSensor);
  timer.setInterval(5000L, writeTOblynk);  //60000
  //timer.setInterval(300000L, writeGPS2ThingSpeak); //this is called ifmoved()
  timer.setInterval(59000L, checkmovement);     //every minute
  timer.setInterval(601000L, checksmallmove);   //every 10 minute
  timer.setInterval(3600000L, getBatVoltage);   //every 60 minute >>move to button press mayb not
  
  Blynk.virtualWrite(V20, sendtoblynkenable);
  Blynk.virtualWrite(V21, DisSensorPowerState);
  Blynk.virtualWrite(V22, megapowerstate);
  Blynk.virtualWrite(V23, motorkillstate);


  // Publish a little welcome message
  Particle.publish("FerTruck GPS tracker online");
  Particle.subscribe("MotorKill", motorkill, MY_DEVICES);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // Turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  
  pinMode (LPUSig,          INPUT); //A1
  pinMode (batIN12V,        INPUT); //A2
  
  pinMode (Relay1,          OUTPUT);
  pinMode (motorkillrelay,  OUTPUT); 
  pinMode (megapowerrelay,  OUTPUT); 
  pinMode (Relay2,          OUTPUT);
  pinMode (Relay5,          OUTPUT);
  pinMode (R6_battChecker,  OUTPUT);
  pinMode (Relay7,          OUTPUT);
  pinMode (Rel8DisPower,    OUTPUT);

  
  digitalWrite (Relay1,         OFF); //<<test this
  digitalWrite (motorkillrelay, motorkillstate);
  digitalWrite (megapowerrelay, megapowerstate);
  digitalWrite (Relay2,         HIGH);
  digitalWrite (Relay5,         HIGH);
  digitalWrite (R6_battChecker, HIGH);
  digitalWrite (Relay7,         megapowerstate);
  digitalWrite (Rel8DisPower,   DisSensorPowerState); //powered on all the time for now



  delay(1000);
}

void loop() {
    Blynk.run();
    timer.run();
    readGPS();
}  //end void loop

void MOTOR_KILLoff()  {
    digitalWrite(motorkillrelay, HIGH);
}
void resetlockout()  {
    motorkilllockout = 0;
}
void motorkill(const char *event, const char *data) {
    if(megapowerstate == 0 && motorkilllockout == 0)  // if boom is powered from Blynk
    {
        digitalWrite(motorkillrelay, LOW);
        int delayedOff = timer.setTimeout(3000, MOTOR_KILLoff);
        int lockout = timer.setTimeout(motorkillreset, resetlockout);
        motorkilllockout = 1;
        Blynk.notify("Motor Shutting Down");
    }
}

void writeGPS2ThingSpeak()  {
    ThingSpeak.setField(1, String(GPS.latitudeDegrees));
    ThingSpeak.setField(2, String(GPS.longitudeDegrees));
    ThingSpeak.setField(3, String(calibratedlevel));
    //ThingSpeak.setField(3, String(5250.8));
    //ThingSpeak.setField(4, String(GPS.latitudeDegrees) + ", " + String(GPS.longitudeDegrees));//Field 4 update through particle intergration
    ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    
    //Particle.publish("Landed writeGPS2Things");
    //Particle.publish("LocationWGPS2TS: ", String(GPS.latitudeDegrees) + ", " + String(GPS.longitudeDegrees));
}// end write2ThingsSpeak()

void writeTOblynk()  {
    if(sendtoblynkenable == 1)  {
        Blynk.virtualWrite(V0, Time.format("%r - %a %D"));
        Blynk.virtualWrite(V1, tankLevel); 
        Blynk.virtualWrite(V2, calibratedlevel);
        Blynk.virtualWrite(V5, motorkilllockout);
        
        Blynk.virtualWrite(V10, batteryVolts);
        Blynk.virtualWrite(V11, rawbatvolt);
    }
}

void getBatVoltage()  {
    digitalWrite (R6_battChecker, ON); //turn on the relay so we can get a voltage reading
    if (!timer.isEnabled(batVoltMeterDelay)) {
        batVoltMeterDelay = timer.setTimeout(3000L, []() { 
            const float voltsPerBit = 3.30000000 / 4095.0;  // .00080586 Calculate volts per bit of ADC reading
            //Blynk.virtualWrite(V1, voltsPerBit);
            const float ratioV = (120000.0 + 33000.0) / 33000.0;  //Calculates to 4.636363
            //Blynk.virtualWrite(V2, ratioV);
            const float rawVolts = analogRead(A2) * voltsPerBit;  //Calculate voltage at A0 input
            //Blynk.virtualWrite(V3, rawVolts);
            batteryVolts = (rawVolts * ratioV) - 0.3; //.5
            //Blynk.virtualWrite(V4, batteryVolts);
            rawbatvolt = analogRead(batIN12V); //A2
            Blynk.virtualWrite(V10, batteryVolts);
            Blynk.virtualWrite(V11, rawbatvolt);
            digitalWrite (R6_battChecker, OFF);
            batVoltMeterDelay = timerNA;
        });
    }

/*At the typical battery voltage of 13.8V, you will get 2.976 volts at input A0 and an analogRead() value of about 3693. 
This makes rawVolts = 3693 * (3.3 / 4095) = 2.976 and batteryVolts = 2.976 * (120000 + 33000) / 33000 = 13.8.*/
    
}

void readLPUSensor()  {
    tankLevel = analogRead(LPUSig);
    calibratedlevel = tankLevel / 19.5758;
    calibratedlevel = 67 - calibratedlevel;
    if(cuLevel - tankLevel > 40 || cuLevel - tankLevel < -40) 
    {
        cuLevel = tankLevel;
        //Particle.publish("Tank Level:", String(cuLevel));
        
        //send date to thingspeak
        //ThingSpeak.setField(3, String(tankLevel));
        //ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

    }
    
}

void readGPS()  {
    // Read data from the GPS
    char c = GPS.read();
    // If a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) 
    {
        if (!GPS.parse(GPS.lastNMEA())) // This also sets the newNMEAreceived() flag to false
        return; // we can fail to parse a sentence in which case we should just wait for another
    }
} //end readGPS()
void ifmoved()  {
    if (movementDetected == true)  {
        movementDetected = false;
        writeGPS2ThingSpeak();
        //Particle.publish("Landed ifMoved");
    }
}
void checkmovement()  {
  //float latdif = currentlatitude - GPS.latitudeDegrees;
  float latdif = GPS.latitudeDegrees - currentlatitude;
  float longdif = GPS.longitudeDegrees - currentlongitude;
  //Particle.publish("LatDif", String(latdif, 6), PRIVATE);
  //Particle.publish("LongDif", String(longdif, 6), PRIVATE);
  //Particle.publish("CheckMovemntLoop");
  //Particle.publish("ABSlatDif", latdif1, PRIVATE);
  if (longdif <= -0.004 || longdif >= 0.004 || latdif <= -0.004 || latdif >= 0.004) 
  {
      //this is about 1/4mile .008 would be about 1/2mile
      //Particle.publish("2WayMovementDetected");
      movementDetected = true;
      currentlatitude = GPS.latitudeDegrees;
      currentlongitude = GPS.longitudeDegrees;
      //Particle.publish("currentLocation: ", String(currentlatitude) + ", " + String(currentlongitude));
  }
  ifmoved();
} //end checkmovement loop
void checksmallmove()  {
  //float latdif = currentlatitude - GPS.latitudeDegrees;
  float latdif = GPS.latitudeDegrees - currentlatitude;
  float longdif = GPS.longitudeDegrees - currentlongitude;
  //Particle.publish("LatDif", String(latdif, 6), PRIVATE);
  //Particle.publish("LongDif", String(longdif, 6), PRIVATE);
  //Particle.publish("CheckMovemntLoop");
  //Particle.publish("ABSlatDif", latdif1, PRIVATE);
  if (longdif <= -0.0003 || longdif >= 0.0003 || latdif <= -0.0003 || latdif >= 0.0003) //.0003 is about 100feet
  {
      //Particle.publish("2WayMovementDetected");
      movementDetected = true;
      currentlatitude = GPS.latitudeDegrees;
      currentlongitude = GPS.longitudeDegrees;
      //Particle.publish("currentLocation: ", String(currentlatitude) + ", " + String(currentlongitude));
  }
  ifmoved();
} //end checkmovement loop







