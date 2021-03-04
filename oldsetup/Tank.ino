/**
 * Tank node sketch for the three node system.
 */

/***************************************************
 * 
 * These values are shared with other nodes
 * 
 * *************************************************/

#define TRACTOR_NODE 1
#define TANK_NODE 2
#define SEMI_NODE 3

// Codes to tell receiving end the type of data to expect
#define COMMAND 100  //         [Opcode: 1][Pin: 1][On/Off: 1]
#define TANK_LEVEL 101 //       [Opcode: 1][Payload: 4]
#define SEQUENCE 102 //         [Opcode: 1][Sequence: 1]

// Semi sequences codes
#define SEMI_SEQUENCE_1 1
#define SEMI_SEQUENCE_2 2
#define PUMP_SHUTDOWN_SEQUENCE 3

// Tank sequence codes
#define FUNCTION_1 4
#define FUNCTION_2 5
#define FUNCTION_3 6
#define FUNCTION_4 7

// Semi relays
#define RELAY1 32
#define RELAY2 33

// Tank valves
#define INLET_VALVE 30
#define REVERSE_VALVE 28 
#define RETURN_VALVE 36
#define CLEANOUT_1 34
#define CLEANOUT_2 32
#define BOTTOM_CLEANOUT_1 23
#define BOTTOM_CLEANOUT_2 25
#define BOTTOM_CLEANOUT_3 27
#define TANK_LID 29
#define PLACEHOLDER1 31

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>

#define TXPOWER 5 // TX power in dbm. (Range of 5 - 23)

#define MSG_MAX_LEN 5 //        [Opcode: 1][Payload: 4]

#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95;

uint8_t buf[MSG_MAX_LEN];
uint8_t buf_len = sizeof(buf);

/***************************************************
 * 
 * End of shared values
 * 
 * *************************************************/

/***************************************************
 * 
 * These values are unique to this node
 * 
 * *************************************************/

#include <ModbusMaster.h>

unsigned long lastSentTime = 0;

#define NODEID TANK_NODE

RHReliableDatagram manager(rf95, NODEID);

#define DEBUG_ENABLED 1 // Set to 0 to turn off print statements.
#define SEND_WAIT_TIMEOUT 500 // In milliseconds
#define UPDATE_MIN_TIME 10000 // In milliseconds. 300000 = 5 minutes

// Modbus values. DE and RE can be any available pin
#define MAX485_DE 48
#define MAX485_RE_NEG 49

// Serial1 TX/RX pins are 18 19
#define MAX485_TX 18
#define MAX485_RX 19

ModbusMaster node;

#define TANK_FLOAT_SENSOR 32

bool floatSensorState = HIGH;
bool sendShutdown = false;
unsigned long lastShutdownSent = 0;
uint8_t last_shutdown_mid;

uint8_t valves[] = {
  INLET_VALVE,
  REVERSE_VALVE,
  RETURN_VALVE,
  CLEANOUT_1,
  CLEANOUT_2,
  BOTTOM_CLEANOUT_1,
  BOTTOM_CLEANOUT_2,
  BOTTOM_CLEANOUT_3,
  TANK_LID,
  PLACEHOLDER1
};

// Delays for bottom cleanout function
#define BOTTOM_CLEANOUT_DELAY_1 3000
#define BOTTOM_CLEANOUT_DELAY_2 5000
  
uint8_t numValves = sizeof(valves);

typedef union {
  uint8_t bytes[4];
  uint32_t num;
} UINT32_ARRAY;

/***************************************************
 * 
 * End of unique values
 * 
 * *************************************************/

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.print(F("Node "));Serial.print(NODEID);Serial.println(F(" intializing..."));
  
  radio_init();

  modbus_init();

  pinMode(TANK_LID, INPUT_PULLUP);
  pinMode(TANK_FLOAT_SENSOR, INPUT_PULLUP);
  
  for (byte i = 0; i < numValves; i++) {
    pinMode(valves[i], OUTPUT);
    digitalWrite(valves[i], HIGH);
  }

  Serial.print(F("Main program starting..."));
}

void loop() {

  checkIncoming(); // Waits for incoming messages
  if (millis() - lastSentTime > UPDATE_MIN_TIME) {
    int32_t tankLevel = getTankLevel();
    if ( tankLevel >= 0 ) {
      sendTankLevel(TRACTOR_NODE, (uint32_t)tankLevel);
      lastSentTime = millis();
    }
  }
  checkFloat();
}

void checkFloat() {
  if (buttonPressed( TANK_FLOAT_SENSOR, &floatSensorState )) {

    if (DEBUG_ENABLED) {
      Serial.println(F("Tank float sensor detected. Shutting off pump..."));
    }
    // Keep sending until successful
    while (!sendSequence( SEMI_NODE, PUMP_SHUTDOWN_SEQUENCE ));
  }
}

int32_t getTankLevel() {
  // Read 11 registers starting at FC(0x04) Register(30299) Address(298) HEX(12A)
  uint8_t result = node.readInputRegisters(298, 11);
  
  if ( result == node.ku8MBSuccess ) {

    // Print extra if debug is enabled
    if ( DEBUG_ENABLED ) {
      uint16_t data;
      Serial.print("ModelType: ");
      Serial.println(node.getResponseBuffer(0));
      
      Serial.print("Raw Distance/Level Reading (in mm, unsigned): ");
      Serial.println(node.getResponseBuffer(1)); 
      
      Serial.print("Temperature Reading (in C, signed): ");
      Serial.println(node.getResponseBuffer(3));

      Serial.print("Calculated (raw): ");
      Serial.println(node.getResponseBuffer(4) * 65536) + (node.getResponseBuffer(5));

      Serial.print("Version: ");
      data = node.getResponseBuffer(8);
      Serial.println(highByte(data));

      Serial.print("Signal Strength: ");
      Serial.println(lowByte(data));

      Serial.print("Trip 1 Alarm: ");
      data = node.getResponseBuffer(10);
      Serial.println(highByte(data));

      Serial.print("Trip 1 Status: ");
      Serial.println(lowByte(data));

      Serial.print("Trip 2 Alarm: ");
      data = node.getResponseBuffer(11);
      Serial.println(highByte(data));

      Serial.print("Trip 2 Status: ");
      Serial.println(lowByte(data));
    }

    return (node.getResponseBuffer(4) * 65536) + (node.getResponseBuffer(5));

  } else {
    return -1;
  }

}

void checkIncoming() {
  
  if (manager.available()) {
    uint8_t from;
    uint8_t to;
    uint8_t id;
    if (manager.recvfromAck(buf, &buf_len, &from, &to, &id)) {

      uint8_t opcode = buf[0];

      if (opcode == COMMAND) {

        uint8_t pin = buf[1];
        bool state = buf[2];

        doCommand( pin, state );

        if (DEBUG_ENABLED) {
          Serial.println(F("Received COMMAND packet"));
        }

      } else if (opcode == SEQUENCE) {

        uint8_t sequence = buf[1];
        doSequence( sequence );

        if (DEBUG_ENABLED) {
          Serial.println(F("Received SEQUENCE packet"));
        }

      } else if (DEBUG_ENABLED) {
        Serial.print(F("Received UNKNOWN packet: "));Serial.println(opcode);
      }

      if (DEBUG_ENABLED) {
        Serial.print(F("From NodeID: "));Serial.println(from);
        Serial.print(F("To NodeID: "));Serial.println(to);
        Serial.print(F("With MessageID: "));Serial.println(id);
      }
    }
  }
}

void doCommand( uint8_t pin, bool state ) {

  if (DEBUG_ENABLED) {
    Serial.print(F("Switching pin "));
    Serial.print(pin);
    Serial.println(state ? F(" Off"):F(" On"));
  }
  
  digitalWrite(pin, state);
}

void doSequence( uint8_t sequence ) {

  if (DEBUG_ENABLED) {
    Serial.print(F("Running Sequence "));Serial.println(sequence);
  }

  switch( sequence ) {
    case FUNCTION_1:
      bottomCleanOut();     
      break;
    case FUNCTION_2:
      break;
    case FUNCTION_3:
      break;
    case FUNCTION_4:
      break;
    default:
      Serial.println(F("Unknown Sequence"));
  }
}

bool sendSequence(uint8_t to, uint8_t sequence) {
  
  // Build packet
  buf[0] = SEQUENCE;
  buf[1] = sequence;

  return sendDataPacket(buf, buf_len, to);
}

bool sendCommand(uint8_t to, uint8_t pin, bool state) {

  // Build packet
  buf[0] = COMMAND;
  buf[1] = pin;
  buf[2] = state;

  return sendDataPacket(buf, buf_len, to);
}

bool sendTankLevel(uint8_t to, uint32_t data) {
  // Build packet
  buf[0] = TANK_LEVEL;

  UINT32_ARRAY dataArr;

  dataArr.num = data;

  buf[1] = dataArr.bytes[3];
  buf[2] = dataArr.bytes[2];
  buf[3] = dataArr.bytes[1];
  buf[4] = dataArr.bytes[0];

  return sendDataPacket(buf, buf_len, to);
}

bool sendDataPacket(uint8_t *packet, uint8_t packetSize, uint8_t to) {
  
  if (manager.sendtoWait(packet, packetSize, to)) {

    if (DEBUG_ENABLED) {
      Serial.println(F("Sent"));
    }

    return true;
    
  } else {
    Serial.println(F("Failed to send"));
    return false;
  }
}

void bottomCleanOut() {
  digitalWrite(BOTTOM_CLEANOUT_1, LOW);
  delay(BOTTOM_CLEANOUT_DELAY_1);
  digitalWrite(BOTTOM_CLEANOUT_2, LOW);
  digitalWrite(BOTTOM_CLEANOUT_1, HIGH);
  delay(BOTTOM_CLEANOUT_DELAY_2);
  digitalWrite(BOTTOM_CLEANOUT_2, HIGH);  
}

void radio_init() {

  if (!manager.init()) {
    Serial.println(F("LoRa radio init failed"));
    while (1) {
      delay(100);
    }
  }
  Serial.println(F("LoRa radio init OK!"));

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println(F("setFrequency failed"));
    while (1) {
      delay(100);
    }
  }
  Serial.print(F("Set Freq to: ")); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(TXPOWER, false);

  manager.setTimeout(SEND_WAIT_TIMEOUT);
}

// Returns true if the state of the button changes from HIGH to LOW. False in all other cases.
bool buttonPressed (int pin, bool *val) {
  if( digitalRead(pin) == LOW ) {
    if ( *val == HIGH ) {
      *val = LOW;
      return true;
    }
  } else {
    *val = HIGH;
  }
  return false;
}

void modbus_init() {
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus RTU Ultrasonic communication runs at 9600 baud.
  // Ensure the pins mach on MAX485_TX and MAX485_RX with the Serial object used here.
  Serial1.begin(9600);
 
  // Modbus slave ID 1
  node.begin(1, Serial1);
  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);  
}

void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}
