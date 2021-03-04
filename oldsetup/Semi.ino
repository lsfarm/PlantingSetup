/**
 * Semi trailer node sketch for the three node system.
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
 * These values are unique to this node
 * 
 * *************************************************/
 
#define NODEID SEMI_NODE

RHReliableDatagram manager(rf95, NODEID);

#define PUMP_SHUTDOWN_PIN 24

#define DEBUG_ENABLED 1 // Set to 0 to turn off print statements.
#define SEND_WAIT_TIMEOUT 500 // In milliseconds
#define UPDATE_MIN_TIME 3000 // In milliseconds. 300000 = 5 minutes

unsigned long lastSentTime = 0;

/***************************************************
 * 
 * End of unique values
 * 
 * *************************************************/

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.print(F("Node "));Serial.print(NODEID);Serial.println(F(" intializing..."));
  
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);

  digitalWrite(RELAY1, HIGH);
  digitalWrite(RELAY2, HIGH);

  radio_init();
  
  Serial.print(F("Main program starting..."));
}

void loop() {

  checkIncoming(); // Waits for incoming messages

}

bool sentToMe (uint8_t *packet) {
  return packet[1] == NODEID;
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
    case SEMI_SEQUENCE_1:
    case SEMI_SEQUENCE_2:
      break;
    case PUMP_SHUTDOWN_SEQUENCE:
      shutDownPump();      
      break;
    default:
      Serial.println(F("Unknown Sequence"));
  }
}

bool sendCommand(uint8_t to, uint8_t pin, bool state) {

  // Build packet
  buf[0] = COMMAND;
  buf[1] = pin;
  buf[2] = state;

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

void shutDownPump() {
  digitalWrite( PUMP_SHUTDOWN_PIN, HIGH );
  delay(5000);
  digitalWrite( PUMP_SHUTDOWN_PIN, LOW );
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
