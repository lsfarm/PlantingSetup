/**
 * Tractor node sketch for the three node system.
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
 
#define NODEID TRACTOR_NODE

RHReliableDatagram manager(rf95, NODEID);

// Pins for potentiometer max tank level control
#define FINE_MAX_TANK_LEVEL_POT A8 // 0-100 gal in 5 gal intervals
#define COARSE_MAX_TANK_LEVEL_POT A9 // 0-1600 gal in 100 gal intervals

#define DEBUG_ENABLED 1 // Set to 0 to turn off print statements.
#define SEND_WAIT_TIMEOUT 100 // In milliseconds
#define SEND_RETRIES 5 // 
#define SEND_WAIT 500

// Stores the latest received tank level values to calculate a running average
#define NUM_TANK_READINGS 10
uint32_t tankReadings[NUM_TANK_READINGS] = {0};
uint8_t tankCount = 0;

// Momentary push buttons
#define BUTTON_1 18
#define BUTTON_2 19
#define BUTTON_3 20
#define BUTTON_4 21

// LEDs
#define INLET_VALVE_LED 22
#define REVERSE_VALVE_LED 23 
#define RETURN_VALVE_LED 24
#define CLEANOUT_1_LED 25
#define CLEANOUT_2_LED 26
#define BOTTOM_CLEANOUT_1_LED 27
#define BOTTOM_CLEANOUT_2_LED 28
#define BOTTOM_CLEANOUT_3_LED 29
#define TANK_LID_LED 30
#define PLACEHOLDER1_LED 31

// Switches to Tank valves
#define INLET_VALVE_SWT 41
#define REVERSE_VALVE_SWT 40 
#define RETURN_VALVE_SWT 38
#define CLEANOUT_1_SWT 39
#define CLEANOUT_2_SWT 37
#define BOTTOM_CLEANOUT_1_SWT 36
#define BOTTOM_CLEANOUT_2_SWT 35
#define BOTTOM_CLEANOUT_3_SWT 34
#define TANK_LID_SWT 32
#define PLACEHOLDER1_SWT 33

// Switches to Semi relays
#define RELAY1_SWT 42
#define RELAY2_SWT 43

// Structure to map local pins to remote pins or sequences
struct PinMap {
  // Button input pin
  uint8_t pin;

  // Previous recorded button state. Default state is LOW
  bool state;

  // Valve/Sequence/Relay output pin/number
  uint8_t output;

  // LED pin to light up
  uint8_t led;

  // Switch to true after send and false when confirmation returns
  bool sending;

  // Last time this was sent
  unsigned long lastSent;

  // // Message ID for acknowledgement
  // uint8_t mid;

// 10 Switch pins that correspond to 10 valves on the tank
} tankSwitches[] = {
  {
    pin: INLET_VALVE_SWT,
    state: LOW,
    output: INLET_VALVE,
    led: INLET_VALVE_LED,
    sending: false,
    lastSent: 0L
    // mid: 0
  },
  {
    pin: REVERSE_VALVE_SWT,
    state: LOW,
    output: REVERSE_VALVE,
    led: REVERSE_VALVE_LED,
    sending: false,
    lastSent: 0L
    // mid: 0
  },
  {
    pin: RETURN_VALVE_SWT,
    state: LOW,
    output: RETURN_VALVE,
    led: RETURN_VALVE_LED,
    sending: false,
    lastSent: 0L
    // mid: 0
  },
  {
    pin: CLEANOUT_1_SWT,
    state: LOW,
    output: CLEANOUT_1,
    led: CLEANOUT_1_LED,
    sending: false,
    lastSent: 0L
    // mid: 0
  },
  {
    pin: CLEANOUT_2_SWT,
    state: LOW,
    output: CLEANOUT_2,
    led: CLEANOUT_2_LED,
    sending: false,
    lastSent: 0L
    // mid: 0
  },
  {
    pin: BOTTOM_CLEANOUT_1_SWT,
    state: LOW,
    output: BOTTOM_CLEANOUT_1,
    led: BOTTOM_CLEANOUT_1_LED,
    sending: false,
    lastSent: 0L
    // mid: 0
  },
  {
    pin: BOTTOM_CLEANOUT_2_SWT,
    state: LOW,
    output: BOTTOM_CLEANOUT_2,
    led: BOTTOM_CLEANOUT_2_LED,
    sending: false,
    lastSent: 0L
    // mid: 0
  },
  {
    pin: BOTTOM_CLEANOUT_3_SWT,
    state: LOW,
    output: BOTTOM_CLEANOUT_3,
    led: BOTTOM_CLEANOUT_3_LED,
    sending: false,
    lastSent: 0L
    // mid: 0
  },
  {
    pin: TANK_LID_SWT,
    state: LOW,
    output: TANK_LID,
    led: TANK_LID_LED,
    sending: false,
    lastSent: 0L
    // mid: 0
  },
  {
    pin: PLACEHOLDER1_SWT,
    state: LOW,
    output: PLACEHOLDER1,
    led: PLACEHOLDER1_LED,
    sending: false,
    lastSent: 0L
    // mid: 0
  }
};

// Semi switch relays
PinMap semiSwitches[] = {
  {
    pin: RELAY1_SWT,
    state: LOW,
    output: RELAY1,
    led: LED_BUILTIN,
    sending: false,
    lastSent: 0L
    // mid: 0
  },
  {
    pin: RELAY2_SWT,
    state: LOW,
    output: RELAY2,
    led: LED_BUILTIN,
    sending: false,
    lastSent: 0L
    // mid: 0
  }
};

// Inititate a sequence of actions on tank when pressed
PinMap tankBtns[] = {
  {
    pin: BUTTON_1,
    state: HIGH,
    output: FUNCTION_1,
    led: LED_BUILTIN,
    sending: false,
    lastSent: 0L
    // mid: 0
  },
  {
    pin: BUTTON_2,
    state: HIGH,
    output: FUNCTION_2,
    led: LED_BUILTIN,
    sending: false,
    lastSent: 0L
    // mid: 0
  },
  {
    pin: BUTTON_3,
    state: HIGH,
    output: FUNCTION_3,
    led: LED_BUILTIN,
    sending: false,
    lastSent: 0L
    // mid: 0
  },
  {
    pin: BUTTON_4,
    state: HIGH,
    output: FUNCTION_4,
    led: LED_BUILTIN,
    sending: false,
    lastSent: 0L
    // mid: 0
  }
};

uint8_t numTankSwitches = sizeof(tankSwitches) / sizeof(PinMap);
uint8_t numSemiSwitches = sizeof(semiSwitches) / sizeof(PinMap);
uint8_t numTankBtns = sizeof(tankBtns) / sizeof(PinMap);

bool sendShutdown = false;
bool shutdownDebounce = true;
unsigned long lastShutdownSent = 0;

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

  for (byte i = 0; i < numTankSwitches; i++) {
    pinMode(tankSwitches[i].pin, INPUT_PULLUP);
    pinMode(tankSwitches[i].led, OUTPUT);
    tankSwitches[i].state = digitalRead(tankSwitches[i].pin);
    tankSwitches[i].sending = true;
    
    digitalWrite(tankSwitches[i].led, !tankSwitches[i].state);
    
  }

  for (byte i = 0; i < numSemiSwitches; i++) {
    pinMode(semiSwitches[i].pin, INPUT_PULLUP);
    semiSwitches[i].state = digitalRead(semiSwitches[i].pin);
    semiSwitches[i].sending = true;
    
  }

  for (byte i = 0; i < numTankBtns; i++) {
    pinMode(tankBtns[i].pin, INPUT_PULLUP);
  }
    
  Serial.print(F("Main program starting..."));
}

void loop() {
  
  checkButtons();
  checkIncoming(); // Checks for incoming messages
  sendPackets();

}

// Sends one packet
void sendPackets() {

  if ( sendShutdown && millis() - lastShutdownSent > SEND_WAIT ) {
    sendSequence( SEMI_NODE, PUMP_SHUTDOWN_SEQUENCE );
    lastShutdownSent = millis();
    return;
  }
  
  for (byte i = 0; i < numTankSwitches; i++) {
    if ( tankSwitches[i].sending && millis() - tankSwitches[i].lastSent > SEND_WAIT ) {
      if (sendCommand( TANK_NODE, tankSwitches[i].output, tankSwitches[i].state )) {
        tankSwitches[i].sending = false;
      }
      tankSwitches[i].lastSent = millis();
      return;
    }
  }

  for (byte i = 0; i < numSemiSwitches; i++) {
    if ( semiSwitches[i].sending && semiSwitches[i].lastSent > SEND_WAIT ) {
      if (sendCommand( SEMI_NODE, semiSwitches[i].output, semiSwitches[i].state )) {
        semiSwitches[i].sending = false;
      }
      semiSwitches[i].lastSent = millis();
      return;
    }
  }

  for (byte i = 0; i < numTankBtns; i++) {
    if ( tankBtns[i].sending && millis() - tankBtns[i].lastSent > SEND_WAIT ) {
      if (sendSequence( TANK_NODE, tankBtns[i].output )) {
        tankBtns[i].sending = false;
      }
      tankBtns[i].lastSent = millis();
      return;
    }
  }

}

void checkButtons() {

  for (byte i = 0; i < numTankBtns; i++) {
    if ( buttonPressed( tankBtns[i].pin, &tankBtns[i].state )) {
      tankBtns[i].sending = true;
    }
  }
  
  for (byte i = 0; i < numTankSwitches; i++) {
    if ( switched( tankSwitches[i].pin, &tankSwitches[i].state )) {
      tankSwitches[i].sending = true;
      digitalWrite(tankSwitches[i].led, !tankSwitches[i].state);
    }
  }

  for (byte i = 0; i < numSemiSwitches; i++) {
    if ( switched( semiSwitches[i].pin, &semiSwitches[i].state )) {
      semiSwitches[i].sending = true;
    }
  }
}

void checkIncoming() {

  if (manager.available()) {
    uint8_t from;
    uint8_t to;
    uint8_t id;
    if (manager.recvfromAck(buf, &buf_len, &from, &to, &id)) {

      uint8_t opcode = buf[0];

      if (opcode == TANK_LEVEL) {
        uint32_t tankLevel = getFertilizerData(buf);
        if ( DEBUG_ENABLED ) {
          Serial.print(F("Tank level received: "));Serial.print(tankLevel);
        }
        if ( inFillMode() && tankLevel >= getMaxTankLevel() ) {
          if ( shutdownDebounce ) {
            sendShutdown = true;
            shutdownDebounce = false;
          }
        } else {
          shutdownDebounce = true;
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

bool inFillMode() {
  return digitalRead( TANK_LID_SWT ) == HIGH;
}

uint16_t getMaxTankLevel() {
  static uint16_t coarse;
  static uint16_t fine;
  static uint16_t maxLevel;
  if (coarse != analogRead( COARSE_MAX_TANK_LEVEL_POT ) || fine != analogRead( FINE_MAX_TANK_LEVEL_POT )) {
    coarse = analogRead( COARSE_MAX_TANK_LEVEL_POT );
    fine = analogRead( FINE_MAX_TANK_LEVEL_POT );
    
    maxLevel = round100(map( coarse, 0, 1023, 0, 1600 )) + round5(map( fine, 0, 1023, 0, 100));

    if ( DEBUG_ENABLED ) {
      Serial.print(F("Max level set to "));Serial.println(maxLevel);
    }
  }
  return maxLevel;
}

uint32_t getFertilizerData(uint8_t *buf) {

  uint32_t tankLevel = buf[1] * 16777216 + buf[2] * 65536 + buf[3] * 256 + buf[4];
  
  tankReadings[tankCount++ % NUM_TANK_READINGS] = tankLevel;

  // Return instant level if in fill mode. Otherwise calculate running average.
  if ( inFillMode() ) {
    return tankLevel;
  }

  uint64_t total = 0;

  for (byte i = 0; i < NUM_TANK_READINGS; i++) {
    total += tankReadings[i];
  }

  return total / NUM_TANK_READINGS;

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

bool sendDataPacket(uint8_t *packet, uint8_t packetSize, uint8_t to) {
  
  if (manager.sendtoWait(packet, packetSize, to)) {

    if (DEBUG_ENABLED) {
      Serial.println(F("Sent"));
    }

    return true;
    
  } else {
    if (DEBUG_ENABLED) {
      Serial.println(F("Failed to send"));
      Serial.print(F("To NodeID: "));Serial.println(to);
      Serial.print(F("Packet contents: "));
      for (byte i=0; i < packetSize; i++) {
        Serial.print(packet[i]);
        Serial.print(' ');
      }
      Serial.println();
    }
    return false;
  }
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
  manager.setRetries(SEND_RETRIES);
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

bool switched (int pin, bool *val) {
  if ( digitalRead(pin) != *val ) {
    *val = !*val;
    return true;
  } else {
    return false;
  }
}

int8_t round5delta[5] = {0, -1, -2, 2, 1};
uint16_t round5(uint16_t in) {
  return in + round5delta[in%5];
}

uint16_t round100(uint16_t in) {
  return 100 * round(in / 100.0);
}
