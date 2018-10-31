/*
 * octopus-lora-octahedron-switch
 *
 * Copyright (C) 2018 Wolfgang Klenk <wolfgang.klenk@gmail.com>
 * 
 * Parts of this code are taken from the OTAA example for "The Things Network" 
 * by Thomas Telkamp and Matthijs Kooijman.
 * 
 * It uses OTAA (Over-the-air activation), where where a DevEUI, AppEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <lmic.h>
#include <hal/hal.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]= { 0x69, 0x45, 0x00, 0xF0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]= { 0x00, 0x00, 0x3B, 0x90, 0x10, 0xB6, 0x76, 0x98 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x1C, 0x4C, 0x42, 0x7B, 0xF0, 0x95, 0x09, 0xA2, 0x39, 0x51, 0x37, 0x91, 0xE3, 0x32, 0xAF, 0x62 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}


static osjob_t getOrientationJob, blinkJob;
static uint8_t lora_payload[1] = { 0x00 };

// Check orientation periodically every 2 seconds.
const unsigned GET_ORIENTATION_INTERVAL = 2;
static boolean is_blinking = true;

const unsigned int NUMBER_OF_SAMPLES = 5;
int last_octant_values[NUMBER_OF_SAMPLES];
int current_sample_index = 0;
int last_octant_value_sent = -1;

// Pin mapping
const lmic_pinmap lmic_pins = { 
   .nss = 2, 
   .rxtx = LMIC_UNUSED_PIN, 
   .rst = LMIC_UNUSED_PIN, 
   .dio = {15, 15, LMIC_UNUSED_PIN}, 
};

Adafruit_BNO055 bno055 = Adafruit_BNO055(55);

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));

            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            // Comment out to allow Adaptive Data Rate (ADR) tests
            // https://www.thethingsnetwork.org/wiki/LoRaWAN/ADR
            //LMIC_setLinkCheckMode(0);

            // Turn off blinking
            is_blinking = false;
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next orientation check
            os_setTimedCallback(&getOrientationJob, os_getTime()+sec2osticks(GET_ORIENTATION_INTERVAL), get_octahedron_orientation);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_blink(osjob_t* j){
  static boolean led_on;

  if (led_on) {
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  led_on = !led_on;


  // Schedule next blinking
  if (is_blinking) {
    os_setTimedCallback(&blinkJob, os_getTime()+sec2osticks(1), do_blink);
  } else {
    digitalWrite(LED_BUILTIN, HIGH); // HIGH = LED off
  }
}

void get_octahedron_orientation(osjob_t* j){
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
  } else {

    // Get orientation of octahedron and send if it has changed.
    int octant = get_octant_number();

    // Put octant value in sample ring buffer
    last_octant_values[current_sample_index++] = octant;
    current_sample_index %= NUMBER_OF_SAMPLES;

    boolean all_samples_match_current_octant = true;
    for (int i=0 ; i<NUMBER_OF_SAMPLES ; i++) {
      if (last_octant_values[i] != octant) {
        all_samples_match_current_octant = false;
      }
      
      Serial.print(last_octant_values[i]);
      Serial.print(" ");
    }
    Serial.println();

    if (all_samples_match_current_octant && (octant != last_octant_value_sent)) {
      last_octant_value_sent = octant;
      lora_payload[0] = octant;
      
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1, lora_payload, 1, 0);
      Serial.println(F("Packet queued"));  
    } else {
      // Either the orientation was not stable the last recent NUMBER_OF_SAMPLES, or the value doesn't differ 
      // from the value already sent via LoRaWAN before.
        
      // Schedule next orientation check
      os_setTimedCallback(&getOrientationJob, os_getTime()+sec2osticks(GET_ORIENTATION_INTERVAL), get_octahedron_orientation);
    }
  }
}

int64_t get_timestamp_us()
{
    return (int64_t) millis() * 1000;
}

void setup() {
  // Initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

  /*
  // Will block until serial port is connected.
  while (!Serial) {
    // Wait for serial port to connect. Needed for native USB
    delay(100);
  }
  */

  Serial.print("Initializing BNO055: ");
  if (bno055.begin()) {
    Serial.println("OK");
  } else {
    Serial.println("Not found");
  }
    
  Serial.println(F("Starting"));

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Reset the array of octant samples
  for (int i=0 ; i<NUMBER_OF_SAMPLES ; i++) {
    last_octant_values[i] = -1;
  }
    
  // Start job 
  do_blink(&blinkJob);
  get_octahedron_orientation(&getOrientationJob);    
}

void loop() {
    os_runloop_once();
}

int get_octant_number() {
  float x;
  float y;
  
  // Read acceleration values for x, y and z
  imu::Vector<3> bnoAccel = bno055.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // Unfortunately, the Octopus board is positioned in parallel to the edges of the
  // ground rectangle of the pyramid. It would possibly have been a better design
  // decision to position it on a line from corner to corner. This would have maken it
  // easier to evaluate the octant number.

  // However, we can apply a little bit mathematics on the x and y coordinates
  // to rotate the gravity vector by 45 degrees.
  // x = bnoAccel.x() * cos(45°) - bnoAccel.y() * sin(45°)
  // y = bnoAccel.x() * sin(45°) + bnoAccel.y() * cos(45°)

  x = bnoAccel.x() * 0.71 - bnoAccel.y() * 0.71;
  y = bnoAccel.x() * 0.71 + bnoAccel.y() * 0.71;

  Serial.print("Acceleration (m/s^2) x=");
  Serial.print(x);
  Serial.print(", y=");
  Serial.print(y);
  Serial.print(", z=");
  Serial.print(bnoAccel.z());

  // Evaluate octant number.
  // See https://en.wikipedia.org/wiki/Octant_(solid_geometry)
  int octant = -1;

  if (bnoAccel.z() >= 0) {
    // z >= 0
    if ( y >= 0) {
      if (x >= 0) {
        octant = 1;
      } else {
        octant = 2;
      }
    }
    else {
      // y < 0
      if (x >= 0) {
        octant = 4;
      } else {
        octant = 3;
      }
    }
  } else {
    // z < 0
    if ( y >= 0) {
      // y >= 0
      if (x >= 0) {
        octant = 5;
      } else {
        octant = 6;
      }
    }
    else {
      // y < 0
      if (x >= 0) {
        octant = 8;
      } else {
        octant = 7;
      }
    }
  }

  Serial.print(", octant=");
  Serial.println(octant);
  
  return octant;
}


