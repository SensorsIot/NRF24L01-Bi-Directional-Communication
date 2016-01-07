/*
 *
 * NRF24L01_Bi-Directional_Communication
 *
 * Testbed for NRF24L01 boards
 *
 * Bi-directional traffic
 *
 * This sketch uses the same software for both nodes
 *
 * written by Andreas Spiess. Based on Example Sketch of RF24 Library

 */

#include <SPI.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>
#include <printf.h>

//
// Pin configuration
#define NEO_PIN            6


#define ROLE_PIN 5  //  Leave open to be the 'Potentiometer' transmitter

// TCPIN_S3200
#define PIN_S0  0
#define PIN_S1  1
#define PIN_S2  3
#define PIN_S3  4
#define PIN_OUT  8


// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      8


// Colors read by TCS3200
int red = 0;
int green = 0;
int blue = 0;



#define COUNT 10 // nomber for statistics

#define DATARATE RF24_2MBPS
//  #define DATARATE RF24_1MBPS
// #define DATARATE RF24_250KBPS


// The various roles supported by this sketch
typedef enum {
  role_potentiometer,
  role_neopixel
}
roleDef;
roleDef role;


// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL};


// Initialization of  nRF24L01 radio on SPI bus plus pins 8 & 9 for Uno and Pro Mini and 48,53 for Mega
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
RF24 radio(48, 53);
#else
RF24 radio(9, 10);
#endif

// Initialization of Neopixel object
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);


//
// Role management
//
// Set up role.  This sketch uses the same software for all the nodes
// in this system.  Doing so greatly simplifies testing.  The hardware itself specifies
// which node it is.
//
// This is done through the ROLE_PIN
//
//



// The debug-friendly names of those roles
const char* role_friendly_name[] = {"Potentiometer", "Neopixel"};

// The role of the current running sketch

int _success = 0;
int _fail = 0;
unsigned long _startTime = 0;


typedef struct
{
  int A0;
  int A1;
  int A2;
}
controlDef;
controlDef controlPak;


typedef struct
{
  float red;
  float green;
  float blue;
}
displayDef;
displayDef displayPak;


void color()
{
  digitalWrite(PIN_S2, LOW);
  digitalWrite(PIN_S3, LOW);
  //count OUT, pRed, RED
  red = 5000 - pulseIn(PIN_OUT, digitalRead(PIN_OUT) == HIGH ? LOW : HIGH);
  digitalWrite(PIN_S3, HIGH);
  //count OUT, pBLUE, BLUE
  blue = 5000 - pulseIn(PIN_OUT, digitalRead(PIN_OUT) == HIGH ? LOW : HIGH);
  digitalWrite(PIN_S2, HIGH);
  //count OUT, pGreen, GREEN
  green = 5000 - pulseIn(PIN_OUT, digitalRead(PIN_OUT) == HIGH ? LOW : HIGH);
}


void setup(void)
{
  Serial.begin(115200);

  // TCPIN_S3200
  pinMode(PIN_S0, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);
  pinMode(PIN_OUT, INPUT);
  pinMode(ROLE_PIN, INPUT_PULLUP);

  digitalWrite(PIN_S0, HIGH);
  digitalWrite(PIN_S1, HIGH);

  // read the role pin to establish our role
  if ( digitalRead(ROLE_PIN) )
    role = role_potentiometer;
  else
    role = role_neopixel;

  pixels.begin(); // This initializes the NeoPixel library.

  printf_begin();
  printf("\n\rRF24/examples/pingpair/\n\r");
  printf("ROLE: %s\n\r", role_friendly_name[role]);

  //
  // Setup and configure RF radio
  radio.begin();

  if ( role == role_potentiometer )
  {
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1, pipes[1]);
  }
  else
  {
    radio.openWritingPipe(pipes [1]);
    radio.openReadingPipe(1, pipes[0]);
  }

  radio.setDataRate( DATARATE ) ;
  radio.setPALevel( RF24_PA_MAX ) ;
  radio.setChannel(0x34);
  radio.enableDynamicPayloads() ;
  radio.enableAckPayload();               // not used here
  radio.setRetries(0, 15);                // Smallest time between retries, max no. of retries
  radio.setAutoAck( true ) ;

  radio.printDetails();                   // Dump the configuration of the rf unit for debugging

  radio.powerUp();
  radio.startListening();
} 

void loop(void)
{

  // read potentiometer values and send it out. Receive telemetry data and send it to serial monitor
  if (role == role_potentiometer)
  {
    unsigned long loop_start = millis();

    // Take the time, and send it.  This will block until complete
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);

    controlPak.A0 = analogRead(A0);   // red
    controlPak.A1 = analogRead(A1);   // green
    controlPak.A2 = analogRead(A2);   // blue

    // printf("Now sen ding %i...", controlPak.steering);

    // First, stop listening so we can talk.
    radio.stopListening();
    if (!radio.write( &controlPak, sizeof(controlPak) )) {
      // Serial.println(F("failed."));
    }
    radio.startListening();
    // Serial.println(F("delivery success."));
    // printf("Time: %i ", millis() - loop_start);

    while ( !radio.available() && (millis() - loop_start) < 200) {
      // Serial.println(F("waiting."));
    }
    if (millis() - loop_start >= 200) {
      // printf("Failed. Timeout: %i...", millis() - loop_start);
      _fail++;
    } else {
      // get the telemetry data
      radio.read( &displayPak, sizeof(displayPak) );
      // Serial.print("Got response ");
      _success++;
    }

    if (_fail + _success >= COUNT)
    {
      int _ratio = 100 * _fail / (_fail + _success);
      Serial.print("Time ");
      _startTime = (millis() - _startTime);
      Serial.print(_startTime);
      Serial.print(" success ");
      Serial.print(_success);
      Serial.print(" timeout ");
      Serial.print(_fail);
      Serial.print(" Failed ");
      Serial.print(_ratio);
      Serial.print("% ");
      for (int _i = 0; _i < _ratio; _i++) Serial.print("*");
      Serial.print(" red ");
      Serial.print(displayPak.red);
      Serial.print(" green ");
      Serial.print(displayPak.green);
      Serial.print(" blue ");
      Serial.print(displayPak.blue);
      Serial.println();
      _success = 0;
      _fail = 0;
      _startTime = millis();
    }
  }

  //
  // Receive potentiometer values, display them on neopixels, read TMC3200 and send values back
  if ( role == role_neopixel)
  {
    // if there is data ready
    if ( radio.available())
    {
      radio.read(&controlPak, sizeof(controlPak));

      color();  // read TCS3200
      displayPak.red = (float)red;
      displayPak.green = (float)green;
      displayPak.blue = (float)blue;


      // Send the final one back. This way, we don't delay
      // the reply while we wait on serial i/o.
      radio.stopListening();
      radio.write( &displayPak, sizeof(displayPak) );
      // Serial.print("Sent response ");

      // Now, resume listening so we catch the next packets.
      radio.startListening();
      for (int ii = 0; ii < 8; ii++) {
        pixels.setPixelColor(ii, pixels.Color(controlPak.A0 / 4, controlPak.A1 / 4, controlPak.A2 / 4)); // potentiometer readings are from 0 to 1023, Neopixels only from 0 to 255
      }
      pixels.show(); // This sends the updated pixel color to the hardware.
    }
  }
}


