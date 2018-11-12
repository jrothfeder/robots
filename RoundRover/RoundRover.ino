#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Ultrasonic.h>

#include <FastLED.h>
#define LED_PIN     6
#define NUM_LEDS    24
#define BRIGHTNESS  64
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
#define UPDATES_PER_SECOND 100

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// And connect 2 DC motors to port M3 & M4 !
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(4);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(3);

//Name your RC here
String BROADCAST_NAME = "round robot rover";

String BROADCAST_CMD = String("AT+GAPDEVNAME=" + BROADCAST_NAME);

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// Ultrasonic sensor
Ultrasonic ultrasonic(9, 10);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

char buf[60];

byte maxSpeed = 127;

void setup(void)
{
  Serial.begin(9600);

  AFMS.begin();  // create with the default frequency 1.6KHz
  
  // turn on motors
  L_MOTOR->setSpeed(0);
  L_MOTOR->run(RELEASE);

  R_MOTOR->setSpeed(0);
  R_MOTOR->run(RELEASE);
    
  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Robot Controller Example"));
  Serial.println(F("-----------------------------------------"));

  BLEsetup();

  delay( 3000 ); // power-up safety delay
  setUpLights();  
}

int velocity = 0;

float x, y;

int L_restrict = 0;
int R_restrict = 0;

unsigned long lastAccelPacket = 0;

bool modeToggle = false;
bool followMode = false;

int cm = 0;

void loop(void)
{
  // read new packet data
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  checkButtons(); 
  readSensors();
  if(followMode) {
    handleFollowMode();
  }
}

void handleFollowMode() {
  if(shouldFollowForward()) {
    L_MOTOR->run(FORWARD);
    R_MOTOR->run(FORWARD);
  } else if(shouldFollowBackward()) {
    L_MOTOR->run(BACKWARD);
    R_MOTOR->run(BACKWARD);
  } else {
    L_MOTOR->run(RELEASE);
    R_MOTOR->run(RELEASE);
  }
}

bool shouldFollowForward() {
  return cm == 0 || cm >= 25;
}

bool shouldFollowBackward() {
  return cm <= 15;
}

void readSensors() {
  cm = ultrasonic.read();
  
  if(cm > 0 && cm <= 10) {
    fill_solid(leds, NUM_LEDS, CRGB::Red);
  } else if(cm > 10 && cm <= 20) {
    fill_solid(leds, NUM_LEDS, CRGB::Yellow);
  } else if(cm > 20 && cm <= 30) {
    fill_solid(leds, NUM_LEDS, CRGB::Blue);
  } else {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
  }
  FastLED.show();
}

void setUpLights() {
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );  
}

void fadeToBlack() {
  for( int i = 0; i < NUM_LEDS; i++) {
      leds[i].fadeToBlackBy(10);
  }
}

bool checkButtons() {
  static unsigned long lastPress = 0;
  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    
    if (pressed) {
      if(buttnum == 5){
        L_MOTOR->run(FORWARD);
        R_MOTOR->run(FORWARD);
      }
      if(buttnum == 6){
        L_MOTOR->run(BACKWARD);
        R_MOTOR->run(BACKWARD);        
      }
      if(buttnum == 7){
        L_MOTOR->run(RELEASE);
        R_MOTOR->run(FORWARD);
      }
      if(buttnum == 8){
        L_MOTOR->run(FORWARD);
        R_MOTOR->run(RELEASE);        
      }

      lastPress = millis();
      
      L_MOTOR->setSpeed(maxSpeed); 
      R_MOTOR->setSpeed(maxSpeed-5);  
    } else {
      L_MOTOR->run(RELEASE);
      R_MOTOR->run(RELEASE);
      
      if(buttnum == 1) {
        maxSpeed += 64;
      }
      if(buttnum == 2) {
        if(followMode) {
          followMode = false;  
        } else {
          followMode = true;
        }
      }
    }
    return true; 
  }

  return false;
}

void BLEsetup() {
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }

  //Convert the name change command to a char array
  BROADCAST_CMD.toCharArray(buf, 60);

  //Change the broadcast device name here!
  if(ble.sendCommandCheckOK(buf)){
    Serial.println("name changed");
  }
  delay(250);

  //reset to take effect
  if(ble.sendCommandCheckOK("ATZ")){
    Serial.println("resetting");
  }
  delay(250);

  //Confirm name change
  ble.sendCommandCheckOK("AT+GAPDEVNAME");

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("*****************"));

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("*****************"));
}
