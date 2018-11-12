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

int palletteNum = 0;
CRGBPalette16 currentPalette;
TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

bool partyMode = false;
bool autonomousMode = false;
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// And connect 2 DC motors to port M3 & M4 !
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(4);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(3);

//not used, testing acceleration
// int accelTime = 200;

//Name your RC here
String BROADCAST_NAME = "red robot rover";

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

void loop(void)
{
  // read new packet data
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
//  maintainLights(len);
  maintainMotor(len); 
  readSensors();
}

void readSensors() {
  int cm = ultrasonic.read();
  
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
  currentPalette = RainbowColors_p;
  currentBlending = LINEARBLEND;
}

void maintainLights(uint8_t len) {
  if( partyMode ) {
    
    static uint8_t startIndex = 0;
    startIndex = startIndex + 1; /* motion speed */
    
    FillLEDsFromPaletteColors( startIndex);
    
    FastLED.show();
    FastLED.delay(1000 / UPDATES_PER_SECOND);
  } else {
    fadeToBlack();
    FastLED.show();
  }
}

void maintainMotor(uint8_t len) {
  buttonMode();
}

void FillLEDsFromPaletteColors( uint8_t colorIndex)
{
  uint8_t brightness = 255;
    
  for( int i = 0; i < NUM_LEDS; i++) {
      leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
      colorIndex += 3;
  }
}

void fadeToBlack() {
  for( int i = 0; i < NUM_LEDS; i++) {
      leds[i].fadeToBlackBy(10);
  }
}

void changePallette()
{
  if( palletteNum ==  0)  { currentPalette = RainbowColors_p;         currentBlending = LINEARBLEND; }
  if( palletteNum ==  1)  { currentPalette = RainbowStripeColors_p;   currentBlending = NOBLEND;  }
  if( palletteNum ==  2)  { currentPalette = RainbowStripeColors_p;   currentBlending = LINEARBLEND; }
  if( palletteNum ==  3)  { SetupPurpleAndGreenPalette();             currentBlending = LINEARBLEND; }
  if( palletteNum ==  4)  { SetupTotallyRandomPalette();              currentBlending = LINEARBLEND; }
  if( palletteNum ==  5)  { SetupBlackAndWhiteStripedPalette();       currentBlending = NOBLEND; }
  if( palletteNum ==  6)  { SetupBlackAndWhiteStripedPalette();       currentBlending = LINEARBLEND; }
  if( palletteNum ==  7)  { currentPalette = CloudColors_p;           currentBlending = LINEARBLEND; }
  if( palletteNum ==  8)  { currentPalette = PartyColors_p;           currentBlending = LINEARBLEND; }
  if( palletteNum ==  9)  { currentPalette = myRedWhiteBluePalette_p; currentBlending = NOBLEND;  }
  if( palletteNum == 10)  { currentPalette = myRedWhiteBluePalette_p; currentBlending = LINEARBLEND; }
}

void nextPallette() {
  if(++palletteNum > 10) {
    palletteNum = 0;
  }
  changePallette();
}

void previousPallette() {
  if(--palletteNum < 0) {
    palletteNum = 10;
  }
  changePallette();
}
void SetupTotallyRandomPalette() {
    for( int i = 0; i < 16; i++) {
        currentPalette[i] = CHSV( random8(), 255, random8());
    }
}

void SetupBlackAndWhiteStripedPalette() {
    // 'black out' all 16 palette entries...
    fill_solid( currentPalette, 16, CRGB::Black);
    // and set every fourth one to white.
    currentPalette[0] = CRGB::White;
    currentPalette[4] = CRGB::White;
    currentPalette[8] = CRGB::White;
    currentPalette[12] = CRGB::White;
    
}

void SetupPurpleAndGreenPalette() {
    CRGB purple = CHSV( HUE_PURPLE, 255, 255);
    CRGB green  = CHSV( HUE_GREEN, 255, 255);
    CRGB black  = CRGB::Black;
    
    currentPalette = CRGBPalette16(
                                   green,  green,  black,  black,
                                   purple, purple, black,  black,
                                   green,  green,  black,  black,
                                   purple, purple, black,  black );
}

const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM = {
    CRGB::Red,
    CRGB::Gray, // 'white' is too bright compared to red and blue
    CRGB::Blue,
    CRGB::Black,
    
    CRGB::Red,
    CRGB::Gray,
    CRGB::Blue,
    CRGB::Black,
    
    CRGB::Red,
    CRGB::Red,
    CRGB::Gray,
    CRGB::Gray,
    CRGB::Blue,
    CRGB::Blue,
    CRGB::Black,
    CRGB::Black
};

bool isMoving = false;

bool buttonMode() {
  static unsigned long lastPress = 0;
  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    
    if (pressed) {
      isMoving = true;
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
      
      L_MOTOR->setSpeed(150); 
      R_MOTOR->setSpeed(150);  
    } 

    else {
      isMoving = false;
      L_MOTOR->run(RELEASE);
      R_MOTOR->run(RELEASE);

      if(buttnum == 1) {
        if(partyMode == true) {
          partyMode = false;
        } else {
          partyMode = true;
        }
      }

      if(buttnum == 2) {
        if(autonomousMode == true) {
          autonomousMode = false;
        } else {
          autonomousMode = true;
        }
      }

      if(buttnum == 3) {
        previousPallette();
      }

      if(buttnum == 4) {
        nextPallette();
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

//Logarithmic mapping function from http://playground.arduino.cc/Main/Fscale
float fscale( float inputValue,  float originalMin, float originalMax, float newBegin, float newEnd, float curve){

  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;


  // condition curve parameter
  // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output 
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  /*
   Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution  
   Serial.println(); 
   */

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin){ 
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd; 
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

  /*
  Serial.print(OriginalRange, DEC);  
   Serial.print("   ");  
   Serial.print(NewRange, DEC);  
   Serial.print("   ");  
   Serial.println(zeroRefCurVal, DEC);  
   Serial.println();  
   */

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine 
  if (originalMin > originalMax ) {
    return 0;
  }

  if (invFlag == 0){
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  }
  else     // invert the ranges
  {   
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange); 
  }

  return rangedValue;
}
