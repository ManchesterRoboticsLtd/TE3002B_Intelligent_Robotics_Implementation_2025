// Traffic Light Basic Code
/* Kat Nelms, Manchester Robotics, Feb 19 2022
// adapted from Arduino Neopixel Library "RGBWstrandtest"
// "Nlight_oneway" = choose between one or two TLBs, only one-way traffic
// one MCU board (MCUB) and up to two traffic lights (TLB)
// next level of complexity up from "1int_1way" script
// should initialize TLB1 to green and, if TLB2, TLB2 initialized to red
// then change green > yellow > red > yellow > green according to hard coded timers
*/

/* SETUP */
#define LED_PIN 52 // Define the data out pin on the ESP32
#define greenTimer  4  //how long each color lasts, units are sec
#define redTimer    5
#define yellowTimer 1
/* END SETUP */


//lights stuff
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

//Screen stuff
#include <string.h>
#include <stdlib.h>
//#include "manc_logo.h" //copied/pasted/saved from MCR2 github 
//#include "puzzle_logo.h"

// -------- THINGS THAT NEED TO BE CHANGED ----------------------------------------------
int LIGHT_COUNT = 2;   // number of LEDs in daisy chain, declared in set-up loop
int LED_COUNT;     // fn of LIGHT_COUNT calculated in setup loop; then update strip
int lightState = 1;    // make sure this is initialized to the same color as light 1 (green)
int startFlag = 0;
char stringvar [1];
int MAX_LIGHTS = 2;



// NeoPixel brightness, 0 (min) to 255 (max)
#define BRIGHTNESS 5 // Set BRIGHTNESS to about 1/5 (max = 255)


// Misc Var declarations ---------------------------------------------------------------

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(0, LED_PIN, NEO_GRBW + NEO_KHZ800);
// for Arduino: Adafruit_NeoPixel strip("Empty", LED_PIN, NEO_GRBW + NEO_KHZ800);
//   Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_GRBW    Pixels are wired for GRBW bitstream 

// Define states
#define green 1
#define yellow 2
#define red 3
#define error 4

// Setup loop ---------------------------------------------------------------
void setup() {
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(BRIGHTNESS);
  Serial.begin(9600); //for debugging purposes big sigh
  
  //infrastructure to update empty strip so that we can take in user input 
  //for #lights here and solve for #LEDs
  LED_COUNT = 3*LIGHT_COUNT; 
  strip.updateLength(LED_COUNT);
}


// MAIN loop ---------------------------------------------------------------
void loop() {
  // counter ++; //likely not needed until fixing loop durations
  if(LIGHT_COUNT == 1){
    if(lightState == green){ 
      //update light1 to green, pause, then update to yellow
      Serial.print("green 1");Serial.println();
      greenLight(1);
      strip.show(); //  Update strip to match
      delay(greenTimer*1000); //convert to ms
  
      //set middle light to YELLOW, wait 1s
      Serial.print("yellow");Serial.println();
      yellowLight(1);
      strip.show(); //  Update strip to match
      delay(yellowTimer*1000);
      lightState = red; 
      
    }else if(lightState == red){
      //update light1 to red and light2 to green, pause, then update both to yellow
      Serial.print("red 1"); Serial.println();
      redLight(1);
      strip.show(); //  Update strip to match
      delay(greenTimer*1000); //convert to ms
  
      //set middle light to YELLOW, wait 1s
      Serial.print("yellow again"); Serial.println();
      yellowLight(1);
      strip.show(); //  Update strip to match
      delay(yellowTimer*1000);
  
      lightState = green; 
    } else { //if lightState is not red or green
      errorLight(1);
      Serial.print("we are in crisis");
      Serial.println();
    } //  end lightState if statements
  }//end if LIGHT_COUNT == 1
  
  if(LIGHT_COUNT == 2){ 
    if(lightState == green){ 
      //update light1 to green and light2 to red, pause, then update both to yellow
      Serial.print("green");Serial.println();
      greenLight(1);
      redLight(2);
      strip.show(); //  Update strip to match
      delay(greenTimer*1000); //convert to ms
  
      //set middle light to YELLOW, wait 1s
      Serial.print("yellow");Serial.println();
      yellowLight(1);
//      yellowLight(2);/
      strip.show(); //  Update strip to match
      delay(yellowTimer*1000);
      lightState = red; 
      
    }else if(lightState == red){
      //update light1 to red and light2 to green, pause, then update both to yellow
      Serial.print("red"); Serial.println();
      greenLight(2);
      redLight(1);
      strip.show(); //  Update strip to match
      delay(greenTimer*1000); //convert to ms
      yellowLight(2);//
      strip.show(); //  Update strip to match
      delay(yellowTimer*1000);
      lightState = green; 
    } else { //if lightState is not red or green
      errorLight(1);
      errorLight(2);
      Serial.print("we are in crisis");
      Serial.println();
    } //  end lightState if statements
  } // end if LIGHTCOUNT == 2
} // end MAIN loop 

// LIGHT Functions GRBW --------------------------------------------------------------------
void clearLight(int lightNumber){
  // pattern of mapping from TL to LED: redLED = numTL*2 + (numTL-3)
  int redLED = lightNumber*2 + (lightNumber-3); //solve for redLED # in the LED strip
  int yellowLED = redLED + 1;                   // then solve for yellow and green
  int greenLED = redLED + 2;
  strip.setPixelColor(redLED, strip.Color(0,0,0,0)); //top light red
  strip.setPixelColor(yellowLED, strip.Color(0,0,0,0)); //middle light off
  strip.setPixelColor(greenLED, strip.Color(0,0,0,0)); //bottom light off
 }
 
void redLight(int lightNumber){
  //set top light to RED
  // pattern of mapping from TL to LED: redLED = numTL*2 + (numTL-3)
  int redLED = lightNumber*2 + (lightNumber-3); //solve for redLED # in the LED strip
  int yellowLED = redLED + 1;                   // then solve for yellow and green
  int greenLED = redLED + 2;
  strip.setPixelColor(redLED, strip.Color(175,0,0,5)); //top light red
  strip.setPixelColor(yellowLED, strip.Color(0,0,0,0)); //middle light off
  strip.setPixelColor(greenLED, strip.Color(0,0,0,0)); //bottom light off
 }

//GRBW
 void yellowLight(int lightNumber) {
  //set middle light to YELOW
  // pattern of mapping from TL to LED: redLED = numTL*2 + (numTL-3)
  int redLED = lightNumber*2 + (lightNumber-3); //solve for redLED # in the LED strip
  int yellowLED = redLED + 1;                   // then solve for yellow and green
  int greenLED = redLED + 2;
  strip.setPixelColor(redLED, strip.Color(0,0,0,0));         //top light off
  strip.setPixelColor(yellowLED, strip.Color(175,175,0,5));  //middle light yellow
  strip.setPixelColor(greenLED, strip.Color(0,0,0,0));       //bottom light off
 }

 void greenLight(int lightNumber) {
  //set bottom light to GREEN
  // pattern of mapping from TL to LED: redLED = numTL*2 + (numTL-3)
  int redLED = lightNumber*2 + (lightNumber-3); //solve for redLED # in the LED strip
  int yellowLED = redLED + 1;                   // then solve for yellow and green
  int greenLED = redLED + 2;
  strip.setPixelColor(redLED, strip.Color(0,0,0,0));   //top light off
  strip.setPixelColor(yellowLED, strip.Color(0,0,0,0));   //middle light off
  strip.setPixelColor(greenLED, strip.Color(0,175,0,5)); //bottom light green!
 }
//GRBW
 void errorLight(int lightNumber) {
  //set all LEDs to blue
  int redLED = lightNumber*2 + (lightNumber-3); //solve for redLED # in the LED strip
  int yellowLED = redLED + 1;                   // then solve for yellow and green
  int greenLED = redLED + 2;
  strip.setPixelColor(redLED, strip.Color(0,0,255,0));   //top light off
  strip.setPixelColor(yellowLED, strip.Color(0,0,255,0));   //middle light off
  strip.setPixelColor(greenLED, strip.Color(0,0,255,0)); //bottom light green!
 }
 

 
