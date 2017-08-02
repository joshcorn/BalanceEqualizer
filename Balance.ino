/* -----------------------------------------------------------------------
 * Programmer:   Cody Hazelwood
 * Date:         March 20, 2012
 * Platform:     Arduino Uno
 * Description:  Calibrates a motorized fader's max and min
 *               position.  Allows changing the position with an 
 *               external potentiometer.  Uses a capacitance 
 *               sensing circuit for touch sensitivity.
 *               More or less a proof of concept to be used in a future 
 *               project.
 * Dependencies: CapSense Arduino Library (for fader touch sensitivity)
 *               http://www.arduino.cc/playground/Main/CapSense
 * -----------------------------------------------------------------------
 * Copyright 2012.  Cody Hazelwood.
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
 * -----------------------------------------------------------------------
 */
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>
#include <CapacitiveSensor.h>

#define DS_pin  4    // MotorControl Data
#define STCP_pin  5  // MotorControl Latch
#define SHCP_pin  6  // MotorControl Serial
#define touchSend 7

#define NEO_PIN   3  // NeoPixel DATA

#define NEO_PTYPE NEO_GRBW  // f.e. SK6812
#define NUMPIXELS  30
#define BRIGHTNESS  255 //
#define RED     0x00FF0000 // --> RED
#define GREEN   0x0000FF00 // --> GREEN
#define BLUE    0x000000FF // --> BLUE
#define WHITE   0xFF000000 // --> WHITE LED
#define INIT_MSG  "Strip is set to 32-bit type (RGBW)"

#define IWAIT   2000
#define SWAIT   25
#define LWAIT   50
#define HWAIT   5000

#define totalMax      12
#define indMax        8


#define numOfRegisterPins   24
int currentTotal  = 0;
boolean registers[numOfRegisterPins];


uint32_t colors[4] = {RED, GREEN, BLUE, WHITE};
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, NEO_PIN, NEO_PTYPE + NEO_KHZ800);



class Fader {
 
  int ledStart; //number of first LED in chain
  int wiper; //fader read pin
  int motorUp;
  int motorDn;
  int touchReceive; //Receive pin for capacitive sensing circuit
  int currentInd;


  
  int newR;
  int newB;
  int newW; 
  int currentR;
  int currentB;
  int currentW;
  double losslessBrightness;
  
  int debounce;
  int DCgap;
  int eventLast;
  boolean doubleEvent;
  boolean buttonVal;
  boolean buttonLast;
  boolean DCwaiting;
  boolean DConUp;
  boolean singleOK; // whether it's OK to do a single click
  long downTime; // time the button was pressed down
  long upTime; // time the button was released

  double faderMax;
  double lastValue;
  double faderMin;

  CapacitiveSensor touchLine;

  

  volatile bool touched; //Is the fader currently being touched?

  public:
    Fader(int led, int pot, int motUp, int motDn, int capR): touchLine(CapacitiveSensor(touchSend, capR)){
      ledStart = led;
      wiper = pot;
      motorUp = motUp;
      motorDn = motDn;
      
      losslessBrightness = 0;

      debounce = 20;
      newW=250;
      DCgap = 100;
      eventLast = 0;
      doubleEvent = false;
      buttonVal = 1;
      buttonLast = 1;
      DCwaiting = false;
      DConUp = false;
      singleOK = true;
      downTime = -1; 
      upTime = -1; 

      faderMax = 1020;
      faderMin = 2;
      touched  = false;
    }
    
 

    /*void calibrateFader() {
      registers[motorUp] = HIGH;
      writereg();
      delay(250);
      registers[motorUp] = LOW;
      writereg();    
      faderMax = analogRead(wiper);
    
      registers[motorDn] = HIGH;
      writereg();
      delay(250);
      registers[motorDn] = LOW;
      writereg();
      faderMin = analogRead(wiper);
    }
    */

    void checkTouch() {
      //Serial.println(touchLine.capacitiveSensor(30));
      touched = touchLine.capacitiveSensor(30) > 700;
      buttonVal = touched;      
                                                
    }

    void clearRegisters(){
      for(int i = numOfRegisterPins - 1; i >=  0; i--){
         registers[i] = LOW;
      }
    } 
    
    void writeRegisters(){
      digitalWrite(STCP_pin, LOW);
      for(int i = numOfRegisterPins - 1; i >=  0; i--){
        digitalWrite(SHCP_pin, LOW);
        int val = registers[i];
        digitalWrite(DS_pin, val);
        digitalWrite(SHCP_pin, HIGH);
      }
      digitalWrite(STCP_pin, HIGH);
    }
    
    void setRegisterPin(int index, int value){
      registers[index] = value;
    }

    int checkButton() {
      int event = 0;
     // Button pressed
      if (buttonVal == LOW && buttonLast == HIGH && (millis() - upTime) > debounce) {
        downTime = millis();
        singleOK = true;
        if ((millis()-upTime) < DCgap && DConUp == false && DCwaiting == true) DConUp = true;
        else DConUp = false;
        DCwaiting = false;
      }
      // Button released
      else if (buttonVal == HIGH && buttonLast == LOW && (millis() - downTime) > debounce) { 
        upTime = millis();
        if (DConUp == false) DCwaiting = true;
        else {
          event = 2;
          DConUp = false;
          DCwaiting = false;
          singleOK = false;
        } 
      }
      buttonLast = buttonVal;
      return event;
    }

    void updateColor(){
      int linear = analogRead(wiper);
      checkTouch();
      eventLast = checkButton();
      if(linear >= 2 && linear < 510){
        newR = map(linear, 2, 509, 255, 1);
        newW = map(linear, 2, 509, 1, 255);
        newB = 0;
      }
      if(linear >= 510 && linear < 1020){
        newR = 0;
        newW = map(linear, 510, 1020, 255, 1);
        newB = map(linear, 510, 1020, 1, 255);
      }
      for(int i = ledStart; i < ledStart+5; i++) {
        strip.setPixelColor(i, newR, 0, newB, newW);
      }
      strip.show();
    }

    void updateBrightness(){
      int linear = analogRead(wiper);
      
      losslessBrightness = map(analogRead(wiper), 2, 1024, 0, 100);
      currentR = newR * (losslessBrightness/100);
      currentW = newW * (losslessBrightness/100);
      currentB = newB * (losslessBrightness/100);
      for(int i = ledStart; i < ledStart+5; i++) {
        strip.setPixelColor(i, currentR, 0, currentB, currentW);
      }
      strip.show();
    }

    void updateLocation(){
      int linear = analogRead(wiper);
      
      currentInd = map(linear, 2, 1020, 0, indMax);
      currentTotal -= lastValue;
      currentTotal += currentInd;
      
      if(currentTotal > totalMax && !touched){
      delay(200);
        int newDiff = abs(totalMax - currentTotal);
        currentTotal -= currentInd;
        
        while( newDiff > 0 && !touched){
        
          int linear = analogRead(wiper);
          currentInd = map(linear, 2, 1020, 0, indMax);
          currentTotal += currentInd;
          newDiff = abs(totalMax - currentTotal);
          setRegisterPin(motorDn, HIGH);
          writeRegisters();
          currentTotal -= currentInd;
 
        }
        setRegisterPin(motorDn, LOW);
        writeRegisters();
        currentTotal += currentInd;
      }
        currentTotal -= currentInd;
        lastValue = currentInd;
        currentTotal += lastValue; 
        //Serial.println(currentTotal);
    }
      
    void Update(){
      checkTouch();
      eventLast = checkButton();
      
      if(eventLast==2) doubleEvent = !doubleEvent;

      /*while(doubleEvent == true){
        checkTouch();
        eventLast = checkButton();
        updateColor();
        if(eventLast==2){
          doubleEvent = !doubleEvent;
          break;
        }
      }*/
      
      while(doubleEvent == true){
        int fader = analogRead(wiper);
        checkTouch();
        eventLast = checkButton();
        strip.setBrightness(255);
        if(fader >= 2 && fader < 510){
          newR = map(fader, 2, 509, 255, 1);
          newW = map(fader, 2, 509, 1, 255);
          newB = 0;
          }
        if(fader>=510 && fader<1020){
          newR = 0;
          newW = map(fader, 510, 1020, 255, 1);
          newB = map(fader, 510, 1020, 1, 255);
         }
        for(int i = ledStart; i < ledStart+5; i++) {
          strip.setPixelColor(i, newR, 0, newB, newW);
        }
        strip.show();
        if(eventLast==2){
          doubleEvent = !doubleEvent;
          break;
        }
     }
    
      updateBrightness();
      updateLocation();
    }
};

Fader fader1(0, A0, 1, 0, 8);
Fader fader2(5, A1, 3, 2, 9);
Fader fader3(10, A2, 9, 8, 10);
Fader fader4(15, A3, 11, 10, 11);
Fader fader5(20, A4, 17, 16, 12);
Fader fader6(25, A5, 19, 18, 13);


void setup() {    
  #ifdef LED_BUILTIN
      pinMode(LED_BUILTIN, OUTPUT); 
      digitalWrite(LED_BUILTIN, LOW);
  #endif

  #ifdef IWAIT
  delay(IWAIT);
  #endif
    

  strip.begin(); // This initializes the NeoPixel library.
  strip.setBrightness(BRIGHTNESS); // set brightness

  pinMode(STCP_pin, OUTPUT);
  pinMode(SHCP_pin, OUTPUT);  
  pinMode(DS_pin, OUTPUT);
  fader1.clearRegisters();
  fader1.writeRegisters();
  Serial.begin(9600);
  
  //fader1.calibrateFader();
  setAllPixels(colors[3]);
  strip.show();
} 

void loop() {   
    fader1.Update();
    fader2.Update();
    fader3.Update();
    fader4.Update();
    fader5.Update();
    fader6.Update();
    //Serial.println(fader1.touchState());
}

void setAllPixels(uint32_t c) {
  for(uint16_t i=0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
}

   /*if (state < analogRead(wiper1) - 10 && state > faderMin && !touched) {
        registers[0] = HIGH;
        writereg();
        while (state < analogRead(wiper1) - 10 && !touched) {};  //Loops until motor is done moving
        registers[0] = LOW;
        writereg();
    }
    else if (state > analogRead(wiper1) + 10 && state < faderMax && !touched) {
        registers[1] = HIGH;
        writereg();
        while (state > analogRead(wiper1) + 10 && !touched) {}; //Loops until motor is done moving
        registers[1] = LOW;
        writereg();
    }
*/

