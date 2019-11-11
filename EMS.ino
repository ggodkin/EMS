/*
* Project Name: EMS 
* File: EMS.ino
* Description: Electronic muscle stimulator
* Use at you own risk
* Author: Greg Godkin.
* Created May 2019
* URL: https://github.com/ggodkin/EMS
*/
/*
#include <TM1638plus.h>

// GPIO I/O pins on the Arduino connected to strobe, clock, data, 
//pick on any I/O you want.
#define  STROBE_TM 0
#define  CLOCK_TM 1
#define  DIO_TM 2

//Constructor object
TM1638plus tm(STROBE_TM, CLOCK_TM , DIO_TM);

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
uint8_t buttonState;             // the current reading from the buttons
uint8_t lastButtonState = 0;   // the previous reading from the buttons

unsigned int displayBrightness = 0;
unsigned int ctrlPWM = 1;
unsigned int ctrlFreqDiv = 256;

void setup() {
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  tm.reset();
  resetPWM();
  //Serial.begin(115200);
}


void loop() {
  uint8_t buttons = tm.readButtons();
  // If the switch changed, due to noise or pressing:
  if (buttons != lastButtonState) {
   // reset the debouncing timer
   lastDebounceTime = millis();
  }
  long timeSinceLastPress = (millis() - lastDebounceTime);
  if (timeSinceLastPress > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (buttons != buttonState) {
      buttonState = buttons;
      if (buttons == 1) {
        resetPWM();
      } else if (buttons == 2) {
        calcPWM(ctrlPWM, 0, 1); //Decrement PWM by 1
      } else if (buttons == 4) {
        calcPWM(ctrlPWM, 0, 10); //Decrement PWM by 10
      } else if (buttons == 8) {
        calcPWM(ctrlPWM, 1, 0); //Increment PWM by 1
      } else if (buttons == 16) {
        calcPWM(ctrlPWM, 10, 0); //Increment PWM by 10
      } else if (buttons == 32) { //Decrease frequency
        if (ctrlFreqDiv <= 64) {
          ctrlFreqDiv = 64;
        } else {
          ctrlFreqDiv = ctrlFreqDiv / 4;
        }
        setTimerOC2PreScaler(ctrlFreqDiv);
      } else if (buttons == 64) { //Increase frequency
        if (ctrlFreqDiv >= 1024) {
          ctrlFreqDiv = 1024;
        } else {
          ctrlFreqDiv = ctrlFreqDiv * 4;
        }
        setTimerOC2PreScaler(ctrlFreqDiv);
      }
     }
  }
  doLEDs(buttons);
  lastButtonState = buttons;
}

// scans the individual bits of value
void doLEDs(uint8_t value) {
  for (uint8_t position = 0; position < 8; position++) {
    tm.setLED(position, value & 1);
    value = value >> 1;
  }
}

void setTimerOC2PreScaler(int preScaler) {
  if (preScaler == 1024) {
    TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20); //clkT2S/1024 (from prescaler)
  } else if (preScaler == 64) {
    TCCR2B = _BV(CS22); //clkT2S/64 (from prescaler)  }
  } else { 
    TCCR2B = _BV(CS22) | _BV(CS21); //clkT2S/256 (from prescaler)
  }
  ctrlFreqDiv = preScaler;
  calcPWM(ctrlPWM, 0, 0);
  setDisplay();
}

void setPWM(int valOCR2A, int valOCR2B) {
  OCR2A = valOCR2A;
  OCR2B = valOCR2B;
  setDisplay();
}

void calcPWM(int currentPWM, int valIncrement, int valDecrement) {
  int maxPWM = 127;
  int resPWM = currentPWM;
  if (resPWM <= valDecrement) {
    resPWM = 1;
  } else if ((resPWM + valIncrement) > maxPWM) {
    resPWM = maxPWM;
  } else {
    resPWM = currentPWM + valIncrement - valDecrement;
  }
  ctrlPWM = resPWM;
  setPWM(resPWM, 255 - resPWM);
}

void resetPWM () {
  setTimerOC2PreScaler(ctrlFreqDiv);
  calcPWM(ctrlPWM, 0, 256);
  setDisplay();
}

void setDisplay() {
  char valStr[8] = "";
  char charPWM[8] = "";
  char charFreq[8] = "";
  if (ctrlFreqDiv == 1024) {
    strcat(charFreq, "32 H");
  } else if (ctrlFreqDiv == 64) {
    strcat(charFreq, " 2 H");
  } else { 
    strcat(charFreq, " 8 H");
  }

  itoa(ctrlPWM,charPWM, 10);
  strcat(charPWM,"t");
  strcat(valStr,charFreq);
  strcat(valStr,charPWM);
  tm.brightness(displayBrightness);
  //tm.displayText("        ");
//itoa(ctrlFreqDiv,valStr, 10);
  tm.displayText(valStr);
}
*/
///*
#include <TM1638lite.h>
#include <TM1638plus.h>
// I/O pins on the Arduino connected to strobe, clock, data
TM1638plus tm(0, 1, 2);
//TM1638lite tm(0, 1, 2);
//Analog input to control Duty cycle
  int analogPin = A0;
  int valADC = 0;
  int aPWM = 1;
  int bPWM = 254;
  int valInd = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pins as an output.
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  analogReference(DEFAULT); //power supply

  for (int i = 4; i <= 10; i++) {
    pinMode (i,OUTPUT);
  }
  tm.reset();
  tm.displayText("starting");
}

// the loop function runs over and over again forever
void loop() {
  uint8_t buttons = tm.readButtons();
  doLEDs(buttons);
  //TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);  //Fast PWM
  TCCR2A = _BV(COM2A1) | _BV(COM2B1)  | _BV(COM2B0) | _BV(WGM20);  //Phase-Correct PWM
  //TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20); //clkT2S/1024 (from prescaler)
  TCCR2B = _BV(CS22) | _BV(CS21); //clkT2S/256 (from prescaler)
  //TCCR2B = _BV(CS22); //clkT2S/64 (from prescaler)
  valADC = analogRead(analogPin);
  aPWM = valADC / 8;
  if (aPWM <1) 
    aPWM = 1;
  if (aPWM > 127) 
    aPWM = 127;
  bPWM = 255-aPWM;
  OCR2A = aPWM;
  OCR2B = bPWM;
  char tmpV[10];
  itoa(aPWM,tmpV, 10);
  tm.displayText(tmpV);
  valInd = aPWM/19;
  for (int il = 4; il <= 10; il++) {
    digitalWrite (il,LOW);
  }
  for (int iH = 4; iH <= valInd+4; iH++) {
    digitalWrite (iH,HIGH);
  }
}
void doLEDs(uint8_t value) {
  for (uint8_t position = 0; position < 8; position++) {
    tm.setLED(position, value & 1);
    value = value >> 1;
  }
}
//*/
