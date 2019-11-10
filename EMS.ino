/*
*/
#include <TM1638lite.h>
// I/O pins on the Arduino connected to strobe, clock, data
//TM1638lite tm(0, 1, 2);
TM1638lite tm(PD4,PD5,PD6);
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

  //for (int i = 4; i <= 10; i++) {
  //  pinMode (i,OUTPUT);
  //}
  tm.reset();
  tm.sendCommand(0x8f);
  //tm.displayText("starting");
  delay(1000);
  Serial.begin(115200);
}

// the loop function runs over and over again forever
void loop() {
  uint8_t buttons = tm.readButtons();
  Serial.println(String(buttons));
  //tm.sendCommand(0x8f);
  tm.displayText(String(buttons));
  doLEDs(buttons);
  //TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);  //Fast PWM
  TCCR2A = _BV(COM2A1) | _BV(COM2B1)  | _BV(COM2B0) | _BV(WGM20);  //Phase-Correct PWM
  //TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20); //clkT2S/1024 (from prescaler)
  TCCR2B = _BV(CS22) | _BV(CS21); //clkT2S/256 (from prescaler)
  //TCCR2B = _BV(CS22); //clkT2S/64 (from prescaler)
  //valADC = analogRead(analogPin);
  aPWM = valADC / 8;
  if (aPWM <1) 
    aPWM = 1;
  if (aPWM > 127) 
    aPWM = 127;
  bPWM = 255-aPWM;
  OCR2A = aPWM;
  OCR2B = bPWM;
  //Serial.println(String(aPWM));
  //tm.displayText(String(aPWM));
  valInd = aPWM/19;
  //linear indicator
  //for (int il = 4; il <= 10; il++) {
  //  digitalWrite (il,LOW);
  //}
  //for (int iH = 4; iH <= valInd+4; iH++) {
  //  digitalWrite (iH,HIGH);
  //}
}
void doLEDs(uint8_t value) {
  for (uint8_t position = 0; position < 8; position++) {
    tm.setLED(position, value & 1);
    value = value >> 1;
  }
}
