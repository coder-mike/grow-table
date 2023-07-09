#include "io.h"

#include <Arduino_LSM6DS3.h>

const int redPin = 5;
const int greenPin = 3;
const int bluePin = 4;
const int builtinLed = 13;

const int relayL1Pin = 10; // Middle relay
const int relayL2Pin = 12; // // Bottom relay
const int relayL3Pin = 11; // Top relay (not connected)
const int fanPin = 7;
const int buzzerPin = 2;

const int tempSense1Pin = A1;
const int tempSense2Pin = A2;

// Constants for fan control
const int fanPwmPin = 9; // Pin connected to the PWM input of the fan
const int fanTachPin = 21; // Pin connected to the tachometer output of the fan

// Variables for fan speed measurement
volatile unsigned long lastTachTime = 0; // Time of the last tachometer pulse
volatile unsigned long pulseInterval = 0; // Time between tachometer pulses

//void setPwmFrequency25kHz();
//void configureCustomPWM();
//void setPWMDutyCycle(float dutyCycle);

void initIo() {
  // RGB LED
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);
  
  // Builtin LED
  pinMode(builtinLed, OUTPUT);
  digitalWrite(builtinLed, HIGH);
  
  // Relays
  pinMode(relayL1Pin, OUTPUT);
  pinMode(relayL2Pin, OUTPUT);
  pinMode(relayL3Pin, OUTPUT);
  digitalWrite(relayL1Pin, LOW);
  digitalWrite(relayL2Pin, LOW);
  digitalWrite(relayL3Pin, LOW);

  // Fan
  pinMode(fanPin, OUTPUT);
  digitalWrite(fanPin, LOW);

  // Buzzer
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  

  // Set the analog reference to external
  analogReference(AR_EXTERNAL);
  
  // Initialize fan
  pinMode(fanPwmPin, OUTPUT);
  pinMode(fanTachPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(fanTachPin), tachInterrupt, RISING);
  //setPwmFrequency25kHz();
  //configureCustomPWM();
}

void rgb_off() {
  digitalWrite(redPin, HIGH);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin, HIGH);
}

void rgb_red() {
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin, HIGH);
}

void rgb_green() {
  digitalWrite(redPin, HIGH);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, HIGH);
}

void rgb_blue() {
  digitalWrite(redPin, HIGH);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);
}

void rgb_yellow() {
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, HIGH);
}

void builtinLed_set(bool value) {
  digitalWrite(builtinLed, value);
}


// Relay L1
void relayL1_set(bool value) {
  digitalWrite(relayL1Pin, value ? HIGH : LOW);
}

// Relay L2
void relayL2_set(bool value) {
  digitalWrite(relayL2Pin, value ? HIGH : LOW);
}

// Relay L3
void relayL3_set(bool value) {
  digitalWrite(relayL3Pin, value ? HIGH : LOW);
}

// Fan
void fanOnOff_set(bool value) {
  digitalWrite(fanPin, value ? HIGH : LOW);
}

// Buzzer
void buzzer_on() {
  digitalWrite(buzzerPin, HIGH);
}

void buzzer_off() {
  digitalWrite(buzzerPin, LOW);
}

void buzzer_beepLong() {
  buzzer_on();
  delay(400);
  buzzer_off();
}

void buzzer_beepShort() {
  buzzer_on();
  delay(50);
  buzzer_off();
}

// Returns temperature in thousandths of a degree C
long tempSense1_read() {
  /*
  import numpy as np
  
  referenceVoltage = 2500  # 2.5V in millivolts
  voltageAt15C = 1446
  voltageAt35C = 1283
  tempAt15C = 15000
  tempAt35C = 35000
  SHIFT_AMOUNT = 8
  
  # Convert voltages to their corresponding analog values
  analogAt15C = (voltageAt15C * 1023) / referenceVoltage
  analogAt35C = (voltageAt35C * 1023) / referenceVoltage
  
  # Set up system of equations
  X = np.array([[analogAt15C, 1], [analogAt35C, 1]])
  Y = np.array([tempAt15C, tempAt35C])
  
  # Solve for [m, c]
  [m, c] = np.linalg.solve(X, Y)
  
  # Multiply m by 2**SHIFT_AMOUNT to allow for shifts in Arduino
  m_shifted = m * (2**SHIFT_AMOUNT)
  
  print(f'm = {m_shifted}, c = {c}')
  */
  static const long SHIFT_AMOUNT = 8;
  static const long m = -76762;
  static const long c = 192423;
  long analogValue = analogRead(tempSense1Pin);
  return analogValue * m / (1 << SHIFT_AMOUNT) + c;
}


// This function should be called every time the tachometer sends a pulse
void tachInterrupt() {
  unsigned long currentTime = micros();
  pulseInterval = currentTime - lastTachTime;
  lastTachTime = currentTime;

  // Debugging: print pulse interval
  Serial.print("Pulse Interval (microseconds): ");
  Serial.println(pulseInterval);
}


// Read fan speed as RPM
float fan_readSpeed() {
  noInterrupts();
  unsigned long interval = pulseInterval; // Copy interval into local variable
  interrupts();
  
  // Calculate the frequency (in Hz)
  float frequency = 1.0 / (interval * 0.000001);

  // Convert frequency to RPM and normalize to range [0, 1]
  float rpm = (frequency / 2) * 60;

  return rpm;
}

// // Set fan speed as a ratio from 0.0 to 1.0
// void fan_setSpeed(float speed) {
//   // Clamp speed to the range [0, 1]
//   speed = constrain(speed, 0.0, 1.0);
  
//   // // Convert speed to PWM value and set the PWM output
//   // int pwmValue = (int)(speed * 255.0);
//   // analogWrite(fanPwmPin, pwmValue);
  
//   setPWMDutyCycle(speed);
// }

// Set fan speed as a ratio from 0.0 to 1.0
void fan_setSpeed(float speed) {
  // Clamp speed to the range [0, 1]
  speed = constrain(speed, 0.0, 1.0);
  
  // Convert speed to PWM value and set the PWM output
  int pwmValue = (int)(speed * 255.0);
  analogWrite(fanPwmPin, pwmValue);
}

// void setPwmFrequency25kHz() {
//     // Enable the port multiplexer for the D9 PWM channel
//     PORT->Group[g_APinDescription[9].ulPort].PINCFG[g_APinDescription[9].ulPin].bit.PMUXEN = 1;

//     // Connect the TCC0 timer to the port outputs - port pin D9 (PA07) in this case
//     PORT->Group[g_APinDescription[9].ulPort].PMUX[g_APinDescription[9].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E;

//     // Feed GCLK0 to TCC0 and TCC1
//     GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Enable the generic clock...
//                         GCLK_CLKCTRL_GEN_GCLK0 |     // ...GCLK0
//                         GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK0 to TCC0 and TCC1
//     while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

//     // Set up the TCC0 timer as a PWM
//     TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;          // Normal (single slope) PWM mode
//     while (TCC0->SYNCBUSY.bit.WAVE);                 // Wait for synchronization

//     // Set the PWM frequency to 25kHz
//     TCC0->PER.reg = (uint32_t)(48000000 / 25000) - 1;
//     while (TCC0->SYNCBUSY.bit.PER);                  // Wait for synchronization

//     // Set the PWM duty cycle to 50%
//     TCC0->CC[1].reg = (uint32_t)((48000000 / 25000) / 2) - 1;
//     while (TCC0->SYNCBUSY.bit.CC1);                  // Wait for synchronization

//     // Enable the TCC0 output
//     TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;
//     while (TCC0->SYNCBUSY.bit.ENABLE);               // Wait for synchronization
// }

// void configureCustomPWM() {
//     // Configure the generic clock (GCLK7) to clock the TCC0 and TCC1
//     GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) |          // Divide the clock source by divisor 1
//                       GCLK_GENDIV_ID(7);            // Select Generic Clock (GCLK) 7
//     GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
//                         GCLK_GENCTRL_GENEN |         // Enable GCLK7
//                         GCLK_GENCTRL_SRC_DFLL48M |   // Set the clock source to 48MHz clock
//                         GCLK_GENCTRL_ID(7);          // Select GCLK7
//     GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Enable the generic clock...
//                         GCLK_CLKCTRL_GEN_GCLK7 |     // ...GCLK7
//                         GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK7 to TCC0 and TCC1
//     while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization
    
//     // Enable the port multiplexer for the D9 PWM channel
//     PORT->Group[g_APinDescription[9].ulPort].PINCFG[g_APinDescription[9].ulPin].bit.PMUXEN = 1;
    
//     // Connect the TCC0 timer to the port outputs - port pin D9 (PA07) in this case
//     PORT->Group[g_APinDescription[9].ulPort].PMUX[g_APinDescription[9].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E;
    
//     // Set up the TCC0 timer as a PWM
//     TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;          // Normal (single slope) PWM mode
//     while (TCC0->SYNCBUSY.bit.WAVE);                 // Wait for synchronization

//     // Set the PWM frequency to 25kHz
//     TCC0->PER.reg = (uint32_t)(48000000 / 25000) - 1;
//     while (TCC0->SYNCBUSY.bit.PER);                  // Wait for synchronization

//     // Set the output polarity to normal
//     TCC0->DRVCTRL.reg &= ~TCC_DRVCTRL_INVEN0;  // Not inverted (D9 - WO[0])
    
//     // Enable the TCC0 output
//     TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;
//     while (TCC0->SYNCBUSY.bit.ENABLE);               // Wait for synchronization
// }


// void setPWMDutyCycle(float dutyCycle) { // dutyCycle is between 0.0 and 1.0
//     uint32_t compareValue = (uint32_t)(((48000000 / 25000) - 1) * dutyCycle);
//     TCC0->CC[0].reg = compareValue;
//     while (TCC0->SYNCBUSY.bit.CC0);  // Use CC0 here
// }
