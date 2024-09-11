#ifndef ENCODER_H
#define ENCODER_H

#ifdef ARDUINO_ENC_COUNTER
  // Update for Arduino Mega: PORTD corresponds to digital pins 21 and 20
  #define LEFT_ENC_PIN_A 18  // PD2 (Digital Pin 21 on Mega)
  #define LEFT_ENC_PIN_B 19  // PD3 (Digital Pin 20 on Mega)
  
  // Update for Arduino Mega: PORTC corresponds to analog pins A8 and A9
  #define RIGHT_ENC_PIN_A 17  // PC4 (Analog Pin A8 on Mega)
  #define RIGHT_ENC_PIN_B 16  // PC5 (Analog Pin A9 on Mega)
#endif

// Declaration of encoder functions
long readEncoder(int i);      // Reads the encoder count for motor i (LEFT or RIGHT)
void resetEncoder(int i);     // Resets the encoder count for motor i (LEFT or RIGHT)
void resetEncoders();         // Resets encoder counts for both motors

#endif  // ENCODER_H