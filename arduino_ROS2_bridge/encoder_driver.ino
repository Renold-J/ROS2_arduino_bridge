/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   ************************************************************ */

#ifdef USE_BASE

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }

#elif defined(ARDUINO_ENC_COUNTER)
  // Updated for the type of encoder used in your project

  // Encoder count variables (volatile for interrupt use)
  volatile long encoderCountA = 0;  // Motor A encoder count
  volatile long encoderCountB = 0;  // Motor B encoder count

  // Pin assignments for Motor A and Motor B encoders
  const int ENCA1 = 18;  // Motor A Encoder A pin
  const int ENCA2 = 19;  // Motor A Encoder B pin
  const int ENCB1 = 17;  // Motor B Encoder A pin
  const int ENCB2 = 16;  // Motor B Encoder B pin

  // Encoder tick function for Motor A
  void encoderTickA() {
    encoderCountA++;  // Increment encoder count for Motor A
  }

  // Encoder tick function for Motor B
  void encoderTickB() {
    encoderCountB++;  // Increment encoder count for Motor B
  }

  void setupEncoders() {
    // Set up interrupts for Motor A encoder
    attachInterrupt(digitalPinToInterrupt(ENCA1), encoderTickA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCA2), encoderTickA, CHANGE);

    // Set up interrupts for Motor B encoder
    attachInterrupt(digitalPinToInterrupt(ENCB1), encoderTickB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB2), encoderTickB, CHANGE);
  }

  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoderCountA;
    else return encoderCountB;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) {
      encoderCountA = 0;
    } else {
      encoderCountB = 0;
    }
  }

#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif
