#include <SparkFunMAX31855k.h>

/*********************************************************************
 *  ROSArduinoBridge

    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using
    different motor controller or encoder method.

    Authors: Patrick Goebel, James Nugen

    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.
 *********************************************************************/

#define USE_BASE      // Enable the base controller code

#ifdef USE_BASE
   /* Encoders directly attached to Arduino board */
   #define ARDUINO_ENC_COUNTER

   /* L298 Motor driver*/
   #define TB06612FNG
#endif

#undef USE_SERVOS     // Disable use of PWM servos

/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

#ifdef USE_BASE
  /* Motor driver function definitions */
  #include "motor_driver.h"

  /* Encoder driver function definitions */
  #include "encoder_driver.h"

  /* PID parameters and functions */
  #include "diff_controller.h"

  /* Run the PID loop at 30 times per second */
  #define PID_RATE           30     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE;

  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

/* Variable initialization */
int arg = 0;
int index = 0;
char chr;
char cmd;
char argv1[16];
char argv2[16];
long arg1;
long arg2;

/* Encoder counts */
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

/* Function to reset command */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Setup encoder interrupts and pins */
void setupEncoders() {
  DDRD &= ~(1 << 2);  // Set pin 2 as input for encoder A
  DDRD &= ~(1 << 3);  // Set pin 3 as input for encoder B

  PORTD |= (1 << 2);  // Enable pull-up resistors
  PORTD |= (1 << 3);

  attachInterrupt(digitalPinToInterrupt(2), encoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), encoderISR_B, CHANGE);
}

/* Encoder ISR for motor A */
void encoderISR_A() {
  if (digitalRead(2) == digitalRead(3)) {
    encoderCountA++;
  } else {
    encoderCountA--;
  }
}

/* Encoder ISR for motor B */
void encoderISR_B() {
  if (digitalRead(2) == digitalRead(3)) {
    encoderCountB++;
  } else {
    encoderCountB--;
  }
}

/* Read encoder values */
long readEncoder(int i) {
  if (i == LEFT) return encoderCountA;
  else return encoderCountB;
}

/* Reset encoder counts */
void resetEncoders() {
  encoderCountA = 0;
  encoderCountB = 0;
}

/* Run a command */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK");
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK");
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PING:
    Serial.println(Ping(arg1));
    break;
#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
  case RESET_ENCODERS:
    resetEncoders();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
    } else {
      setMotorSpeeds(arg1, arg2);
    }
    Serial.println("OK");
    break;
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function */
void setup() {
  Serial.begin(BAUDRATE);
  setupEncoders();

#ifdef USE_BASE
  initMotorController();
#endif
}

/* Main loop */
void loop() {
  while (Serial.available() > 0) {
    chr = Serial.read();
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    } else if (chr == ' ') {
      if (arg == 0) arg = 1;
      else if (arg == 1) {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
    } else {
      if (arg == 0) {
        cmd = chr;
      } else if (arg == 1) {
        argv1[index] = chr;
        index++;
      } else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }

#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0);
  }
#endif
}
