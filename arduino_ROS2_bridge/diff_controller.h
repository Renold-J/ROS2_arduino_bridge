/* Functions and type-defs for PID control.
   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // last input

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  int ITerm;                    // integrated term

  long output;                  // last motor setting
} SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
   leftPID.TargetTicksPerFrame = 0.0;
   leftPID.Encoder = readEncoder(LEFT);  // Initialize encoder counts
   leftPID.PrevEnc = leftPID.Encoder;    // Sync previous encoder with current
   leftPID.output = 0;
   leftPID.PrevInput = 0;
   leftPID.ITerm = 0;

   rightPID.TargetTicksPerFrame = 0.0;
   rightPID.Encoder = readEncoder(RIGHT); // Initialize encoder counts
   rightPID.PrevEnc = rightPID.Encoder;   // Sync previous encoder with current
   rightPID.output = 0;
   rightPID.PrevInput = 0;
   rightPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  input = p->Encoder - p->PrevEnc; // Encoder difference since last frame
  Perror = p->TargetTicksPerFrame - input; // Error between target and actual ticks

  /*
  * Avoid derivative kick and allow tuning changes,
  * by using previous input instead of error directly
  */
  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevEnc = p->Encoder;  // Update previous encoder count for the next loop

  output += p->output;

  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    p->ITerm += Ki * Perror;  // Integral term accumulates error if not saturated

  p->output = output;         // Save the computed output
  p->PrevInput = input;       // Save the current input for the next derivative calculation
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  leftPID.Encoder = readEncoder(LEFT);
  rightPID.Encoder = readEncoder(RIGHT);
  
  /* If we're not moving, there is nothing more to do */
  if (!moving) {
    /*
    * Reset PIDs once, to prevent startup spikes,
    * PrevInput is considered a good proxy to detect
    * whether reset has already happened
    */
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doPID(&rightPID);
  doPID(&leftPID);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftPID.output, rightPID.output);
}

/* Main loop to control the robot's motion based on PID */
void loop() {
  /* Call updatePID() periodically, depending on your loop timing */
  updatePID();

  /* Your existing control code, including checking for motor commands and updating movement */
}

/* Setup function */
void setup() {
  /* Initialize encoders, motors, and serial communication */
  Serial.begin(57600);   // Communication with the serial monitor
  setupEncoders();       // Initialize encoders (from your encoder setup code)
  initMotorController(); // Initialize motor controller (from your motor control code)
  resetPID();            // Initialize PID values
}
