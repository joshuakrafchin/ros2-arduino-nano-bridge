#define USE_BASE      // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
   /* L298 Motor driver */
   #define L298_MOTOR_DRIVER

   /* Encoders directly attached to Arduino board */
   #define ARDUINO_ENC_COUNTER
#endif

//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
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

#include <avr/io.h>

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

/* Include servo support if required */
#ifdef USE_SERVOS
   #include <Servo.h>
   #include "servos.h"
#endif

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
  #define AUTO_STOP_INTERVAL 10000
  long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

/* Variable initialization */

// A pair of variables to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Encoder variables */
volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

// Pin definitions for encoders
#define LEFT_ENC_PIN_A 7
#define LEFT_ENC_PIN_B 8
#define RIGHT_ENC_PIN_A 2
#define RIGHT_ENC_PIN_B 3

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
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
  #ifdef USE_SERVOS
  case SERVO_WRITE:
    servos[arg1].setTargetPosition(arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
    Serial.println(servos[arg1].getServo().read());
    break;
  #endif
    
  #ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(leftEncoderTicks);
    Serial.print(" ");
    Serial.println(rightEncoderTicks);
    break;
   case RESET_ENCODERS:
    leftEncoderTicks = 0;
    rightEncoderTicks = 0;
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    }
    else moving = 1;
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.println("OK"); 
    break;
  case MOTOR_RAW_PWM:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    resetPID();
    moving = 0; // Sneaky way to temporarily disable the PID
    setMotorSpeeds(arg1, arg2);
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;
  #endif
  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Interrupt service routines for encoders */
void handleLeftEncA() {
  if (digitalRead(LEFT_ENC_PIN_A) == digitalRead(LEFT_ENC_PIN_B)) {
    leftEncoderTicks++;
  } else {
    leftEncoderTicks--;
  }
  // Serial.print("Left Encoder Ticks: ");
  // Serial.println(leftEncoderTicks);
}

void handleLeftEncB() {
  if (digitalRead(LEFT_ENC_PIN_A) == digitalRead(LEFT_ENC_PIN_B)) {
    leftEncoderTicks--;
  } else {
    leftEncoderTicks++;
  }
  // Serial.print("Left Encoder Ticks: ");
  // Serial.println(leftEncoderTicks);
}

void handleRightEncA() {
  if (digitalRead(RIGHT_ENC_PIN_A) == digitalRead(RIGHT_ENC_PIN_B)) {
    rightEncoderTicks++;
  } else {
    rightEncoderTicks--;
  }
  // Serial.print("Right Encoder Ticks: ");
  // Serial.println(rightEncoderTicks);
}

void handleRightEncB() {
  if (digitalRead(RIGHT_ENC_PIN_A) == digitalRead(RIGHT_ENC_PIN_B)) {
    rightEncoderTicks--;
  } else {
    rightEncoderTicks++;
  }
  // Serial.print("Right Encoder Ticks: ");
  // Serial.println(rightEncoderTicks);
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);
  Serial.println("Motor controller test starting...");

  // Initialize the motor controller if used
  #ifdef USE_BASE
    #ifdef ARDUINO_ENC_COUNTER
      // Set as inputs
      pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
      pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
      pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
      pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);
      
      // Enable pin change interrupts
      attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), handleLeftEncA, CHANGE);
      attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_B), handleLeftEncB, CHANGE);
      attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), handleRightEncA, CHANGE);
      attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_B), handleRightEncB, CHANGE);
    #endif
    initMotorController();
    resetPID();
  #endif

  // Attach servos if used
  #ifdef USE_SERVOS
    int i;
    for (i = 0; i < N_SERVOS; i++) {
      servos[i].initServo(
          servoPins[i],
          stepDelay[i],
          servoInitPosition[i]);
    }
  #endif

  Serial.println("Setup complete.");
}

void loop() {
  while (Serial.available() > 0) {
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      Serial.println("Running command...");
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  
  // If we are using base control, run a PID calculation at the appropriate intervals
  #ifdef USE_BASE
    if (millis() > nextPID) {
      updatePID();
      nextPID += PID_INTERVAL;
    }
    
    // Check to see if we have exceeded the auto-stop interval
    if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
      setMotorSpeeds(0, 0);
      moving = 0;
    }
  #endif

  // Sweep servos
  #ifdef USE_SERVOS
    int i;
    for (i = 0; i < N_SERVOS; i++) {
      servos[i].doSweep();
    }
  #endif
}