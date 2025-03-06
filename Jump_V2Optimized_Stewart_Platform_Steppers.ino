/*************************************************** 
  3RPS Stewart Platform Ball Bounce & Balance
  ----------------------------------------------
  @description: this program operates a 3DOF RPS Stewart Platform to balance a ball on a platform
                using computer vision for position detection & PID controller to control the balls position. 

  @authors: Jose Ramirez & Ethan Lendo
  @institution: Cal Poly Pomona, Dept. of Mech. Engineering
  @date: 07-22-2024
  @version: 1.0

 ****************************************************/
 /* TO-DO
    -> tune the PID
    -> Develop bounce
    -> Develop ball patterns

*/
#include <Arduino.h>
#include <Wire.h>
#include <Pixy2.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "InverseKinematics.h"

#define DEBUG 0 // for debugging 1 is on, else off 0

#if DEBUG == 1
#define debug(db) Serial.print(db)
#define debugln(db) Serial.println(db)
#else
#define debug(db)
#define debugln(db)
#endif

// Camera Object
Pixy2 pixy;

// Pixy2 camera offsets (camera's center in pixels)
constexpr double xOffset = 158.0;     // Updated x offset of the platform's center in pixels
constexpr double yOffset = 104.0;     // Updated y offset of the platform's center in pixels
constexpr double zOffset = 0.0;

// Platform and camera parameters
constexpr double circumscribedRadius = 240.0;   // 240 Radius of the circumscribed circle for the hexagon in mm
constexpr double distanceToPlatform = 460.0;    // Distance from camera to the platform in mm
constexpr int cBall= 2;                         //Calibration constant (derived from the known distance and size)
int actualBallSize= 40;

double ball[3];               // X and Y coordinates of the ball
constexpr int x = 0, y = 1, z = 2;   // Define x, y array indexes
bool detected = false;        // flag to verify ball is detected

// Stepper Motors Object
AccelStepper stepperA(AccelStepper::DRIVER, 9, 8);  //(driver type, STEP, DIR) Driver A
AccelStepper stepperB(AccelStepper::DRIVER, 5, 4);  //(driver type, STEP, DIR) Driver B
AccelStepper stepperC(AccelStepper::DRIVER, 3, 2);  //(driver type, STEP, DIR) Driver C

// Create instance of MultiStepper
MultiStepper steppers;           

// Stepper Motor Variables
double speed[3] = {0, 0, 0};     // Motor speed array
double speedPrev[3] = {0, 0, 0}; // Previous motor speed array
int pos[3] = {0, 0, 0};          // Array to hold angle position of each servo
constexpr double angleOrigin = -2.24;  // Origin angle at the start
constexpr double ks = 6000.0;           // Speed multiplier constant
constexpr double angleToStep = 1600.0 / 360.0; // Convert angle to step count (microsteps * gearRatio / 360 degrees)
constexpr double gearRatio = 5.18;

// PID Constants (2s settling time)
double kp = 0.150, ki = 0.005, kd = 0.11; //0.002

//double kp = 0.230, ki = 0.0118, kd = 0.046; 
//double kp = 0.045, ki = 0.00002, kd = 0.11; //much better 
//double kp = 0.0475, ki = 0.00075, kd = 0.0778;
//double kp = 0.0830, ki = 0.0019, kd = 0.082; 

//double kp = 0.0415, ki = 0.00095, kd = 0.081; //Nearly Perfect
//double kp = 0.05, ki = 0.002, kd = 0.08; 
//double kp = 0.072, ki = 0.0025, kd = 0.07;
//double kp = 0.075, ki = 0.0025, kd = 0.07; //pretty good
//double kp = 0.068, ki = 0.0025, kd = 0.078; // almost perfect
//double kp = 0.068, ki = 0.002, kd = 0.25
//double kp = 0.03263, ki = 0.001886, kd = 0.1411;
//double kp = 0.01658, ki = 0.0008346, kd = 0.0823;
//double kp = 0.04302, ki = 0.002855, kd = 0.1621;
//double kp = 0.07756, ki = 0.004005, kd = 0.3755;

double kpz = 0.0, kiz = 0.0, kdz = 0.0; // PID for z values

double error[3] = {0, 0, 0}, errorPrev[3] = {0, 0, 0}, integr[3] = {0, 0, 0}, deriv[3] = {0, 0, 0}, out[3] = {0, 0, 0};  // PID terms for X and Y directions outf[2] = {0, 0}

// Add a sample time for PID calculations
//constexpr unsigned long sampleTime = 20; // Sample time in milliseconds
// Variables for time management
unsigned long previousTimePID = 0;
unsigned long timeI = 0;
double elapsedTimePID;

/*  Plant(b, p, f, g)
      b = distance from the center of the base to any of its corners
      P = distance from the center of the platform to any of its corners
      f = length of link #1
      s = length of link #2
*/
Plant plant(125.0, 125.0, 100.0, 60.0);

// Function Declarations
void moveTo(double hz, double nx, double ny);
void findBall();
void PID(double setPointX, double setPointY);
void convertToMillimeters(double &pixelX, double &pixelY);

///////////////////////////////////////////////////////////////////////////////////////
/// 
/// @brief Arduino setup() Function.
/// 
///   This function initializes serial communication with PIXY2 and the initial servo 
///   parameters needed to initialize the platform. 
///  
//////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  
  // Initialize Pixy2
  pixy.init();

  // Set max speed and acceleration for steppers
  stepperA.setMaxSpeed(6000);
  stepperA.setAcceleration(3000);

  stepperB.setMaxSpeed(6000);
  stepperB.setAcceleration(3000);

  stepperC.setMaxSpeed(6000);
  stepperC.setAcceleration(3000);

  // SteppersControl instance for multi stepper control
  steppers.addStepper(stepperA);
  steppers.addStepper(stepperB);
  steppers.addStepper(stepperC);

  moveTo(37.75, 0, 0);  // Moves the platform to the home position located at hz=37.75mm
  delay(100);
}
int j = 0;
///////////////////////////////////////////////////////////////////////////////////////
/// 
/// @brief Main Arduino Loop() Function.
/// 
///   This loop executes the PID control of the Platform. 
/// 
//////////////////////////////////////////////////////////////////////////////////////
void loop() {
  //while ((ball[x] < -5 || ball[x] > 5) && (ball[y] < -5 || ball[y] > 5)){
    PID(0, 0, 0);  // (X setpoint, Y setpoint
  //PID(0, 0, 0);  // (X setpoint, Y setpoint)
  //detected = false;
  //moveTo(37.75, 0 ,0);
}

/// @fn Find the location of the ball using the pixy2 cam 
void findBall(double pointSetZ) {
  // Request blocks (objects) from Pixy2
  int numBlocks = pixy.ccc.getBlocks();

  // debugln((String)"Number of Balls Detected: " + numBlocks);

  if (numBlocks == 1) {
    detected = true;

    // Set current X and Y coordinates for object 0 (assuming this is the ball)
    ball[x] = pixy.ccc.blocks[0].m_y;  // Absolute Y location of the ball, now treated as X
    ball[y] = pixy.ccc.blocks[0].m_x;  // Absolute X location of the ball, now treated as Y
    int ballZ = pixy.ccc.blocks[0].m_width;  // Example size from Pixy2 detection
    bool maxZ = false;
    
    // only execute if setpoint for Z is not 0
    if (pointSetZ != 0) {
      // Find the Max height of the ball after bounce 
      while (!maxZ) {
        int currentValue = ballZ;
        delay(5);
        int nextValue = pixy.ccc.blocks[0].m_width;
        
        // Check if the current value is greater than the next
        if (currentValue < nextValue) {
          ball[z]= currentValue; // Update the highest value
          ball[z] /= cBall * actualBallSize;
          maxZ = true;
        } else {
          maxZ = false;
        }
      }
    } else {
      ball[z] = 37.75;
    }

   // debugln((String)"Absolute (X, Y) pixels: (" + ball[x] + ", " + ball[y] + ", " + ball[z] +  ")");

    // Calculate relative coordinates based on the center of the camera's field of view
    ball[x] -= yOffset;
    ball[y] -= xOffset;
    

    debugln((String)"Relative (X, Y) pixels: (" + ball[x] + ", " + ball[y] + ", " + ball[z] +  ")");

    // Convert relative coordinates (pixels) to real-world coordinates (mm)
    convertToMillimeters(ball[x], ball[y]);
    
    //debugln((String)"Real-world (X, Y) mm: (" + ball[x] + ", " + ball[y] + ", " + ball[z] +  ")");

  } else {
    detected = false;
    if (numBlocks > 1) {
      Serial.println("MULTIPLE BALLS DETECTED!");
    } else {
      Serial.println("NO BALL DETECTED");
    }
  }
}

/// @fn PID control to calculate platform movement based on setpoints
void PID(double setPointX, double setPointY, double setPointZ) {
  findBall(setPointZ);  // Find the location of the ball

  if (detected) {
    // Calculate elapsed time since last PID calculation
    unsigned long currentTimePID = millis();
    elapsedTimePID = (currentTimePID - previousTimePID) / 1000.0;  // Convert ms to seconds
    previousTimePID = currentTimePID;
    //Serial.println(elapsedTimePID);

    for (int i = 0; i < 2; i++) {
      // Determine which axis we're calculating for (0 = X, 1 = Y)
      double setPoint = (i == 0) ? setPointX : setPointY;
      double currentPos = (i == 0) ? ball[x] : ball[y];

      // Calculate the error
      error[i] = currentPos - setPoint;

      // Proportional term
      double Pout = kp * error[i];

      // Integral term with windup guard
      integr[i] += error[i] * elapsedTimePID;
      // Limit for the integral to avoid windup
      integr[i] = constrain(integr[i], -1000, 1000);
      double Iout = ki * integr[i];

      // Derivative term (rate of change of error)
      deriv[i] = (error[i] - errorPrev[i]) / elapsedTimePID;
      //deriv[i] = (isnan(deriv[i]) || isinf(deriv[i]) || (deriv[i] < 120 && deriv[i] > 0)|| (deriv[i] > -120 && deriv[i] < 0)) ? 0 : deriv[i];  // Check for validity
     deriv[i] = isnan(deriv[i]) || isinf(deriv[i]) ? 0 : deriv[i];
     // deriv[i] = (0.1 / (50 + elapsedTimePID)) * deriv[i] + (elapsedTimePID / (0.1 + elapsedTimePID)) * deriv[i];

      //Serial.println(deriv[i]);

      double Dout = kd * deriv[i];

      // PID output for the axis
      out[i] = Pout + Iout + Dout;
      //outf[i] = constrain(out[i], -5, 5);
      //debugln((String) "I = " + Iout);

      // Store the current error as previous error for next loop
      errorPrev[i] = error[i];
    }
    
    if (setPointZ != 0) {
    // Calculate the error Z
      double setPoint =  setPointZ;
      double currentPosZ = ball[z];
      error[2] = currentPosZ - setPoint;

      // Proportional term
      double Poutz = kpz * error[2];

      // Integral term with windup guard
      integr[2] += error[2] * elapsedTimePID;
      // Limit for the integral to avoid windup
      integr[2] = constrain(integr[2], -1000, 1000);
      double Ioutz = kiz * integr[2];

      // Derivative term (rate of change of error)
      deriv[2] = (error[2] - errorPrev[2]) / elapsedTimePID;
      deriv[2] = isnan(deriv[2]) || isinf(deriv[2]) ? 0 : deriv[2];  // Check for validity
      double Doutz = kdz * deriv[2];

      // PID output for the axis
      out[2] = Poutz + Ioutz + Doutz;
      out[2] = constrain(out[2], 2, 80);
      
      errorPrev[2] = error[2];

    } else {
        out[2] = 37.75;
        } 
    // Move the platform based on PID output
    
    for (int i = 0; i < 3; i++) {
      speedPrev[i] = speed[i];                                                                                                           //sets previous speed
      speed[i] = (i == A) * stepperA.currentPosition() + (i == B) * stepperB.currentPosition() + (i == C) * stepperC.currentPosition();  //sets current position
      speed[i] = abs(speed[i] - pos[i]) * ks;                                                                                            //calculates the error in the current position and target position
      speed[i] = constrain(speed[i], speedPrev[i] - 500, speedPrev[i] + 500);                                                            //filters speed by preventing it from beign over 100 away from last speed
      speed[i] = constrain(speed[i], 0, 8000);  
    }

    timeI = millis();
    while (millis() - timeI < 9) {
      moveTo(out[2], -out[0], -out[1]);
    }
    delay(1);
    // Debugging output to monitor PID terms and output
    debugln((String) "Input (x, y, z): ( " + ball[x] + ", " + ball[y] + ", " + ball[z] + ")" );
    debugln((String) "Output (x, y, z): ( " + out[x] + ", " + out[y] + ", " + out[z] + ")" );
  
  } else {
    // If ball not detected, retry finding
    Serial.println("NO BALL DETECTED");
    }
}

/// @fn moves the steppers to the desired position based on the given X & Y coordinates
/// @param hz height of the z translation from the platform.
/// @param nx X- coordinate position of the ball relative to origin
/// @param ny Y- coordinate position of the ball relative to origin
void moveTo(double hz, double nx, double ny) {
  // Check if ball is detected
  if (detected) {
    debugln("Ball Detected :) ");
    
    // Calculate Stepper position
    for (int i = 0; i < 3; i++) {
      pos[i] = round((angleOrigin - plant.theta(i, hz, nx, ny)) * angleToStep * gearRatio);

      /*/ Calculate Stepper Motor Speeds
      speedPrev[i] = speed[i];
      speed[i] = abs(pos[i] - ((i == 0) ? stepperA.currentPosition() : (i == 1) ? stepperB.currentPosition() : stepperC.currentPosition())) * ks;
      speed[i] = constrain(speed[i], speedPrev[i] - 500, speedPrev[i] + 500);  // Limit speed change to smooth movement
      speed[i] = constrain(speed[i], 0, 8000);  
      */                              
    }
    
    debugln((String) "Angle A = " + pos[0] / (angleToStep * gearRatio) + " degrees, Speed = " + speed[0] );
    debugln((String) "Angle B = " + pos[1] / (angleToStep * gearRatio) + " degrees, Speed = " + speed[1] );
    debugln((String) "Angle C = " + pos[2] / (angleToStep * gearRatio) + " degrees, Speed = " + speed[2] );
    
    // Set Calculated speed and target position
    stepperA.setMaxSpeed(speed[A]);
    stepperA.setAcceleration(speed[A] * 0.80);
    stepperA.moveTo(pos[A]);

    stepperB.setMaxSpeed(speed[B]);
    stepperB.setAcceleration(speed[B] * 0.80);
    stepperB.moveTo(pos[B]);

    stepperC.setMaxSpeed(speed[C]);
    stepperC.setAcceleration(speed[C] * 0.80);
    stepperC.moveTo(pos[C]);
  
    

    // Run steppers to target position
    //while (stepperA.distanceToGo() != 0 || stepperB.distanceToGo() != 0 || stepperC.distanceToGo() != 0) {
      stepperA.run();
      stepperB.run();
      stepperC.run();
   // }
  } else {
    debugln("Ball NOT Detected :( ");

    for (int i = 0; i < 3; i++) {
      pos[i] = round((angleOrigin - plant.theta(i, hz, 0, 0)) * angleToStep * gearRatio);
    }

    /debugln((String) "Angle A = " + pos[0] / (angleToStep * gearRatio) + " degrees");
    //debugln((String) "Angle B = " + pos[1] / (angleToStep * gearRatio) + " degrees");
    //debugln((String) "Angle C = " + pos[2] / (angleToStep * gearRatio) + " degrees");

    // Set Stepper Max Speed and target position
    stepperA.setAcceleration(7000);
    stepperA.setMaxSpeed(8000);
    stepperA.moveTo(pos[A]);

    stepperB.setAcceleration(7000);
    stepperB.setMaxSpeed(8000);
    stepperB.moveTo(pos[B]);

    stepperC.setAcceleration(7000);
    stepperC.setMaxSpeed(8000);
    stepperC.moveTo(pos[C]);

    // Run steppers to target position
    while (stepperA.distanceToGo() != 0 || stepperB.distanceToGo() != 0 || stepperC.distanceToGo() != 0) {
      stepperA.run();
      stepperB.run();
      stepperC.run();
    }
  }
}

/// @fn Convert pixel coordinates to real-world coordinates in millimeters
void convertToMillimeters(double &pixelX, double &pixelY) {
  // Convert the pixel positions to positions within the hexagonal platform's circumscribed circle
  // Assuming linear transformation, map the pixel range to the circumscribed radius

  pixelX = (pixelX / 208.0) * 2 * circumscribedRadius;  // Scaling X based on pixel range and radius
  pixelY = (pixelY / 316.0) * 2 * circumscribedRadius;  // Scaling Y based on pixel range and radius

  // Ensure ball position stays within the platform's circular bounds
  double distanceFromCenter = sqrt(pixelX * pixelX + pixelY * pixelY);
  if (distanceFromCenter > circumscribedRadius) {
    // Ball is outside the platform bounds
    //debugln("Warning: Ball is outside the platform!");
  }
}