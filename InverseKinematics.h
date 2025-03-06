#ifndef InverseKinematics_H
#define InverseKinematics_H

#include <Arduino.h>
#include <math.h>

// Define servo constants
#define A 0
#define B 1
#define C 2

class Plant {
  public:
    Plant(double b, double p, double f, double s);
    double theta(int leg, double h, double xe, double ye); // returns the angle value of each servo A, B, C

  private:
    double b;  // distance from the center of the base to any of its corners
    double p;  // distance from the center of the platform to any of its corners
    double f;  // length of link #1
    double s;  // length of link #2
    double endEffector[3][2];
    double basePrisJoint[3][2];
};

#endif