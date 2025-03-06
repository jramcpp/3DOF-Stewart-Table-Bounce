#include "InverseKinematics.h"

// Define constants for calculations
double SQRT_3_OVER_2 = sqrt(3.0) / 2.0;

constexpr double HEIGHT_OFFSET = 77.69;  // Offset for height calculation
constexpr double GRAVITY = 9.81;
constexpr double COEFF = 7.0 / (5.0 * GRAVITY);

// Constructor initializes constants and end effector/base positions
Plant::Plant(double _b, double _p, double _f, double _s) : b(_b), p(_p), f(_f), s(_s) {
    // Initialize end effector positions
    endEffector[A][0] = p;
    endEffector[A][1] = 0;
    endEffector[B][0] = -0.5 * p;
    endEffector[B][1] = SQRT_3_OVER_2 * p;
    endEffector[C][0] = -0.5 * p;
    endEffector[C][1] = -SQRT_3_OVER_2 * p;
    
    // Initialize base positions
    basePrisJoint[A][0] = p;
    basePrisJoint[A][1] = 0;
    basePrisJoint[B][0] = -0.5 * p;
    basePrisJoint[B][1] = SQRT_3_OVER_2 * p;
    basePrisJoint[C][0] = -0.5 * p;
    basePrisJoint[C][1] = -SQRT_3_OVER_2 * p;
}

double Plant::theta(int leg, double H, double xe, double ye) {
    // Calculate height and tilt angles
    double Hz = H + HEIGHT_OFFSET;
    double alpha = ye * COEFF * DEG_TO_RAD;
    double beta = xe * COEFF * DEG_TO_RAD;

    // Precompute trigonometric values
    double cos_alpha = cos(alpha);
    double sin_alpha = sin(alpha);
    double cos_beta = cos(beta);
    double sin_beta = sin(beta);

    double nx, ny, nz, length;

    // Calculate length based on the leg
    switch (leg) {
        case A:
            nx = endEffector[A][0] * cos_beta - basePrisJoint[A][0];
            nz = Hz - endEffector[A][0] * sin_beta;
            length = sqrt(nx * nx + nz * nz);
            break;

        case B:
            nx = endEffector[B][0] * cos_beta + endEffector[B][1] * sin_beta * sin_alpha - basePrisJoint[B][0];
            ny = endEffector[B][1] * cos_alpha - basePrisJoint[B][1];
            nz = Hz - endEffector[B][0] * sin_beta + endEffector[B][1] * cos_beta * sin_alpha;
            length = sqrt(nx * nx + ny * ny + nz * nz);
            break;

        case C:
            nx = endEffector[C][0] * cos_beta + endEffector[C][1] * sin_beta * sin_alpha - basePrisJoint[C][0];
            ny = endEffector[C][1] * cos_alpha - basePrisJoint[C][1];
            nz = Hz - endEffector[C][0] * sin_beta + endEffector[C][1] * cos_beta * sin_alpha;
            length = sqrt(nx * nx + ny * ny + nz * nz);
            break;

        default:
            return 0.0;  // Invalid leg, return 0
    }

    // Calculate the angle and return it in degrees
    double angle = HALF_PI - acos((length * length + s * s - f * f) / (2 * s * length));
    return angle * RAD_TO_DEG;
}
