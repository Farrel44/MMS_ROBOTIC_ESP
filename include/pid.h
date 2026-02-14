#ifndef PID_H
#define PID_H

#include <Arduino.h>

/**
 * PID Controller with anti-windup and output clamping
 * Features:
 * - Independent dt calculation (micros-based)
 * - Configurable integral limits (prevents windup)
 * - Output saturation limits
 */
class PIDController {
private:
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float prevError;
    unsigned long prevTime;

    float outputMin;
    float outputMax;
    float integralMax;

public:
    PIDController(float kp, float ki, float kd);

    void setOutputLimits(float min, float max);
    void setIntegralLimits(float limit);
    void setTuning(float kp, float ki, float kd);

    float compute(float input, float setpoint);

    void reset();

    float getKp() {return Kp;}
    float getKi() {return Ki;}
    float getKd() {return Kd;}
};

#endif