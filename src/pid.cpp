#include "pid.h"

PIDController::PIDController(float kp, float ki, float kd){
    Kp = kp;
    Ki = ki;
    Kd = kd;

    integral = 0;
    prevError = 0;
    prevTime = 0;

    outputMin = -255;
    outputMax = 255;
    integralMax = 1000;
}

void PIDController::setOutputLimits(float min, float max){
    outputMax = max;
    outputMin = min;
}

void PIDController::setIntegralLimits(float limit){
    integralMax = limit;
}

void PIDController::setTuning(float kp, float ki,float kd){
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

float PIDController::compute(float input, float setpoint){
    // Note: micros() syscall unavoidable here (each PID needs independent timing)
    // Alternative (passing dt from main) would break encapsulation
    unsigned long currentTime = micros();

    float dt = (prevTime == 0) ? 0.01 : (currentTime - prevTime) / 1000000.0;
    prevTime = currentTime;

    // Sanity check: reject invalid dt
    if (dt <= 0.00001 || dt > 1.0) {
        dt = 0.02;
    }

    float error = setpoint - input;

    float P = Kp * error;
    integral += error * dt;

    // Anti-windup: clamp integral term
    if (integral > integralMax) integral = integralMax;
    if (integral < -integralMax) integral = -integralMax;

    float I = Ki * integral;
    float derivative = (error - prevError) / dt;
    float D = Kd * derivative;

    prevError = error;

    float output = P + I + D;

    // Clamp final output
    if (output > outputMax) output = outputMax;
    if (output < outputMin) output = outputMin;
    
    return output;
}

void PIDController::reset() {
    integral = 0;
    prevError = 0;
    prevTime = 0;
}