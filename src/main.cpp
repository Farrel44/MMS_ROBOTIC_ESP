#include <Arduino.h>
#include <ESP32Encoder.h>
#include "pid.h"
#include "serial_protocol.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// === PIN MOTOR ===
#define M1_RPWM 14
#define M1_LPWM 12 
#define M1_EN 13 
#define M2_RPWM 19
#define M2_LPWM 15
#define M2_EN  18
#define M3_RPWM 26
#define M3_LPWM 25
#define M3_EN  27

// === PIN ENCODER (ENC2 & ENC3 swapped - wiring fisik) ===
#define ENC1_A 4
#define ENC1_B 5
#define ENC2_A 32
#define ENC2_B 33
#define ENC3_A 21
#define ENC3_B 22
#define LED_PIN 2

// IMU / MPU 6050
#define SDA 23
#define SCL 17

Adafruit_MPU6050 mpu;
bool imuReady = false;
int16_t lastGoodGyroZ = 0;
int16_t lastGoodAccelZ = 981;
uint8_t imuErrorCount = 0;

// === COMPLEMENTARY FILTER STATE ===
float filteredAngleX = 0.0;
float filteredAngleY = 0.0;
float filteredGyroZ = 0.0;
const float FILTER_ALPHA = 0.95;  //Complementary filter constant (0.95)
unsigned long lastFilterTime = 0;

ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;

// === PHASE 2: BINARY PROTOCOL ===
SerialProtocol protocol(&Serial);
unsigned long lastFeedbackTime = 0;
const int FEEDBACK_INTERVAL = 20;

// === DIRECTION CONFIG ===
const bool INVERT_ENC1 = true;
const bool INVERT_ENC2 = true;
const bool INVERT_ENC3 = true;
const bool INVERT_PWM1 = false;
const bool INVERT_PWM2 = false;
const bool INVERT_PWM3 = false;

// === PID (Feedforward + Trim architecture) ===
// untuk Ki di half dari matlab karena ada feedforward
PIDController pid1(0.40, 4.60, 0.05);
PIDController pid2(0.40, 4.60, 0.05);
PIDController pid3(0.40, 4.60, 0.05);

// === CONSTANTS ===
const int TICKS_PER_REV = 380;
const int MAX_PWM = 255;
const int MAX_RPM = 600;

// === FEEDFORWARD: PWM = FF_OFFSET + FF_SLOPE * |RPM| ===
const float FF_SLOPE = 0.3;
const float FF_OFFSET = 5.0;
const float FF_DEADBAND = 3.0;

// === TIMING ===
const int PID_INTERVAL = 10;
const int RPM_INTERVAL = 10;  // SYNCHRONIZED with PID for consistent derivative calculation
const int IMU_INTERVAL = 10;   // Separate IMU sampling rate
const int PRINT_INTERVAL = 200;

// === RATE LIMITER (prevents PWM jerk) ===
int lastPWM1 = 0, lastPWM2 = 0, lastPWM3 = 0;
const int PWM_RATE_LIMIT = 3;

// === RAMP SYSTEM ===
const float RAMP_UP_RATE = 5.0;
const float RAMP_DOWN_RATE = 3.0;
const float DEAD_ZONE_RPM = 2.0;
const float ZERO_THRESHOLD = 0.5;
const float EMA_ALPHA = 0.4;

// === STATE VARIABLES ===
float targetRPM1 = 0, targetRPM2 = 0, targetRPM3 = 0;
float actualTarget1 = 0, actualTarget2 = 0, actualTarget3 = 0;
float rawRPM1 = 0, rawRPM2 = 0, rawRPM3 = 0;
float currentRPM1 = 0, currentRPM2 = 0, currentRPM3 = 0;
long prevTicks1 = 0, prevTicks2 = 0, prevTicks3 = 0;
unsigned long prevMicros = 0;
unsigned long lastPIDTime = 0, lastRPMTime = 0, lastIMUTime = 0;

// Delta ticks
long lastSentTicks1 = 0,lastSentTicks2 = 0,lastSentTicks3 = 0;

// === FUNCTION DECLARATIONS ===
void calculateRPM();
void applyFilter();
void updateRampTargets();
void updatePID();
void setMotorPWM(int id, int pwm);
void forceStopMotors();
float computeFeedforward(float targetRPM);
int rateLimitPWM(int newPWM, int* lastPWM);
bool isNearZero(float value);
void sendFeedbackPacket();
void handleBinaryProtocol();
void updateIMU();

void setup() {
    Serial.begin(115200);
    Serial.setRxBufferSize(512);  // Prevent buffer overflow on burst transmission
    Serial.setTimeout(10);

    protocol.setMotorStopCallback(forceStopMotors);

    pinMode(M1_RPWM, OUTPUT); pinMode(M1_LPWM, OUTPUT); pinMode(M1_EN, OUTPUT);
    pinMode(M2_RPWM, OUTPUT); pinMode(M2_LPWM, OUTPUT); pinMode(M2_EN, OUTPUT);
    pinMode(M3_RPWM, OUTPUT); pinMode(M3_LPWM, OUTPUT); pinMode(M3_EN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder1.attachFullQuad(ENC1_A, ENC1_B);
    encoder2.attachFullQuad(ENC2_A, ENC2_B);
    encoder3.attachFullQuad(ENC3_A, ENC3_B);
    delay(100);
    encoder1.clearCount();
    encoder2.clearCount();
    encoder3.clearCount();

    pid1.setOutputLimits(-MAX_PWM, MAX_PWM);
    pid2.setOutputLimits(-MAX_PWM, MAX_PWM);
    pid3.setOutputLimits(-MAX_PWM, MAX_PWM);
    pid1.setIntegralLimits(50);
    pid2.setIntegralLimits(50);
    pid3.setIntegralLimits(50);

    digitalWrite(M1_EN, HIGH);
    digitalWrite(M2_EN, HIGH);
    digitalWrite(M3_EN, HIGH);

    for(int i=0; i<3; i++) {
        digitalWrite(LED_PIN, HIGH); delay(100);
        digitalWrite(LED_PIN, LOW); delay(100);
    }
    forceStopMotors();

    protocol.resetWatchdog();

    // === IMU SETUP (with timeout to prevent blocking) ===
    Wire.begin(SDA, SCL);
    Wire.setClock(400000);
    
    if (!mpu.begin(0x68, &Wire)) {
        imuReady = false;
    } else {
        imuReady = true;

        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

        delay(100);
    }
    
    // Binary mode only - no debug output
}

void loop() {
    static unsigned long lastModeBlink = 0;
    if (millis() - lastModeBlink > 1000) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        lastModeBlink = millis();
    }

    handleBinaryProtocol();

    if (millis() - lastRPMTime >= RPM_INTERVAL) {
        calculateRPM();
        applyFilter();
        lastRPMTime = millis();
    }

    if (millis() - lastPIDTime >= PID_INTERVAL) {
        updateRampTargets();
        
        if (!protocol.isCommHealthy()) {
            // Motors already stopped by watchdog callback
        } else {
            updatePID();
        }
        lastPIDTime = millis();
    }

    if (millis() - lastIMUTime >= IMU_INTERVAL){
        updateIMU();
        lastIMUTime = millis();
    }

    if (millis() - lastFeedbackTime >= FEEDBACK_INTERVAL) {
        sendFeedbackPacket();
        lastFeedbackTime = millis();
    }
}

void handleBinaryProtocol() {
    CommandPacket cmd = protocol.parseCommand();
    
    if (cmd.valid) {
        targetRPM1 = constrain(cmd.rpm1, -MAX_RPM, MAX_RPM);
        targetRPM2 = constrain(cmd.rpm2, -MAX_RPM, MAX_RPM);
        targetRPM3 = constrain(cmd.rpm3, -MAX_RPM, MAX_RPM);
        
        digitalWrite(LED_PIN, HIGH);
        delayMicroseconds(100);
        digitalWrite(LED_PIN, LOW);
    }
    
    if (protocol.checkWatchdog()) {
        static unsigned long lastBlink = 0;
        if (millis() - lastBlink > 100) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            lastBlink = millis();
        }
    }
}

void sendFeedbackPacket() {
    FeedbackPacket feedback;

    long currentTicks1 = INVERT_ENC1 ? -encoder1.getCount() : encoder1.getCount();
    long currentTicks2 = INVERT_ENC2 ? -encoder2.getCount() : encoder2.getCount();
    long currentTicks3 = INVERT_ENC3 ? -encoder3.getCount() : encoder3.getCount();

    feedback.tick1 = currentTicks1 - lastSentTicks1;
    feedback.tick2 = currentTicks2 - lastSentTicks2;
    feedback.tick3 = currentTicks3 - lastSentTicks3;
    
    lastSentTicks1 = currentTicks1;
    lastSentTicks2 = currentTicks2;
    lastSentTicks3 = currentTicks3;

    feedback.gyro_z = lastGoodGyroZ;
    feedback.accel_z = lastGoodAccelZ;
    
    protocol.sendFeedback(feedback);
}

void updateIMU() {
    if(!imuReady){
        return;
    }

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long currentTime = micros();
    float dt = (lastFilterTime == 0) ? 0.02 : (currentTime - lastFilterTime) / 1000000.0;
    lastFilterTime = currentTime;

    if(dt < 0.001 || dt > 0.1) {
        dt = 0.02;
    }

    float accelAngleX = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z));
    float accelAngleY = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z));

    filteredAngleX = FILTER_ALPHA * (filteredAngleX + g.gyro.x * dt) + (1.0 - FILTER_ALPHA) * accelAngleX;
    filteredAngleY = FILTER_ALPHA * (filteredAngleY + g.gyro.y * dt) + (1.0 - FILTER_ALPHA) * accelAngleY;
    filteredGyroZ = FILTER_ALPHA * filteredGyroZ + (1.0 - FILTER_ALPHA) * g.gyro.z;

    lastGoodGyroZ = (int16_t)(filteredGyroZ * 1000);
    lastGoodAccelZ = (int16_t)(filteredAngleX * 1000);
}

void calculateRPM() {
    unsigned long currentMicros = micros();

    if (prevMicros == 0) {
        prevMicros = currentMicros;
        prevTicks1 = encoder1.getCount();
        prevTicks2 = encoder2.getCount();
        prevTicks3 = encoder3.getCount();
        return;
    }

    float deltaT = (currentMicros - prevMicros) / 1000000.0;
    prevMicros = currentMicros;

    if (deltaT <= 0.005 || deltaT > 0.5) {
        prevTicks1 = encoder1.getCount();
        prevTicks2 = encoder2.getCount();
        prevTicks3 = encoder3.getCount();
        return;
    }

    long ticks1 = encoder1.getCount();
    long ticks2 = encoder2.getCount();
    long ticks3 = encoder3.getCount();

    long deltaTicks1 = ticks1 - prevTicks1;
    long deltaTicks2 = ticks2 - prevTicks2;
    long deltaTicks3 = ticks3 - prevTicks3;

    prevTicks1 = ticks1;
    prevTicks2 = ticks2;
    prevTicks3 = ticks3;

    float rpmMulti = 60.0 / (TICKS_PER_REV * deltaT);
    rawRPM1 = (float)deltaTicks1 * rpmMulti;
    rawRPM2 = (float)deltaTicks2 * rpmMulti;
    rawRPM3 = (float)deltaTicks3 * rpmMulti;
    
    if (INVERT_ENC1) rawRPM1 = -rawRPM1;
    if (INVERT_ENC2) rawRPM2 = -rawRPM2;
    if (INVERT_ENC3) rawRPM3 = -rawRPM3;
}

void applyFilter() {
    currentRPM1 = EMA_ALPHA * rawRPM1 + (1.0 - EMA_ALPHA) * currentRPM1;
    currentRPM2 = EMA_ALPHA * rawRPM2 + (1.0 - EMA_ALPHA) * currentRPM2;
    currentRPM3 = EMA_ALPHA * rawRPM3 + (1.0 - EMA_ALPHA) * currentRPM3;
}

void updatePID() {
    // Use threshold comparison (not == 0) for floating point safety
    bool allZero = isNearZero(actualTarget1) && isNearZero(actualTarget2) && isNearZero(actualTarget3);
    
    if (allZero) {
        setMotorPWM(1, 0);
        setMotorPWM(2, 0);
        setMotorPWM(3, 0);
        lastPWM1 = lastPWM2 = lastPWM3 = 0;
        pid1.reset(); pid2.reset(); pid3.reset();
        return;
    }
    
    if (abs(actualTarget1) < 1.0) pid1.reset();
    if (abs(actualTarget2) < 1.0) pid2.reset();
    if (abs(actualTarget3) < 1.0) pid3.reset();
    
    // PWM = Feedforward + PID trim (with rate limiting)
    float ff1 = computeFeedforward(actualTarget1);
    float trim1 = pid1.compute(rawRPM1, actualTarget1);
    int pwm1_raw = (int)constrain(ff1 + trim1, -MAX_PWM, MAX_PWM);
    int pwm1 = rateLimitPWM(pwm1_raw, &lastPWM1);
    
    float ff2 = computeFeedforward(actualTarget2);
    float trim2 = pid2.compute(rawRPM2, actualTarget2);
    int pwm2_raw = (int)constrain(ff2 + trim2, -MAX_PWM, MAX_PWM);
    int pwm2 = rateLimitPWM(pwm2_raw, &lastPWM2);
    
    float ff3 = computeFeedforward(actualTarget3);
    float trim3 = pid3.compute(rawRPM3, actualTarget3);
    int pwm3_raw = (int)constrain(ff3 + trim3, -MAX_PWM, MAX_PWM);
    int pwm3 = rateLimitPWM(pwm3_raw, &lastPWM3);
    
    setMotorPWM(1, pwm1);
    setMotorPWM(2, pwm2);
    setMotorPWM(3, pwm3);
}

int rateLimitPWM(int newPWM, int* lastPWM) {
    int delta = newPWM - *lastPWM;
    if (delta > PWM_RATE_LIMIT) delta = PWM_RATE_LIMIT;
    if (delta < -PWM_RATE_LIMIT) delta = -PWM_RATE_LIMIT;
    *lastPWM += delta;
    return *lastPWM;
}

void setMotorPWM(int id, int pwm) {
    int r_pin, l_pin;
    bool invert = false;

    if (id == 1) { r_pin = M1_RPWM; l_pin = M1_LPWM; invert = INVERT_PWM1; }
    else if (id == 2) { r_pin = M2_RPWM; l_pin = M2_LPWM; invert = INVERT_PWM2; }
    else { r_pin = M3_RPWM; l_pin = M3_LPWM; invert = INVERT_PWM3; }

    if (invert) pwm = -pwm;
    pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

    if (pwm > 0) { analogWrite(r_pin, pwm); analogWrite(l_pin, 0); }
    else if (pwm < 0) { analogWrite(r_pin, 0); analogWrite(l_pin, -pwm); }
    else { analogWrite(r_pin, 0); analogWrite(l_pin, 0); }
}

float computeFeedforward(float targetRPM) {
    if (abs(targetRPM) < FF_DEADBAND) return 0.0;
    float pwm = FF_OFFSET + FF_SLOPE * abs(targetRPM);
    if (targetRPM < 0) pwm = -pwm;
    return pwm;
}

bool isNearZero(float value) {
    return (abs(value) < ZERO_THRESHOLD);
}

void updateRampTargets() {
    // Motor 1
    if (actualTarget1 < targetRPM1) {
        actualTarget1 += RAMP_UP_RATE;
        if (actualTarget1 > targetRPM1) actualTarget1 = targetRPM1;
    } else if (actualTarget1 > targetRPM1) {
        actualTarget1 -= RAMP_DOWN_RATE;
        if (actualTarget1 < targetRPM1) actualTarget1 = targetRPM1;
    }
    // Snap to 0 if very small (prevents floating point residue)
    if (targetRPM1 == 0 && abs(actualTarget1) < RAMP_DOWN_RATE) actualTarget1 = 0;
    
    // Motor 2
    if (actualTarget2 < targetRPM2) {
        actualTarget2 += RAMP_UP_RATE;
        if (actualTarget2 > targetRPM2) actualTarget2 = targetRPM2;
    } else if (actualTarget2 > targetRPM2) {
        actualTarget2 -= RAMP_DOWN_RATE;
        if (actualTarget2 < targetRPM2) actualTarget2 = targetRPM2;
    }
    if (targetRPM2 == 0 && abs(actualTarget2) < RAMP_DOWN_RATE) actualTarget2 = 0;
    
    // Motor 3
    if (actualTarget3 < targetRPM3) {
        actualTarget3 += RAMP_UP_RATE;
        if (actualTarget3 > targetRPM3) actualTarget3 = targetRPM3;
    } else if (actualTarget3 > targetRPM3) {
        actualTarget3 -= RAMP_DOWN_RATE;
        if (actualTarget3 < targetRPM3) actualTarget3 = targetRPM3;
    }
    if (targetRPM3 == 0 && abs(actualTarget3) < RAMP_DOWN_RATE) actualTarget3 = 0;
}

void forceStopMotors() {
    analogWrite(M1_RPWM, 0); analogWrite(M1_LPWM, 0);
    analogWrite(M2_RPWM, 0); analogWrite(M2_LPWM, 0);
    analogWrite(M3_RPWM, 0); analogWrite(M3_LPWM, 0);
    
    targetRPM1 = targetRPM2 = targetRPM3 = 0;
    actualTarget1 = actualTarget2 = actualTarget3 = 0;
    rawRPM1 = rawRPM2 = rawRPM3 = 0;
    currentRPM1 = currentRPM2 = currentRPM3 = 0;
    lastPWM1 = lastPWM2 = lastPWM3 = 0;
    
    pid1.reset(); pid2.reset(); pid3.reset();
}