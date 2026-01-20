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
int16_t lastGoodGyroZ = 0;   // Cache last good IMU values
int16_t lastGoodAccelZ = 981; // Default ~9.81 m/s² * 100
uint8_t imuErrorCount = 0;    // Track consecutive errors

ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;

// === PHASE 2: BINARY PROTOCOL ===
SerialProtocol protocol(&Serial);
bool binaryMode = true;  // Start in binary mode for Raspi communication
unsigned long lastFeedbackTime = 0;
const int FEEDBACK_INTERVAL = 50;

// === DIRECTION CONFIG ===
const bool INVERT_ENC1 = true;
const bool INVERT_ENC2 = true;
const bool INVERT_ENC3 = true;
const bool INVERT_PWM1 = false;
const bool INVERT_PWM2 = false;
const bool INVERT_PWM3 = false;

// === PID (Feedforward + Trim architecture) ===
PIDController pid1(0.3, 0.05, 0.005);
PIDController pid2(0.3, 0.05, 0.005);
PIDController pid3(0.3, 0.05, 0.005);

// === CONSTANTS ===
const int TICKS_PER_REV = 380;
const int MAX_PWM = 255;
const int MAX_RPM = 600;

// === FEEDFORWARD: PWM = FF_OFFSET + FF_SLOPE * |RPM| ===
const float FF_SLOPE = 0.3;
const float FF_OFFSET = 5.0;
const float FF_DEADBAND = 3.0;

// === TIMING ===
const int PID_INTERVAL = 20;
const int RPM_INTERVAL = 50;
const int PRINT_INTERVAL = 200;

// === RATE LIMITER (prevents PWM jerk) ===
int lastPWM1 = 0, lastPWM2 = 0, lastPWM3 = 0;
const int PWM_RATE_LIMIT = 8;

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
unsigned long lastPIDTime = 0, lastRPMTime = 0, lastPrintTime = 0;
bool testModeActive = false;
unsigned long testModeStartTime = 0;
const unsigned long TEST_DURATION = 5000;

// === FUNCTION DECLARATIONS ===
void parseAndDrive(String data);
void calculateRPM();
void applyFilter();
void updateRampTargets();
void updatePID();
void setMotorPWM(int id, int pwm);
void printStatus();
void forceStopMotors();
float computeFeedforward(float targetRPM);
int rateLimitPWM(int newPWM, int* lastPWM);
bool isNearZero(float value);
float applyDeadZone(float rpm);
void sendFeedbackPacket();
void handleBinaryProtocol();
void handleDebugMode();

void setup() {
    Serial.begin(115200);
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
    pid1.setIntegralLimits(10);
    pid2.setIntegralLimits(10);
    pid3.setIntegralLimits(10);

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
    Wire.setClock(100000);  // 100kHz I2C (slower = more stable)
    Wire.setTimeOut(50);    // 50ms I2C timeout
    
    // Try to init MPU6050 with timeout protection
    unsigned long imuStartTime = millis();
    while (millis() - imuStartTime < 500) {  // Max 500ms untuk init
        if (mpu.begin(0x68, &Wire)) {
            imuReady = true;
            mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
            mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
            mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
            break;
        }
        delay(50);
    }

    // Binary mode: NO text output (akan corrupt binary stream)
    // Untuk debug, ketik 'd' di Serial Monitor untuk switch ke debug mode
    
    // Only print welcome if NOT in binary mode
    if (!binaryMode) {
        Serial.println();
        Serial.println("=== ESP32 MOTOR CONTROLLER (Phase 2) ===");
        Serial.print("IMU Status: ");
        Serial.println(imuReady ? "READY" : "NOT READY");
        Serial.println("Commands: r=reset, s=stop, w=test300, 1/2/3=openloop, b=binary, i=imu");
    }
}

void loop() {
    if (binaryMode) {
        handleBinaryProtocol();
    } else {
        handleDebugMode();
    }

    if (millis() - lastRPMTime >= RPM_INTERVAL) {
        calculateRPM();
        applyFilter();
        lastRPMTime = millis();
    }

    if (millis() - lastPIDTime >= PID_INTERVAL) {
        updateRampTargets();
        
        if (binaryMode && !protocol.isCommHealthy()) {
            // Motors already stopped by watchdog callback
        } else {
            updatePID();
        }
        lastPIDTime = millis();
    }

    if (binaryMode && millis() - lastFeedbackTime >= FEEDBACK_INTERVAL) {
        sendFeedbackPacket();
        lastFeedbackTime = millis();
    }

    if (!binaryMode && millis() - lastPrintTime >= PRINT_INTERVAL) {
        printStatus();
        lastPrintTime = millis();
    }
}

void handleBinaryProtocol() {
    // Check for debug mode switch ('d' or 'D' character)
    while (Serial.available() > 0) {
        uint8_t byte = Serial.peek();
        
        // If it's 'd' or 'D', switch to debug mode
        if (byte == 'd' || byte == 'D') {
            Serial.read();  // Consume the byte
            binaryMode = false;
            forceStopMotors();
            Serial.println(">>> DEBUG MODE <<<");
            Serial.println("Commands: r=reset, s=stop, w=test300, 1/2/3=openloop, b=binary, i=imu");
            return;
        }
        break;  // Not a mode switch, let parseCommand handle it
    }
    
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
    
    feedback.tick1 = INVERT_ENC1 ? -encoder1.getCount() : encoder1.getCount();
    feedback.tick2 = INVERT_ENC2 ? -encoder2.getCount() : encoder2.getCount();
    feedback.tick3 = INVERT_ENC3 ? -encoder3.getCount() : encoder3.getCount();
    
    // Read IMU data with robust error handling
    if (imuReady) {
        sensors_event_t a, g, temp;
        if (mpu.getEvent(&a, &g, &temp)) {
            lastGoodGyroZ = (int16_t)(g.gyro.z * 1000);
            lastGoodAccelZ = (int16_t)(a.acceleration.z * 100);
            imuErrorCount = 0;
        } else {
            imuErrorCount++;
            if (imuErrorCount == 10) {
                Wire.end();
                delay(10);
                Wire.begin(SDA, SCL);
                Wire.setClock(100000);
                Wire.setTimeOut(50);
            }
        }
        
        feedback.gyro_z = lastGoodGyroZ;
        feedback.accel_z = lastGoodAccelZ;
    } else {
        feedback.gyro_z = lastGoodGyroZ;
        feedback.accel_z = lastGoodAccelZ;
    }
    
    protocol.sendFeedback(feedback);
}

void handleDebugMode() {
    if (testModeActive && (millis() - testModeStartTime >= TEST_DURATION)) {
        testModeActive = false;
        targetRPM1 = targetRPM2 = targetRPM3 = 0;
        Serial.println(">>> TEST SELESAI <<<");
    }

    if (Serial.available() > 0) {
        String data = "";
        while (Serial.available() > 0) {
            data = Serial.readStringUntil('\n');
            data.trim();
        }
        
        digitalWrite(LED_PIN, HIGH);
        
        if (data == "b" || data == "B") {
            binaryMode = true;
            protocol.resetWatchdog();
            // No print - akan corrupt binary stream
        }
        else if (data == "d" || data == "D") {
            binaryMode = false;
            Serial.println(">>> DEBUG MODE <<<");
            Serial.println("Commands: r=reset, s=stop, w=test300, 1/2/3=openloop, b=binary, i=imu");
        }
        else if (data == "i" || data == "I") {
            Serial.print("IMU Status: ");
            Serial.println(imuReady ? "READY" : "NOT READY");
            if (imuReady) {
                sensors_event_t a, g, temp;
                mpu.getEvent(&a, &g, &temp);
                Serial.print("Gyro Z: "); Serial.print(g.gyro.z, 4); Serial.println(" rad/s");
                Serial.print("Accel Z: "); Serial.print(a.acceleration.z, 2); Serial.println(" m/s^2");
            }
        }
        else if (data == "r" || data == "R") {
            encoder1.clearCount(); encoder2.clearCount(); encoder3.clearCount();
            pid1.reset(); pid2.reset(); pid3.reset();
            prevTicks1 = prevTicks2 = prevTicks3 = 0;
            testModeActive = false;
            forceStopMotors();
            Serial.println(">>> RESET <<<");
        } 
        else if (data == "s" || data == "S") {
            testModeActive = false;
            forceStopMotors();
            Serial.println(">>> STOP <<<");
        }
        else if (data == "w" || data == "W") {
            targetRPM1 = targetRPM2 = targetRPM3 = 300;
            testModeActive = true;
            testModeStartTime = millis();
            Serial.println(">>> TEST 300 RPM <<<");
        }
        else if (data == "1") {
            forceStopMotors();
            encoder1.clearCount(); encoder2.clearCount(); encoder3.clearCount();
            prevTicks1 = prevTicks2 = prevTicks3 = 0;
            prevMicros = 0;
            Serial.println(">>> M1 OPEN LOOP (PWM=80) <<<");
            analogWrite(M1_RPWM, 80); analogWrite(M1_LPWM, 0);
            delay(1500);
            Serial.print("ENC: "); Serial.print(encoder1.getCount());
            Serial.print(","); Serial.print(encoder2.getCount());
            Serial.print(","); Serial.println(encoder3.getCount());
            forceStopMotors();
        }
        else if (data == "2") {
            forceStopMotors();
            encoder1.clearCount(); encoder2.clearCount(); encoder3.clearCount();
            prevTicks1 = prevTicks2 = prevTicks3 = 0;
            prevMicros = 0;
            Serial.println(">>> M2 OPEN LOOP (PWM=80) <<<");
            analogWrite(M2_RPWM, 80); analogWrite(M2_LPWM, 0);
            delay(1500);
            Serial.print("ENC: "); Serial.print(encoder1.getCount());
            Serial.print(","); Serial.print(encoder2.getCount());
            Serial.print(","); Serial.println(encoder3.getCount());
            forceStopMotors();
        }
        else if (data == "3") {
            forceStopMotors();
            encoder1.clearCount(); encoder2.clearCount(); encoder3.clearCount();
            prevTicks1 = prevTicks2 = prevTicks3 = 0;
            prevMicros = 0;
            Serial.println(">>> M3 OPEN LOOP (PWM=80) <<<");
            analogWrite(M3_RPWM, 80); analogWrite(M3_LPWM, 0);
            delay(1500);
            Serial.print("ENC: "); Serial.print(encoder1.getCount());
            Serial.print(","); Serial.print(encoder2.getCount());
            Serial.print(","); Serial.println(encoder3.getCount());
            forceStopMotors();
        }
        else if (data.length() > 0) {
            parseAndDrive(data);
        }
        
        digitalWrite(LED_PIN, LOW);
    }
}

void parseAndDrive(String data) {
    int comma1 = data.indexOf(',');
    int comma2 = data.indexOf(',', comma1 + 1);

    if (comma1 > 0 && comma2 > 0) {
        float newRPM1 = applyDeadZone(data.substring(0, comma1).toFloat());
        float newRPM2 = applyDeadZone(data.substring(comma1 + 1, comma2).toFloat());
        float newRPM3 = applyDeadZone(data.substring(comma2 + 1).toFloat());

        bool changed = (abs(newRPM1 - targetRPM1) > 1.0) || 
                       (abs(newRPM2 - targetRPM2) > 1.0) || 
                       (abs(newRPM3 - targetRPM3) > 1.0);

        targetRPM1 = newRPM1;
        targetRPM2 = newRPM2;
        targetRPM3 = newRPM3;

        if (changed) {
            Serial.print("TARGET: ");
            Serial.print(targetRPM1); Serial.print(",");
            Serial.print(targetRPM2); Serial.print(",");
            Serial.println(targetRPM3);
        }
    }
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

    if (deltaT <= 0.01 || deltaT > 0.5) {
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

void printStatus() {
    Serial.print("RAW: ");
    Serial.print(rawRPM1, 1); Serial.print(",");
    Serial.print(rawRPM2, 1); Serial.print(",");
    Serial.print(rawRPM3, 1);
    
    Serial.print(" | PWM: ");
    Serial.print(lastPWM1); Serial.print(",");
    Serial.print(lastPWM2); Serial.print(",");
    Serial.print(lastPWM3);

    Serial.print(" | TGT: ");
    Serial.print(actualTarget1, 1); Serial.print(",");
    Serial.print(actualTarget2, 1); Serial.print(",");
    Serial.print(actualTarget3, 1);
    
    if (isNearZero(actualTarget1) && isNearZero(actualTarget2) && isNearZero(actualTarget3)) {
        Serial.println(" [STOP]");
    } else {
        Serial.println();
    }
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

float applyDeadZone(float rpm) {
    return (abs(rpm) < DEAD_ZONE_RPM) ? 0.0 : rpm;
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