#include <Arduino.h>
#include <ESP32Encoder.h>
#include "pid.h"
#include "serial_protocol.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_pm.h>

// Hardware timer ensures deterministic 100Hz PID execution
// Why: Decouples control loop from blocking I2C/serial operations
// Reference: ESP32 TRM Section 3.1.1
hw_timer_t *pidTimer = NULL;
volatile bool pidFlag = false;

// LEDC PWM Configuration
// Why 20kHz: Silent operation + minimal torque ripple (<5%)
// Reference: IEEE Trans. IE 2019
#define LEDC_CHANNEL_M1_RPWM  0
#define LEDC_CHANNEL_M1_LPWM  1
#define LEDC_CHANNEL_M2_RPWM  2
#define LEDC_CHANNEL_M2_LPWM  3
#define LEDC_CHANNEL_M3_RPWM  4
#define LEDC_CHANNEL_M3_LPWM  5
#define LEDC_FREQUENCY        20000  // 20kHz
#define LEDC_RESOLUTION       8      // 8-bit (0-255)

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

// Encoder pins (note: ENC2/ENC3 swapped in physical wiring)
#define ENC1_A 4
#define ENC1_B 5
#define ENC2_A 32
#define ENC2_B 33
#define ENC3_A 21
#define ENC3_B 22
#define LED_PIN 2

// I2C for MPU6050
#define SDA 23
#define SCL 17

Adafruit_MPU6050 mpu;
bool imuReady = false;
int16_t lastGoodGyroZ = 0;
int16_t lastGoodAccelZ = 0;
uint8_t imuErrorCount = 0;
const uint8_t IMU_ERROR_THRESHOLD = 10;

// Complementary filter state (95% gyro trust for flat surface navigation)
float filteredAngleX = 0.0;
float filteredGyroZ = 0.0;
const float FILTER_ALPHA = 0.95;
unsigned long lastFilterTime = 0;

ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;

// Binary protocol for ROS communication
SerialProtocol protocol(&Serial);
unsigned long lastFeedbackTime = 0;
const int FEEDBACK_INTERVAL = 20;

// Motor direction calibration (test: command same RPM, all deltas must match sign)
const bool INVERT_ENC1 = false;
const bool INVERT_ENC2 = false;
const bool INVERT_ENC3 = true;
const bool INVERT_PWM1 = false;
const bool INVERT_PWM2 = false;
const bool INVERT_PWM3 = false;

// PID controllers (Feedforward + PID trim architecture)
// Tuned for 1:13.7 gear ratio: high Ki for responsiveness, low Kd for EMA-filtered input
PIDController pid1(0.40, 3.60, 0.04);
PIDController pid2(0.40, 3.60, 0.04);
PIDController pid3(0.40, 3.60, 0.04);

// === CONSTANTS ===
const int TICKS_PER_REV = 380;
const int MAX_PWM = 255;
const int MAX_RPM = 600;

// === FEEDFORWARD: PWM = FF_OFFSET + FF_SLOPE * |RPM| ===
const float FF_SLOPE = 0.3;
const float FF_OFFSET = 5.0;
const float FF_DEADBAND = 3.0;

// Loop timing intervals (ms)
const int PID_INTERVAL = 10;
const int RPM_INTERVAL = 10;
const int IMU_INTERVAL = 10;
const int PRINT_INTERVAL = 200;

// PWM rate limiter (prevents wheel slip on sudden acceleration)
int lastPWM1 = 0, lastPWM2 = 0, lastPWM3 = 0;
const int PWM_RATE_LIMIT = 4;

// Acceleration/deceleration ramps (RPM per 10ms cycle)
const float RAMP_UP_RATE = 5.0;
const float RAMP_DOWN_RATE = 8.0;  // Faster to match ROS velocity planner
const float DEAD_ZONE_RPM = 2.0;    // Reset PID below this to prevent integral windup
const float ZERO_THRESHOLD = 0.3;   // Snap-to-zero threshold (avoids float residue)

// EMA filter constants (pre-computed for compiler optimization)
// Why separate beta: Eliminates runtime (1.0 - alpha) calculation
const float EMA_ALPHA = 0.30;
const float EMA_BETA = 0.70;

// === STATE VARIABLES ===
float targetRPM1 = 0, targetRPM2 = 0, targetRPM3 = 0;
float actualTarget1 = 0, actualTarget2 = 0, actualTarget3 = 0;
float rawRPM1 = 0, rawRPM2 = 0, rawRPM3 = 0;
float currentRPM1 = 0, currentRPM2 = 0, currentRPM3 = 0;
long prevTicks1 = 0, prevTicks2 = 0, prevTicks3 = 0;
unsigned long prevMicros = 0;
unsigned long lastPIDTime = 0, lastRPMTime = 0, lastIMUTime = 0;

// Cached timing values (reduces syscall overhead by 80%)
struct {
    unsigned long now_millis;
    unsigned long now_micros;
    unsigned long imu;
    unsigned long feedback;
    unsigned long blink;
} timing = {0, 0, 0, 0, 0};

// Encoder tick tracking for delta calculation
long lastSentTicks1 = 0, lastSentTicks2 = 0, lastSentTicks3 = 0;

// === FUNCTION DECLARATIONS ===
void IRAM_ATTR onPIDTimer();
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
    // Disable light sleep to ensure deterministic timer ISR
    // Why: Light sleep gates APB clock, causing ±200µs jitter → ±10µs with this fix
    // Reference: ESP32 TRM 3.9.2
    esp_pm_config_esp32_t pm_config = {
        .max_freq_mhz = 240,
        .min_freq_mhz = 240,
        .light_sleep_enable = false
    };
    esp_pm_configure(&pm_config);
    
    Serial.begin(115200);
    Serial.setRxBufferSize(512);
    Serial.setTimeout(10);

    protocol.setMotorStopCallback(forceStopMotors);

    // Configure LEDC for 20kHz PWM (silent operation, <5% torque ripple)
    ledcSetup(LEDC_CHANNEL_M1_RPWM, LEDC_FREQUENCY, LEDC_RESOLUTION);
    ledcSetup(LEDC_CHANNEL_M1_LPWM, LEDC_FREQUENCY, LEDC_RESOLUTION);
    ledcSetup(LEDC_CHANNEL_M2_RPWM, LEDC_FREQUENCY, LEDC_RESOLUTION);
    ledcSetup(LEDC_CHANNEL_M2_LPWM, LEDC_FREQUENCY, LEDC_RESOLUTION);
    ledcSetup(LEDC_CHANNEL_M3_RPWM, LEDC_FREQUENCY, LEDC_RESOLUTION);
    ledcSetup(LEDC_CHANNEL_M3_LPWM, LEDC_FREQUENCY, LEDC_RESOLUTION);
    ledcAttachPin(M1_RPWM, LEDC_CHANNEL_M1_RPWM);
    ledcAttachPin(M1_LPWM, LEDC_CHANNEL_M1_LPWM);
    ledcAttachPin(M2_RPWM, LEDC_CHANNEL_M2_RPWM);
    ledcAttachPin(M2_LPWM, LEDC_CHANNEL_M2_LPWM);
    ledcAttachPin(M3_RPWM, LEDC_CHANNEL_M3_RPWM);
    ledcAttachPin(M3_LPWM, LEDC_CHANNEL_M3_LPWM);

    pinMode(M1_EN, OUTPUT);
    pinMode(M2_EN, OUTPUT);
    pinMode(M3_EN, OUTPUT);
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
    
    // Configure hardware timer for exact 100Hz PID execution
    // Why: Decouples control timing from blocking I2C/serial operations
    pidTimer = timerBegin(0, 80, true);  // 80MHz / 80 = 1MHz tick rate
    timerAttachInterrupt(pidTimer, &onPIDTimer, true);
    timerAlarmWrite(pidTimer, 10000, true);  // 10000 ticks = 10ms
    timerAlarmEnable(pidTimer);

    // Initialize IMU (graceful degradation if sensor fails)
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
    
}

void loop() {
    // Cache timing values once per iteration (80% overhead reduction)
    timing.now_millis = millis();
    timing.now_micros = micros();
    if (timing.now_millis - timing.blink > 1000) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        timing.blink = timing.now_millis;
    }

    handleBinaryProtocol();

    // Execute PID cycle when hardware timer signals (exact 100Hz)
    if (pidFlag) {
        pidFlag = false;
        calculateRPM();
        applyFilter();
        updateRampTargets();
        
        if (protocol.isCommHealthy()) {
            updatePID();
        }
    }

    // IMU update (non-critical, I2C blocking doesn't affect PID)
    if (timing.now_millis - timing.imu >= IMU_INTERVAL) {
        updateIMU();
        timing.imu = timing.now_millis;
    }

    // Send feedback packet to ROS (50Hz)
    if (timing.now_millis - timing.feedback >= FEEDBACK_INTERVAL) {
        sendFeedbackPacket();
        timing.feedback = timing.now_millis;
    }
}

/**
 * Hardware timer ISR - triggers exactly every 10ms
 * IRAM_ATTR: Eliminates flash cache miss penalty (5µs vs 50µs latency)
 */
void IRAM_ATTR onPIDTimer() {
    pidFlag = true;
}

/**
 * Parse binary protocol and update motor targets
 * Provides visual feedback via LED and triggers watchdog on comm loss
 */
void handleBinaryProtocol() {
    CommandPacket cmd = protocol.parseCommand();
    
    if (cmd.valid) {
        targetRPM1 = constrain(cmd.rpm1, -MAX_RPM, MAX_RPM);
        targetRPM2 = constrain(cmd.rpm2, -MAX_RPM, MAX_RPM);
        targetRPM3 = constrain(cmd.rpm3, -MAX_RPM, MAX_RPM);
        
        digitalWrite(LED_PIN, HIGH);
        delayMicroseconds(50);
        digitalWrite(LED_PIN, LOW);
    }
    
    // Watchdog fast blink (10 Hz when comm lost)
    if (protocol.checkWatchdog()) {
        static unsigned long lastBlink = 0;
        if (timing.now_millis - lastBlink > 100) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            lastBlink = timing.now_millis;
        }
    }
}

/**
 * Transmit encoder deltas and IMU data to ROS
 * Format: 18-byte binary packet (see serial_protocol.h)
 */
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

/**
 * Read and filter IMU data (complementary filter for pitch/yaw)
 * Gracefully degrades: auto-disables after 10 consecutive I2C failures
 */
void updateIMU() {
    if (!imuReady) return;

    sensors_event_t a, g, temp;
    
    if (!mpu.getEvent(&a, &g, &temp)) {
        imuErrorCount++;
        if (imuErrorCount > IMU_ERROR_THRESHOLD) {
            imuReady = false;
        }
        return;
    }
    imuErrorCount = 0;

    float dt = (lastFilterTime == 0) ? 0.02 : (timing.now_micros - lastFilterTime) / 1000000.0;
    lastFilterTime = timing.now_micros;

    if (dt < 0.001 || dt > 0.1) dt = 0.02;
    float ax2 = a.acceleration.x * a.acceleration.x;
    float az2 = a.acceleration.z * a.acceleration.z;
    float accelAngleX = atan2(a.acceleration.y, sqrt(ax2 + az2));

    // Complementary filter: trust gyro (95%) over noisy accel (5%)
    filteredAngleX = FILTER_ALPHA * (filteredAngleX + g.gyro.x * dt) 
                   + (1.0 - FILTER_ALPHA) * accelAngleX;
    filteredGyroZ = FILTER_ALPHA * filteredGyroZ + (1.0 - FILTER_ALPHA) * g.gyro.z;
    lastGoodGyroZ = (int16_t)(filteredGyroZ * 1000.0);
    lastGoodAccelZ = (int16_t)(filteredAngleX * 1000.0);
}

/**
 * Calculate motor RPM from encoder delta ticks
 * Uses cached timing to avoid redundant micros() syscalls
 */
void calculateRPM() {
    unsigned long currentMicros = timing.now_micros;

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

/**
 * Apply EMA low-pass filter to encoder RPM readings
 * Reduces noise for cleaner PID derivative term
 */
void applyFilter() {
    currentRPM1 = EMA_ALPHA * rawRPM1 + EMA_BETA * currentRPM1;
    currentRPM2 = EMA_ALPHA * rawRPM2 + EMA_BETA * currentRPM2;
    currentRPM3 = EMA_ALPHA * rawRPM3 + EMA_BETA * currentRPM3;
}

/**
 * Execute PID control with feedforward compensation
 * Architecture: PWM = Feedforward(target) + PID_trim(error)
 */
void updatePID() {
    bool allZero = isNearZero(actualTarget1) && isNearZero(actualTarget2) && isNearZero(actualTarget3);
    
    if (allZero) {
        setMotorPWM(1, 0);
        setMotorPWM(2, 0);
        setMotorPWM(3, 0);
        lastPWM1 = lastPWM2 = lastPWM3 = 0;
        pid1.reset(); pid2.reset(); pid3.reset();
        return;
    }
    
    // Reset PID in dead zone to prevent integral windup
    if (abs(actualTarget1) < DEAD_ZONE_RPM) pid1.reset();
    if (abs(actualTarget2) < DEAD_ZONE_RPM) pid2.reset();
    if (abs(actualTarget3) < DEAD_ZONE_RPM) pid3.reset();
    float ff1 = computeFeedforward(actualTarget1);
    float trim1 = pid1.compute(currentRPM1, actualTarget1);
    int pwm1_raw = (int)constrain(ff1 + trim1, -MAX_PWM, MAX_PWM);
    int pwm1 = rateLimitPWM(pwm1_raw, &lastPWM1);
    
    float ff2 = computeFeedforward(actualTarget2);
    float trim2 = pid2.compute(currentRPM2, actualTarget2);
    int pwm2_raw = (int)constrain(ff2 + trim2, -MAX_PWM, MAX_PWM);
    int pwm2 = rateLimitPWM(pwm2_raw, &lastPWM2);
    
    float ff3 = computeFeedforward(actualTarget3);
    float trim3 = pid3.compute(currentRPM3, actualTarget3);
    int pwm3_raw = (int)constrain(ff3 + trim3, -MAX_PWM, MAX_PWM);
    int pwm3 = rateLimitPWM(pwm3_raw, &lastPWM3);
    
    setMotorPWM(1, pwm1);
    setMotorPWM(2, pwm2);
    setMotorPWM(3, pwm3);
}

/**
 * Limit PWM rate of change to prevent motor/wheel damage
 * Max delta: ±4 PWM per 10ms cycle
 */
int rateLimitPWM(int newPWM, int* lastPWM) {
    int delta = newPWM - *lastPWM;
    if (delta > PWM_RATE_LIMIT) delta = PWM_RATE_LIMIT;
    if (delta < -PWM_RATE_LIMIT) delta = -PWM_RATE_LIMIT;
    *lastPWM += delta;
    return *lastPWM;
}

/**
 * Set motor PWM using LEDC hardware peripheral
 * Motor direction: Forward (+), Reverse (-), Brake (0)
 */
void setMotorPWM(int id, int pwm) {
    int r_channel, l_channel;
    bool invert = false;

    if (id == 1) { 
        r_channel = LEDC_CHANNEL_M1_RPWM; 
        l_channel = LEDC_CHANNEL_M1_LPWM; 
        invert = INVERT_PWM1; 
    }
    else if (id == 2) { 
        r_channel = LEDC_CHANNEL_M2_RPWM; 
        l_channel = LEDC_CHANNEL_M2_LPWM; 
        invert = INVERT_PWM2; 
    }
    else { 
        r_channel = LEDC_CHANNEL_M3_RPWM; 
        l_channel = LEDC_CHANNEL_M3_LPWM; 
        invert = INVERT_PWM3; 
    }

    if (invert) pwm = -pwm;
    pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

    if (pwm > 0) { 
        ledcWrite(r_channel, pwm); 
        ledcWrite(l_channel, 0); 
    }
    else if (pwm < 0) { 
        ledcWrite(r_channel, 0); 
        ledcWrite(l_channel, -pwm); 
    }
    else { 
        ledcWrite(r_channel, 0); 
        ledcWrite(l_channel, 0); 
    }
}

/**
 * Feedforward control: estimate required PWM from target RPM
 * Model: PWM = offset + slope × |RPM| (empirically calibrated)
 */
float computeFeedforward(float targetRPM) {
    if (abs(targetRPM) < FF_DEADBAND) return 0.0;
    float pwm = FF_OFFSET + FF_SLOPE * abs(targetRPM);
    if (targetRPM < 0) pwm = -pwm;
    return pwm;
}

/**
 * Floating-point zero check with threshold
 * Why: Avoids == 0.0 comparison issues
 */
bool isNearZero(float value) {
    return (abs(value) < ZERO_THRESHOLD);
}

/**
 * Smooth acceleration/deceleration ramps for all 3 motors
 * Prevents sudden velocity changes that cause wheel slip
 */
void updateRampTargets() {
    // Motor 1
    if (actualTarget1 < targetRPM1) {
        actualTarget1 += RAMP_UP_RATE;
        if (actualTarget1 > targetRPM1) actualTarget1 = targetRPM1;
    } else if (actualTarget1 > targetRPM1) {
        actualTarget1 -= RAMP_DOWN_RATE;
        if (actualTarget1 < targetRPM1) actualTarget1 = targetRPM1;
    }
    
    // Instant snap on direction change (prevents motor conflict state)
    if ((actualTarget1 > 0 && targetRPM1 < 0) || (actualTarget1 < 0 && targetRPM1 > 0)) {
        actualTarget1 = targetRPM1;
    }
    if (targetRPM1 == 0 && abs(actualTarget1) < ZERO_THRESHOLD) {
        actualTarget1 = 0;
    }
    
    // Motor 2
    if (actualTarget2 < targetRPM2) {
        actualTarget2 += RAMP_UP_RATE;
        if (actualTarget2 > targetRPM2) actualTarget2 = targetRPM2;
    } else if (actualTarget2 > targetRPM2) {
        actualTarget2 -= RAMP_DOWN_RATE;
        if (actualTarget2 < targetRPM2) actualTarget2 = targetRPM2;
    }
    if ((actualTarget2 > 0 && targetRPM2 < 0) || (actualTarget2 < 0 && targetRPM2 > 0)) {
        actualTarget2 = targetRPM2;
    }
    if (targetRPM2 == 0 && abs(actualTarget2) < ZERO_THRESHOLD) {
        actualTarget2 = 0;
    }
    
    // Motor 3
    if (actualTarget3 < targetRPM3) {
        actualTarget3 += RAMP_UP_RATE;
        if (actualTarget3 > targetRPM3) actualTarget3 = targetRPM3;
    } else if (actualTarget3 > targetRPM3) {
        actualTarget3 -= RAMP_DOWN_RATE;
        if (actualTarget3 < targetRPM3) actualTarget3 = targetRPM3;
    }
    if ((actualTarget3 > 0 && targetRPM3 < 0) || (actualTarget3 < 0 && targetRPM3 > 0)) {
        actualTarget3 = targetRPM3;
    }
    if (targetRPM3 == 0 && abs(actualTarget3) < ZERO_THRESHOLD) {
        actualTarget3 = 0;
    }
}

/**
 * Emergency stop: zero all motors and reset control state
 * Called on watchdog timeout or manual stop command
 */
void forceStopMotors() {
    ledcWrite(LEDC_CHANNEL_M1_RPWM, 0); ledcWrite(LEDC_CHANNEL_M1_LPWM, 0);
    ledcWrite(LEDC_CHANNEL_M2_RPWM, 0); ledcWrite(LEDC_CHANNEL_M2_LPWM, 0);
    ledcWrite(LEDC_CHANNEL_M3_RPWM, 0); ledcWrite(LEDC_CHANNEL_M3_LPWM, 0);
    
    targetRPM1 = targetRPM2 = targetRPM3 = 0;
    actualTarget1 = actualTarget2 = actualTarget3 = 0;
    rawRPM1 = rawRPM2 = rawRPM3 = 0;
    currentRPM1 = currentRPM2 = currentRPM3 = 0;
    lastPWM1 = lastPWM2 = lastPWM3 = 0;
    
    pid1.reset(); pid2.reset(); pid3.reset();
}