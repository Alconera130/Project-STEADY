#include <Arduino.h>
#include <Wire.h>
#include <SparkFunLSM6DS3.h>
#include <MadgwickAHRS.h>
#include <PID_v1.h>
#include <ESP32Servo.h>

#include "config.h"
#include "stabilizer.h"
#include "tremorFilter.h"

LSM6DS3 imu(I2C_MODE, LSM6DS3_ADDRESS);

Servo rollServo;
Servo pitchServo;
Servo yawServo;

Madgwick filter;

double rollInput = 0;
double rollOutput = 0;
double rollSetpoint = 0;

double pitchInput = 0;
double pitchOutput = 0;
double pitchSetpoint = 0;

double yawInput = 0;
double yawOutput = 0;
double yawSetpoint = 0;

PID rollPID(&rollInput, &rollOutput, &rollSetpoint, ROLL_KP, ROLL_KI, ROLL_KD, DIRECT);
PID pitchPID( &pitchInput, &pitchOutput, &pitchSetpoint, PITCH_KP, PITCH_KI, PITCH_KD, DIRECT);

PID yawPID(&yawInput, &yawOutput, &yawSetpoint, YAW_KP, YAW_KI, YAW_KD, DIRECT);

float gyroOffsetX = 0;
float gyroOffsetY = 0;
float gyroOffsetZ = 0;

float yawAngle = 0;

unsigned long lastControlMicros = 0;

void calibrateGyro() {
    const int samples = 500;

    float sx = 0;
    float sy = 0;
    float sz = 0;

    Serial.println("Keep IMU still.");

    for (int i = 0; i < samples; i++) {
        sx += imu.readFloatGyroX();
        sy += imu.readFloatGyroY();
        sz += imu.readFloatGyroZ();

        delay(2);
    }

    gyroOffsetX = sx / samples;
    gyroOffsetY = sy / samples;
    gyroOffsetZ = sz / samples;
}

void setupServos() {
    rollServo.setPeriodHertz(50);
    pitchServo.setPeriodHertz(50);
    yawServo.setPeriodHertz(50);

    rollServo.attach(ROLL_SERVO_PIN, 500, 2500);
    pitchServo.attach(PITCH_SERVO_PIN, 500, 2500);
    yawServo.attach(YAW_SERVO_PIN, 500, 2500);

    rollServo.write(ROLL_CENTER);
    pitchServo.write(PITCH_CENTER);
    yawServo.write(YAW_CENTER);
}

void setupPID() {
    rollPID.SetOutputLimits(-SERVO_LIMIT, SERVO_LIMIT);
    pitchPID.SetOutputLimits(-SERVO_LIMIT, SERVO_LIMIT);
    yawPID.SetOutputLimits(-SERVO_LIMIT, SERVO_LIMIT);

    rollPID.SetSampleTime(5);
    pitchPID.SetSampleTime(5);
    yawPID.SetSampleTime(5);

    rollPID.SetMode(AUTOMATIC);
    pitchPID.SetMode(AUTOMATIC);
    yawPID.SetMode(AUTOMATIC);
}

void stabilizerBegin() {
    Wire.begin(SDA_PIN, SCL_PIN);

    if (imu.begin() != 0) {
        Serial.println("LSM6DS3 ERROR");

        while (true) { delay(1000); }
    }

    setupServos();
    calibrateGyro();
    filter.begin(CONTROL_HZ);
    tremorBegin();

    lastControlMicros = micros();

    Serial.println("STABILIZER READY");
}

void updateServos() {
    int rollPosition =ROLL_CENTER + ROLL_DIRECTION * (int)rollOutput;
    int pitchPosition = PITCH_CENTER + PITCH_DIRECTION * (int)pitchOutput;
    int yawPosition = YAW_CENTER + YAW_DIRECTION * (int)yawOutput;

    rollPosition = constrain(rollPosition, ROLL_MIN, ROLL_MAX);
    pitchPosition = constrain(pitchPosition, PITCH_MIN, PITCH_MAX);
    yawPosition = constrain(yawPosition, YAW_MIN, YAW_MAX);

    rollServo.write(rollPosition);
    pitchServo.write(pitchPosition);
    yawServo.write(yawPosition);
}

void stabilizerUpdate() {
    unsigned long now = micros();

    if (now - lastControlMicros < CONTROL_PERIOD_US) {
        return;
    }

    float dt = (now - lastControlMicros) / 1000000.0f;

    lastControlMicros = now;

    float ax = imu.readFloatAccelX();
    float ay = imu.readFloatAccelY();
    float az = imu.readFloatAccelZ();

    float gx = imu.readFloatGyroX() - gyroOffsetX;
    float gy = imu.readFloatGyroY() - gyroOffsetY;
    float gz = imu.readFloatGyroZ() - gyroOffsetZ;

    if (!tremorReady()) {
        tremorRecord(gx, gy, gz);

        return;
    }

    tremorUpdate(gx, gy, gz);

    float filteredGX = getFilteredGX();
    float filteredGY = getFilteredGY();
    float filteredGZ = getFilteredGZ();

    filter.updateIMU(filteredGX, filteredGY, filteredGZ, ax, ay, az);

    float roll = filter.getRoll();

    float pitch = filter.getPitch();

    yawAngle += filteredGZ * dt;

    if (yawAngle > 180) yawAngle -= 360;
    if (yawAngle < -180) yawAngle += 360;

    rollInput =roll;
    pitchInput = pitch;
    yawInput = yawAngle;

    rollPID.Compute();
    pitchPID.Compute();
    yawPID.Compute();

    updateServos();
}

void stabilizerPrint() {
    static unsigned long lastPrint = 0;

    if (millis() - lastPrint < 500) return;

    lastPrint = millis();

    Serial.print("TREMOR: ");
    Serial.print(getTremorFrequency(), 2);
    Serial.print("Hz | R:");
    Serial.print(rollInput, 2);
    Serial.print(" P:");
    Serial.print(pitchInput, 2);
    Serial.print(" Y:");
    Serial.println(yawInput, 2);
}