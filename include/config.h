#ifndef CONFIG_H
#define CONFIG_H

#define SDA_PIN 4
#define SCL_PIN 5

#define GPS_RX 20
#define GPS_TX 21
#define GPS_BAUD 9600

#define ROLL_SERVO_PIN 6
#define PITCH_SERVO_PIN 7
#define YAW_SERVO_PIN 10

#define LSM6DS3_ADDRESS 0x6B

#define ROLL_CENTER 90
#define PITCH_CENTER 90
#define YAW_CENTER 90

#define ROLL_MIN 70
#define ROLL_MAX 110

#define PITCH_MIN 70
#define PITCH_MAX 110

#define YAW_MIN 70
#define YAW_MAX 110

#define ROLL_DIRECTION 1
#define PITCH_DIRECTION 1
#define YAW_DIRECTION 1

#define CONTROL_HZ 200.0f
#define CONTROL_PERIOD_US 5000

#define ROLL_KP 2.0
#define ROLL_KI 0.0
#define ROLL_KD 0.15

#define PITCH_KP 2.0
#define PITCH_KI 0.0
#define PITCH_KD 0.15

#define YAW_KP 1.5
#define YAW_KI 0.0
#define YAW_KD 0.10

#define SERVO_LIMIT 20

#define TREMOR_LEARN_SECONDS 8
#define TREMOR_MIN_HZ 3.0f
#define TREMOR_MAX_HZ 12.0f
#define TREMOR_STEP_HZ 0.25f

#define NOTCH_Q 4.0f

#define INTENTIONAL_MOTION_CUTOFF_HZ 2.0f

#define BLE_DEVICE_NAME "SmartWallet"

#define BLE_SERVICE_UUID "7f8a0001-5a4b-4c2d-9e10-123456789001"
#define BLE_GPS_UUID "7f8a0002-5a4b-4c2d-9e10-123456789002"

#endif