#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "zf_common_headfile.h"
#include "zf_device_key.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6050.h"
#include "servo.h"
#include "menu.h"
#include "Mymenu.h"
#include "vision.h"
#include "control.h"

// === R79 baseline + is_straight() LPF ===

#define DEFAULT_ASC_FAR        30
#define ADAPT_ERR_TH1     10
#define ADAPT_ERR_TH2     20
#define ADAPT_OFF_STR    (-10)
#define ADAPT_OFF_SML      5
#define ADAPT_OFF_BIG     20

// --- Servo ---
#define DEFAULT_SERVO_KP1      1.00f
#define DEFAULT_SERVO_KP2      0.004f
#define DEFAULT_SERVO_KD       0.24f
#define DEFAULT_GYRO_KD        0.09f
#define DEFAULT_SERVO_CENTER   95.2f
#define DEFAULT_SERVO_MAX_CHA  17.0f
#define DEFAULT_SERVO_DEAD     2.0f
#define DEFAULT_SERVO_DIR      1

// --- Motor ---
#define DEFAULT_MOTOR_BASE       15    // 150 straight
#define DEFAULT_MOTOR_CURVE_DUTY 15    // 150 corner = base
#define DEFAULT_MOTOR_MAX        80
#define DEFAULT_ACKERMANN_GAIN  0.0f

// --- Speed PID ---
#define DEFAULT_SPEED_KP_L       0.7f
#define DEFAULT_SPEED_KI_L       0.15f
#define DEFAULT_SPEED_KD_L       0.10f
#define DEFAULT_SPEED_KP_R       0.8f
#define DEFAULT_SPEED_KI_R       0.15f
#define DEFAULT_SPEED_KD_R       0.10f
#define DEFAULT_SPEED_DELTA_MAX  6.0f

// --- Angle ---
#define DEFAULT_ANGLE_KP_A          0.0f
#define DEFAULT_ANGLE_KP_B          0.0f
#define DEFAULT_ANGLE_KD            0.02f
#define DEFAULT_ANGLE_LOWPASS       0.8f
#define DEFAULT_SERVO_FUSION_ALPHA  0.0f
#endif