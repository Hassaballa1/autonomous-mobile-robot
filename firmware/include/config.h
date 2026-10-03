// Hardware configuration for the AMR base controller (Arduino Nano).
// Everything that depends on the physical build lives here.
#pragma once

// ---------------------------------------------------------------- serial link
#define SERIAL_BAUD 115200
#define TELEMETRY_PERIOD_MS 20      // encoder report to the Pi, 50 Hz
#define BATTERY_PERIOD_MS 1000      // battery report, 1 Hz
#define CMD_TIMEOUT_MS 500          // stop the motors if the Pi goes quiet

// ---------------------------------------------------------------- drivetrain
// 24 V car wiper motors driving 15 cm wheels.
// Encoder ticks per wheel revolution as counted by this firmware (channel A, both edges).
// Calibrate: send "R", turn one wheel exactly one revolution by hand, read the E line.
#define TICKS_PER_REV 1200L

// Wiper motors run at roughly 60 rpm at 24 V: about 6.3 rad/s, 0.47 m/s with 15 cm wheels
#define MAX_WHEEL_SPEED 6.0f        // rad/s, commands above this are clamped
#define CONTROL_PERIOD_MS 20        // PID loop, 50 Hz

// Velocity PID per wheel, output in PWM counts (-255..255).
// The worm gearbox makes wiper motors slow to respond, so most of the work is feed-forward.
#define PID_KP 25.0f
#define PID_KI 80.0f
#define PID_KD 0.0f
#define PID_KF 40.0f                // feed-forward: PWM counts per rad/s (about 255 / top speed)

// Flip these if a wheel spins or counts the wrong way
#define LEFT_MOTOR_REVERSED false
#define RIGHT_MOTOR_REVERSED true
#define LEFT_ENCODER_REVERSED false
#define RIGHT_ENCODER_REVERSED true

// ---------------------------------------------------------------- power
// 24 V Li-ion pack (6S: 25.2 V full, 22.2 V nominal) through a 100k / 15k divider into A0.
// Replace the ratio with (R1 + R2) / R2 from your measured resistor values.
#define BATTERY_DIVIDER_RATIO 7.67f
#define BATTERY_LOW_VOLTS 19.8f     // 3.3 V per cell for 6S; use 23.1 for a 7S pack
#define ADC_MAX 1023.0f
#define ADC_VREF 5.0f

// ---------------------------------------------------------------- pins
// Cytron MD10C / MDD10A motor drivers: one DIR and one PWM input per motor.
#define LEFT_PWM 9
#define LEFT_DIR 8
#define RIGHT_PWM 10
#define RIGHT_DIR 12

// Encoder channel A must be on the Nano's two interrupt pins (D2, D3)
#define LEFT_ENC_A 2
#define LEFT_ENC_B 4
#define RIGHT_ENC_A 3
#define RIGHT_ENC_B 7

#define BATTERY_PIN A0
