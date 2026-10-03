// AMR base controller: closed-loop wheel velocity control and odometry ticks
// over a line-based serial protocol to the Raspberry Pi (see firmware/README.md).
#include <Arduino.h>

#include "config.h"

static const char *FIRMWARE_ID = "amr-base 1.1";

// ---------------------------------------------------------------- encoders
static volatile long left_ticks = 0;
static volatile long right_ticks = 0;

void on_left_encoder() {
  bool forward = digitalRead(LEFT_ENC_A) != digitalRead(LEFT_ENC_B);
  left_ticks += (forward != LEFT_ENCODER_REVERSED) ? 1 : -1;
}

void on_right_encoder() {
  bool forward = digitalRead(RIGHT_ENC_A) != digitalRead(RIGHT_ENC_B);
  right_ticks += (forward != RIGHT_ENCODER_REVERSED) ? 1 : -1;
}

static void read_ticks(long &left, long &right) {
  noInterrupts();
  left = left_ticks;
  right = right_ticks;
  interrupts();
}

// ---------------------------------------------------------------- motors
// Cytron MD10C / MDD10A: DIR selects the direction, PWM duty sets the speed
struct Motor {
  uint8_t pwm, dir;
  bool reversed;

  void begin() {
    pinMode(pwm, OUTPUT);
    pinMode(dir, OUTPUT);
    set(0);
  }

  // command in -255..255
  void set(int command) {
    if (reversed) command = -command;
    command = constrain(command, -255, 255);
    digitalWrite(dir, command >= 0 ? HIGH : LOW);
    analogWrite(pwm, abs(command));
  }
};

static Motor left_motor = {LEFT_PWM, LEFT_DIR, LEFT_MOTOR_REVERSED};
static Motor right_motor = {RIGHT_PWM, RIGHT_DIR, RIGHT_MOTOR_REVERSED};

// ---------------------------------------------------------------- PID
struct WheelPid {
  float kp = PID_KP, ki = PID_KI, kd = PID_KD, kf = PID_KF;
  float integral = 0, last_error = 0;

  void reset() {
    integral = 0;
    last_error = 0;
  }

  // target and measured in rad/s, dt in seconds, returns PWM counts
  int update(float target, float measured, float dt) {
    if (target == 0.0f && fabsf(measured) < 0.2f) {
      reset();  // let the robot rest without integral wind-up humming the motors
      return 0;
    }
    float error = target - measured;
    float derivative = (error - last_error) / dt;
    last_error = error;

    float unclamped = kf * target + kp * error + ki * (integral + error * dt) + kd * derivative;
    // Anti-windup: only integrate while the output is not saturated
    if (fabsf(unclamped) < 255.0f) integral += error * dt;
    float output = kf * target + kp * error + ki * integral + kd * derivative;
    return (int)constrain(output, -255.0f, 255.0f);
  }
};

static WheelPid left_pid, right_pid;

// ---------------------------------------------------------------- state
static float left_target = 0, right_target = 0;     // rad/s
static float left_measured = 0, right_measured = 0;  // rad/s
static float battery_volts = 0;
static bool battery_low = false;
static unsigned long last_cmd_ms = 0;

static float read_battery() {
  return analogRead(BATTERY_PIN) * (ADC_VREF / ADC_MAX) * BATTERY_DIVIDER_RATIO;
}

static void stop_motors() {
  left_target = right_target = 0;
  left_pid.reset();
  right_pid.reset();
  left_motor.set(0);
  right_motor.set(0);
}

// ---------------------------------------------------------------- serial protocol
static char line[64];
static uint8_t line_len = 0;

static void handle_line(char *cmd) {
  switch (cmd[0]) {
    case 'V': {  // V <left rad/s> <right rad/s>
      char *rest = cmd + 1;
      float l = strtod(rest, &rest);
      float r = strtod(rest, &rest);
      left_target = constrain(l, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
      right_target = constrain(r, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
      last_cmd_ms = millis();
      break;
    }
    case 'S':  // stop
      stop_motors();
      break;
    case 'R':  // reset encoder counts
      noInterrupts();
      left_ticks = right_ticks = 0;
      interrupts();
      break;
    case 'P': {  // P <kp> <ki> <kd> <kf>, live PID tuning
      char *rest = cmd + 1;
      float kp = strtod(rest, &rest), ki = strtod(rest, &rest);
      float kd = strtod(rest, &rest), kf = strtod(rest, &rest);
      WheelPid *pids[] = {&left_pid, &right_pid};
      for (WheelPid *pid : pids) {
        pid->kp = kp; pid->ki = ki; pid->kd = kd; pid->kf = kf;
        pid->reset();
      }
      break;
    }
    case '?':  // identify
      Serial.print(F("I "));
      Serial.print(FIRMWARE_ID);
      Serial.print(' ');
      Serial.println(TICKS_PER_REV);
      break;
    default:
      Serial.print(F("! unknown command: "));
      Serial.println(cmd);
  }
}

static void poll_serial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[line_len] = '\0';
      if (line_len > 0) handle_line(line);
      line_len = 0;
    } else if (line_len < sizeof(line) - 1) {
      line[line_len++] = c;
    } else {
      line_len = 0;  // overlong line, drop it
    }
  }
}

// ---------------------------------------------------------------- main loop
void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), on_left_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), on_right_encoder, CHANGE);

  left_motor.begin();
  right_motor.begin();
  battery_volts = read_battery();

  Serial.print(F("I "));
  Serial.print(FIRMWARE_ID);
  Serial.print(' ');
  Serial.println(TICKS_PER_REV);
}

void loop() {
  static unsigned long last_control_ms = 0, last_telemetry_ms = 0, last_battery_ms = 0;
  static long last_left = 0, last_right = 0;
  unsigned long now = millis();

  poll_serial();

  if (now - last_cmd_ms > CMD_TIMEOUT_MS && (left_target != 0 || right_target != 0)) {
    stop_motors();
  }

  if (now - last_control_ms >= CONTROL_PERIOD_MS) {
    float dt = (now - last_control_ms) / 1000.0f;
    last_control_ms = now;

    long left, right;
    read_ticks(left, right);
    const float rad_per_tick = 2.0f * PI / TICKS_PER_REV;
    left_measured = (left - last_left) * rad_per_tick / dt;
    right_measured = (right - last_right) * rad_per_tick / dt;
    last_left = left;
    last_right = right;

    if (battery_low) {
      stop_motors();
    } else {
      left_motor.set(left_pid.update(left_target, left_measured, dt));
      right_motor.set(right_pid.update(right_target, right_measured, dt));
    }
  }

  if (now - last_telemetry_ms >= TELEMETRY_PERIOD_MS) {
    last_telemetry_ms = now;
    long left, right;
    read_ticks(left, right);
    // E <ms> <left ticks> <right ticks>
    Serial.print(F("E "));
    Serial.print(now);
    Serial.print(' ');
    Serial.print(left);
    Serial.print(' ');
    Serial.println(right);
  }

  if (now - last_battery_ms >= BATTERY_PERIOD_MS) {
    last_battery_ms = now;
    // Low-pass the reading so motor current spikes don't trip the cutoff
    battery_volts = 0.8f * battery_volts + 0.2f * read_battery();
    // 0.3 V hysteresis before motors are re-enabled
    if (battery_volts < BATTERY_LOW_VOLTS) battery_low = true;
    else if (battery_volts > BATTERY_LOW_VOLTS + 0.3f) battery_low = false;
    // B <volts> <low flag>
    Serial.print(F("B "));
    Serial.print(battery_volts, 2);
    Serial.print(' ');
    Serial.println(battery_low ? 1 : 0);
  }
}
