# Base controller firmware

Arduino Nano firmware for the AMR's drive base. It runs a 50 Hz velocity PID per wheel using the encoders, drives two 24 V wiper motors through Cytron DIR/PWM drivers, watches the battery voltage, and talks to the Raspberry Pi over USB serial.

## Build and flash

```bash
pip install platformio
cd firmware
pio run -t upload
pio device monitor        # 115200 baud
```

## Wiring

| Signal | Nano pin | Goes to |
|---|---|---|
| Left motor PWM / DIR | D9 / D8 | Cytron channel 1 PWM / DIR |
| Right motor PWM / DIR | D10 / D12 | Cytron channel 2 PWM / DIR |
| Left encoder A / B | D2 / D4 | D2 is interrupt 0 |
| Right encoder A / B | D3 / D7 | D3 is interrupt 1 |
| Battery sense | A0 | 100 kΩ / 15 kΩ divider from the 24 V pack |

All pins, drivetrain constants, PID gains and the battery cutoff are in [`include/config.h`](include/config.h).

## Serial protocol

Plain-text lines at 115200 baud, so you can drive the robot from a serial monitor while testing.

| Direction | Line | Meaning |
|---|---|---|
| Pi → Nano | `V <left> <right>` | Target wheel speeds in rad/s |
| Pi → Nano | `S` | Stop now |
| Pi → Nano | `R` | Reset encoder counts |
| Pi → Nano | `P <kp> <ki> <kd> <kf>` | Set PID gains live |
| Pi → Nano | `?` | Ask for the firmware ID |
| Nano → Pi | `E <ms> <left ticks> <right ticks>` | Encoder counts, 50 Hz |
| Nano → Pi | `B <volts> <low>` | Battery voltage and low flag, 1 Hz |
| Nano → Pi | `I <id> <ticks per rev>` | Firmware ID, sent at boot and on `?` |

## Safety

- **Watchdog:** if no `V` command arrives for 500 ms (Pi crashed, cable pulled), the motors stop.
- **Low battery:** below 19.8 V (3.3 V per cell, 6S) the motors are disabled. They re-enable after the voltage recovers by 0.3 V. With no battery on A0, for example when bench-testing on USB power only, the motors stay disabled.
- **Anti-windup:** the integral term only grows while the output is not saturated.

## Calibration

1. **Ticks per revolution:** send `R`, turn one wheel exactly one revolution by hand, and read the count from the next `E` line. Put it in `TICKS_PER_REV` here and in `ticks_per_rev` in `ros2_ws/src/amr_base/config/base.yaml`. The ROS driver warns if they differ.
2. **Direction:** send `V 2 2`. Both wheels should roll forward and both tick counts should go up. Flip the `*_REVERSED` flags if not.
3. **PID:** with the robot on blocks, send steps such as `V 3 3` and adjust gains live with `P`. Wiper motors have a worm gearbox and respond slowly, so most of the output comes from the feed-forward term `kf`.
