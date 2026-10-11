# ESP32 and Raspberry Pi UART sorting

## Workflow

Normal firmware now uses `SENSOR_STOP_TEST_MODE = false` in `src/main.cpp`.
The previous two-IR servo test remains available with this flag set to `true`.
UART messages cannot trigger sorting while test mode is enabled.

Process one mango at a time:

1. At startup the belt is stopped and both servos are commanded home. Wait for
   homing, then press Start.
2. IR1 (GPIO32, active LOW) detects a mango. The belt stops, settles for 200 ms,
   then ESP32 sends `SCAN,<id>` to the Pi.
3. The belt remains stopped while waiting for a matching result:
   - Class 3: servo 1 on PCA9685 channel 0 gates the mango. After 800 ms the belt
     runs while the gate is held to direct the mango down.
   - Class 2: the belt moves the mango to IR2 (GPIO33). There it stops, settles,
     and servo 2 on channel 1 gates the mango using the same sequence.
   - Class 1: the belt resumes and the mango passes straight through.
4. For a gated mango the belt runs for at least 1000 ms, and the relevant IR
   must be clear for 150 ms before the gate returns. The belt stops during the
   500 ms servo return wait, then resumes for the next mango.

These are timed movements, not servo position or chute-exit feedback. Adjust
timings and mechanics using a disconnected linkage first. Servo 1 must physically
be at the IR1 sorting point and servo 2 at IR2. IR3 is unused. Do not feed multiple
mangoes simultaneously.

Pulse defaults are deliberately limited to `SERVO_REST_US = 1500` and
`SERVO_GATE_US = 1700`. Calibrate direction and travel for each gate; they do
not guarantee specific angles or that a mango falls successfully. MG995 power
must come from a suitably rated regulated supply, not ESP32 GPIO, 3V3, or the
relay's 12 V supply. PCA9685 VCC is logic power; V+ is servo power.

## Faults and Stop

- No valid Pi reply within 15 seconds: `PI_TIMEOUT`, belt stays stopped.
- Class 2 fails to reach IR2 within 10 seconds: `IR2_TIMEOUT`.
- A processed mango fails to leave its IR within 10 seconds: `EXIT_TIMEOUT`.
- PCA9685 missing at boot: no belt movement is allowed.
- Start/Stop remains responsive during UART waits and timed servo movements.
  Stop cancels the pending scan, stops the belt and commands both servos home.
- Old, duplicate, malformed and mismatched results are ignored. A randomized
  starting ID reduces confusion with replies from before an ESP32 reset.
- After a fault inspect/remove the mango and clear the mechanism before pressing
  Start to retry. If a mango is still at IR1, Start requests a fresh scan ID.

Removing a mango while waiting for Pi does not itself restart the conveyor.
After a valid result the machine can move automatically. Keep hands away from
moving parts. The software button is not a safety-rated emergency stop.

GPIO34 has no internal pull-up on ESP32. The Start button needs a proper external
pull-up to 3.3 V: released must read HIGH, pressed LOW. Keep the working relay
polarity/contact wiring. Always power down before changing wiring.

## UART wiring

Power off both boards first. UART signals are 3.3 V; never put 5 V or 12 V on
TX/RX. Power each board normally; do not join their positive supply rails.

| ESP32 | Raspberry Pi 4 40-pin header |
| --- | --- |
| GPIO17 (TX) | GPIO15 (RX), physical pin 10 |
| GPIO16 (RX) | GPIO14 (TX), physical pin 8 |
| GND | GND, physical pin 6 |

UART2 uses 115200 baud, 8N1, no flow control. USB Serial remains available
separately for debug logs at 115200 baud. Upload with PlatformIO.

## Raspberry Pi setup

In `sudo raspi-config`, under Interface Options > Serial Port, disable the serial
login shell and enable serial hardware, then reboot. Pi 3/4 and Zero 2 W normally
use `/dev/serial0` for GPIO UART. Pi 5 has different defaults; consult its UART
configuration before using this wiring.

Install Python serial support:

```sh
sudo apt update
sudo apt install python3-serial
```

For a device permission error, run `sudo usermod -aG dialout "$USER"`, then log
out and back in. Make the `raspberry` folder available on Pi and run inside it:

```sh
python3 uart_bridge.py
```

Expected: `PONG` and `UART OK`. Then start a fixed-result UART/mechanism test
before pressing the ESP32 Start button:

```sh
python3 uart_bridge.py --mode scan --result 3
```

Every mango at IR1 is treated as class 3 and gated by channel 0. Repeat with
`--result 2` to test transport to IR2 and channel 1; `--result 1` tests pass-through.
The script does not use a camera or make a real quality assessment.

Ctrl+C stops Python only, not a belt already moving. Use the ESP32 Stop button.
Loss of Python while waiting for classification produces a timeout fault.

## Protocol and camera integration

Messages are ASCII. Send one newline-terminated result per request.

| Direction | Payload | Meaning |
| --- | --- | --- |
| Pi -> ESP32 | `P` or `P\n` | Connection check, including while stopped |
| ESP32 -> Pi | `PONG\r\n` | Connection check reply |
| ESP32 -> Pi | `SYSTEM_START\r\n` / `SYSTEM_STOP\r\n` | Button events |
| ESP32 -> Pi | `SCAN,123\n` | Classify at IR1; example request ID 123 |
| Pi -> ESP32 | `RESULT,123,3\n` | Matching scan found class 3 |
| ESP32 -> Pi | `ERROR,PI_TIMEOUT\n` | Fault notification; belt stopped |

Classes are 1, 2 and 3. IDs are nonzero unsigned 32-bit integers. Bare bytes
`1`, `2`, `3` and `H` are no longer classification commands. USB debug Serial
is not the Pi classification port. Both ends must use this new protocol.

For an existing camera/model, replace the fixed result in `serve_scans` with
its output: capture/classify after `SCAN,<id>`, then call
`send_classification(uart, scan_id, predicted_class)` within 15 seconds. Preserve
the request ID. This repository does not yet contain the camera/model
implementation; UART/mechanism tests alone do not establish sorting accuracy.

## References

- [Raspberry Pi UART configuration](https://www.raspberrypi.com/documentation/computers/configuration.html#configure-uarts)
- [ESP32 GPIO capabilities](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gpio.html)
- [ESP32 HardwareSerial](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/serial.html)
- [pySerial introduction](https://pyserial.readthedocs.io/en/latest/shortintro.html)
