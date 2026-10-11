# ESP32 and Raspberry Pi UART sorting

## Current mode: two-IR belt test (no Pi)

`IR_BELT_TEST_ENABLED` defaults to `1` in `src/main.cpp`. This mode takes
priority over all sorting and servo-test code. IR1 on GPIO32 and IR2 on GPIO33
control the belt through GPIO26: either LOW (obstacle) requests STOP; both HIGH
(clear) request RUN. Clearing only one sensor does not restart the belt while
the other is blocked. No Start button, Pi, PCA9685 or servo is required;
no servo commands are sent. Connect each sensor OUT to its corresponding GPIO
and share signal GND. Both outputs must be ESP32-compatible 3.3 V logic.

Upload again and open Monitor at 115200 baud. Expect `FIRMWARE: IR_STOP_TEST`
and `IR_TEST: G32=1 G33=0 BLOCKED=1 REQUEST=STOP` when only IR2 is blocked.
`RELAY_CMD` is the commanded GPIO level, not actual motor/contact feedback.
Relay polarity remains `RELAY_ON LOW`, `RELAY_OFF HIGH`; do not change wiring
while powered. The belt starts automatically if both IRs are clear at power-up.
Keep hands clear of the mechanism and use an object to trigger the sensor.

To restore the Pi workflow below, change `IR_BELT_TEST_ENABLED` to `0`, keep
`SENSOR_STOP_TEST_MODE = false`, and rebuild/upload.

## Workflow

Normal sorting uses `IR_BELT_TEST_ENABLED = 0` and
`SENSOR_STOP_TEST_MODE = false` in `src/main.cpp`.
The previous two-IR servo test remains available with this flag set to `true`.
UART messages cannot trigger sorting while test mode is enabled.

Process one mango at a time:

1. At startup the belt is stopped and both servos are commanded home. Wait for
   500 ms homing, then the belt starts automatically. If IR1 is already blocked,
   it stays stopped and automatically requests a scan instead.
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

## Automatic operation and faults

No physical button or USB START/STOP/STATUS command is required or processed.
GPIO34 is unused. USB Serial Monitor displays logs only. After homing, normal
operation begins automatically without a computer connected.

**The belt can move after power-up or reset, including resets caused by opening
Serial Monitor or uploading. Keep hands clear and isolate motor/servo power
before wiring or maintenance. A fault is not a safety-rated emergency stop.**

- No valid Pi reply within 15 seconds: `PI_TIMEOUT`, belt stays stopped.
- Class 2 fails to reach IR2 within 10 seconds: `IR2_TIMEOUT`.
- A processed mango fails to leave its IR within 10 seconds: `EXIT_TIMEOUT`.
- PCA9685 missing at boot: no belt movement is allowed.
- Old, duplicate, malformed and mismatched results are ignored. A randomized
  starting ID reduces confusion with replies from before an ESP32 reset.
- Faults remain latched: clearing IR or sending a late result cannot restart
  the belt. Inspect/clear the machine first, then reset ESP32 to retry.
- A reset re-enables automatic movement after homing; do not reset while working
  on the mechanism. With a mango at IR1, reset requests a new scan ID.

Removing a mango while waiting for Pi does not itself restart the conveyor.
Servo motion depends on a valid Pi result, not just an IR trigger. To stop the
machine manually, use a suitable physical power disconnect/safety stop; closing
Python or Monitor is not a stop command. Keep the working relay polarity/contact
wiring. Always power down before changing wiring.

## Running the automatic workflow

1. Upload the firmware with motor/servo power isolated.
2. Prepare the Pi UART program and verify wiring before feeding any mango.
3. Clear the mechanism, then power the system. It starts after servo homing.
4. Keep the Pi program running. IR1 stops the mango and requests classification;
   no keyboard command is needed to complete normal cycles.
5. Monitor at 115200 baud is optional for logs. Opening it may reset ESP32 and
   therefore restart automatic operation after homing.

Diagnostic firmware prints `FIRMWARE: AUTO_UART_DIAG1` at boot and includes the
same ID in its status heartbeat. `IR_RAW` logs G32/G33 changes (LOW means blocked);
`RELAY_CMD` logs the requested output level, not confirmed contact/motor state.
`SORT_STATE` logs workflow transitions and `BOOT_DIAG` reports the ESP32 reset
reason. Capture these logs before pressing EN/RST when investigating power-up.

With `IR_BELT_TEST_ENABLED = 0`, `SENSOR_STOP_TEST_MODE = true` also runs automatically after its servo homing
wait, but bypasses Pi classification and directly gates on IR1/IR2 for testing.
Normal sorting must use `false`.

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
before placing any mango at IR1:

```sh
python3 uart_bridge.py --mode scan --result 3
```

Every mango at IR1 is treated as class 3 and gated by channel 0. Repeat with
`--result 2` to test transport to IR2 and channel 1; `--result 1` tests pass-through.
The script does not use a camera or make a real quality assessment.

Ctrl+C stops Python only, not a belt already moving. Use a physical power
disconnect/safety stop to stop the machine manually.
Loss of Python while waiting for classification produces a timeout fault.

## Protocol and camera integration

Messages are ASCII. Send one newline-terminated result per request.

| Direction | Payload | Meaning |
| --- | --- | --- |
| Pi -> ESP32 | `P` or `P\n` | Connection check, including while stopped |
| ESP32 -> Pi | `PONG\r\n` | Connection check reply |
| ESP32 -> Pi | `SYSTEM_AUTO_START\r\n` | Automatic operation enabled after homing |
| ESP32 -> Pi | `SCAN,123\n` | Classify at IR1; example request ID 123 |
| Pi -> ESP32 | `RESULT,123,3\n` | Matching scan found class 3 |
| ESP32 -> Pi | `ERROR,PI_TIMEOUT\n` | Fault notification; belt stopped |

Classes are 1, 2 and 3. IDs are nonzero unsigned 32-bit integers. Bare bytes
`1`, `2`, `3` and `H` are no longer classification commands. USB debug Serial
is not the Pi classification port. Both ends must use this new protocol.

For an existing camera/model, replace the fixed result in `serve_scans` with
its output: capture/classify after `SCAN,<id>`, then call
`send_classification(uart, scan_id, predicted_class)` within 15 seconds. Preserve
the request ID. The test bridge preserves scan requests received during its
connection-check handshake so an automatic startup scan is not silently lost.
This repository does not yet contain the camera/model
implementation; UART/mechanism tests alone do not establish sorting accuracy.

## References

- [Raspberry Pi UART configuration](https://www.raspberrypi.com/documentation/computers/configuration.html#configure-uarts)
- [ESP32 GPIO capabilities](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gpio.html)
- [ESP32 HardwareSerial](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/serial.html)
- [pySerial introduction](https://pyserial.readthedocs.io/en/latest/shortintro.html)
