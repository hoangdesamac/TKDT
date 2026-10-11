# Host checks

Run from the folder containing `platformio.ini`.

Python protocol tests do not require pySerial or attached hardware:

```sh
python -B -m unittest discover -s tests -p test_uart_bridge.py -v
```

Compile the real firmware against lightweight hardware fakes with a C++17 host
compiler. Run a PlatformIO build first so `.pio/build` exists:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -I tests/fakes -I include tests/scan_workflow_host.cpp -o .pio/build/scan-workflow-host.exe
```

On Windows PowerShell run `& ./.pio/build/scan-workflow-host.exe`.
This test explicitly disables the default two-IR belt mode to exercise Pi sorting.

Check the default two-IR belt mode separately:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -I tests/fakes -I include tests/ir_stop_host.cpp -o .pio/build/ir-stop-host.exe
```

On Windows PowerShell run `& ./.pio/build/ir-stop-host.exe`. It checks automatic
startup, independent IR1/IR2 stops, overlapping and held obstacles, auto-resume,
startup with either IR blocked, and operation without Pi or PCA9685/servo commands.

These checks cover the production workflow with test mode disabled: UART IDs
and framing, automatic startup (including an occupied IR1), ignored G34/USB
manual commands, classes 1/2/3, conveyor/servo ordering, reset and fault
handling. They do not verify electrical wiring, physical angles, camera accuracy,
actual servo travel time or mango movement. Use `pio run` to build for ESP32.
