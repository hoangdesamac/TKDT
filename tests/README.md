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
These checks cover the production workflow with test mode disabled: UART IDs
and framing, classes 1/2/3, conveyor/servo ordering, Stop, retries and fault
handling. They do not verify electrical wiring, physical angles, camera accuracy,
actual servo travel time or mango movement. Use `pio run` to build for ESP32.
