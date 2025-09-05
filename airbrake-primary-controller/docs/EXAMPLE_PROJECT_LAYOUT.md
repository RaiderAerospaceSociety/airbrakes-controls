Example Modular Layout for PlatformIO + ESP32 (Arduino)

Structure
- src/main.cpp: Orchestrates init and starts tasks
- include/pins.h: Board pin names and overrides
- include/config.h: Feature flags, task sizes/priorities, constants
- include/logging.h + src/logging.cpp: Mutex-guarded Serial + debug macros
- include/bus.h + src/bus.cpp: Global SPI/Wire init + shared bus mutexes
- include/board.h + src/board.cpp: Board-specific setup (UMS3 LED, etc.)
- include/sensors/bmp390.h + src/sensors/bmp390.cpp: BMP390 task + data getter
- include/tasks/led.h + src/tasks/led.cpp: LED animation task

Notes
- Initialize universal resources in setup: Serial, logging mutex, Wire, SPI, board peripherals
- Each module owns its own data + mutex and exposes a small API
- Use a shared SPI mutex across SPI sensors to avoid interleaved transfers
- Favor start_task() style APIs from modules to keep main.cpp minimal

Working Example
- See the files under examples/modular/ for a complete, compilable pattern you can mirror.

