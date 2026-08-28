# Python Motor Control

This folder contains PC-side Python programs. The ESP32 converts these serial
commands into CAN frames for the MF4015V2 motor.

## Install

```powershell
python -m pip install pyserial
```

## Find the ESP32 port

```powershell
python python/esp32_motor_control.py --list-ports
```

## Interactive control

Close Arduino Serial Monitor and VS Code Serial Monitor first, then run:

```powershell
python python/esp32_motor_control.py --port COM7
```

Commands:

```text
r        Read motor state
m 0      Command zero speed
m 30     Run continuously at 30 dps
m 360    Run continuously at 360 dps
n 180 30 Run at 30 dps for 6 seconds, then send m 0 (about 180 degrees)
s        Soft stop: reduce 10 dps every 100 ms
q        Send s, close the serial port, and exit Python
```

The valid speed range is `0..360` dps. For `n`, the angle range is `0..360`
degrees. A nonzero angle requires a speed from `1..360` dps. The ESP32 only
receives `m <speed>` and `m 0`; Python calculates `seconds = angle / speed`.
The program also sends `s` when it exits.

## One-command test

```powershell
python python/esp32_motor_control.py --port COM7 --command r
```

Do not open the same COM port in Arduino Serial Monitor and Python at the same
time. Only one program can own `COM7`.
