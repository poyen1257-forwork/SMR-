# WT901WIFI USB serial receive test

Use this when you only want to confirm that WT901WIFI data is received.

## Steps

1. Connect WT901WIFI to the PC with the USB A-C cable.
2. Open Windows Device Manager and check the COM port, usually shown as `USB-SERIAL CH340 (COMx)`.
3. Install Python dependency:

```powershell
pip install pyserial
```

4. List serial ports:

```powershell
python read_wt901_usb.py --list
```

5. Read data:

```powershell
python read_wt901_usb.py --port COM3
```

Use your actual COM number instead of `COM3`.

## Raw frame check

To also show raw 11-byte frames:

```powershell
python read_wt901_usb.py --port COM3 --raw
```

You should see frames beginning with:

- `55 51`: acceleration
- `55 52`: angular velocity
- `55 53`: angle
- `55 54`: magnetic field

If no data appears, try baud rates `9600`, `115200`, or check the WT901WIFI PC software output settings.
