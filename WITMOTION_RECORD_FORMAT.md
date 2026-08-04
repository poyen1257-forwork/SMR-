# WitMotion WT Text Record Format

This project follows the field order shown by the WitMotion official APP export.

Official APP columns:

```text
加速度X/Y/Z(g)
角速度X/Y/Z(deg/s)
角度X/Y/Z(deg)
磁場X/Y/Z(uT)
溫度(C)
電量(%)
信號()
版本號()
```

For the COM `WT...` text packet used by this BWT901CL/WT Wi-Fi module:

```text
F00-F02 = 加速度 X/Y/Z, raw / 32768 * 16, unit g
F03-F05 = 角速度 X/Y/Z, raw / 32768 * 2000, unit deg/s
F06-F08 = 磁場 X/Y/Z, raw / 10, unit uT
F09-F11 = WitMotion 官方角度 X/Y/Z, raw / 32768 * 180, unit deg
F12     = 溫度, raw / 100, unit C
F13     = 電量/狀態欄位
F14     = 信號欄位
F15     = 版本號
```

Visualizer default:

```text
--pose-source device
--angle-fields f09
--algorithm axis6
```

That means the 3D view uses WitMotion official angle X/Y/Z from `F09-F11`.

Algorithm setting:

```text
axis6 = FF AA 24 01 00
```

Use this for relative heading. This is usually better for checking that a 90 degree turn around the vertical Z axis is shown as a 90 degree change.

```text
axis9 = FF AA 24 00 00
```

Use this for magnetic absolute heading. This depends on magnetometer calibration and nearby magnetic interference. If the magnetic field is not calibrated well, Z/Yaw may look wrong or may be pulled toward an incorrect heading.

`--pose-source gravity-mag` is only for diagnosis. It computes a rough pose from acceleration and magnetic field, but it will not necessarily match the official APP angle because magnetometer calibration, heading reference, axis mapping, and local magnetic interference matter.
