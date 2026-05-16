# Plotting Tools

Three Python scripts provide real-time visualization of motor and IMU data via serial connection.

## File Inventory

| Script | Location | Purpose | Port |
|--------|----------|---------|------|
| `ArduinoPlotter.py` | `/RocBot/` | Single-motor plotter | COM5 |
| `ArduinoMotorPlotter.py` | `/RocBot/Motors/` | Multi-motor plotter | /dev/ttyUSB0 |
| `IMUPlotter.py` | `/RocBot/IMU/I2C/` | IMU data plotter (copy of single-motor) | COM6 |

## Dependencies

- `pyserial` — serial communication
- `matplotlib` — plotting (with `FuncAnimation`)
- Python 3.x

## `ArduinoMotorPlotter.py` (Multi-Motor)

The most advanced plotter. Designed for the `MotorControlPID` firmware.

### Architecture

**`MotorData` class** — per-motor data container:
- Stores 500 data points per field
- Color-coded: FL=blue, FR=red, BL=green, BR=yellow
- `checkMotor()` matches blocks starting with motor ID
- `update()` parses key:value pairs

**`AnimationPlot` class** — manages the figure:
- 3 subplots: RPM tracker, Motor Power, RPM Error
- Sends `'g'` byte every frame
- Splits response on `&` for multi-motor parsing
- Plots each motor with distinct color and legend

### Serial Protocol Parsed

```
&FL;target:60;rpm:123.4;rpm_filt:122.1;pwr:150;pwr_filt:148.2
&FR;target:60;rpm:125.1;rpm_filt:123.8;pwr:145;pwr_filt:143.5
```

### Window

- 500-sample rolling window
- Auto-scaling Y axes
- 10ms animation interval

## `ArduinoPlotter.py` (Single Motor)

Simpler version for debugging one motor at a time:
- Plots raw and filtered RPM, motor power, and error
- 500-data-point window
- Target COM5 at 115200 baud

## `IMUPlotter.py`

Currently identical to `ArduinoPlotter.py`. Intended to be adapted for plotting IMU data (quaternions, Euler angles, etc.) once the IMU integration is active.

## Usage

```bash
# Multi-motor (Linux)
python Motors/ArduinoMotorPlotter.py

# Single motor (Windows)
python ArduinoPlotter.py
```

Adjust `serial_port` and `baud_rate` in the script to match your setup.
