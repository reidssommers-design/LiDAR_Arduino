# LiDAR_Arduino

A cheap 360° 2D scanner. A TF-Luna rangefinder sits on a stepper motor, the Nano
spins it and streams readings over USB, and a Python script draws the room live
and saves everything to a CSV.

## Hardware

- Arduino Nano (ATmega168 in my case)
- A4988 stepper driver + 200 step/rev stepper
- Benewake TF-Luna, in I2C mode (pin 5 tied to GND), default address 0x10

Wiring:

| From       | To          |
|------------|-------------|
| A4988 STEP | D7          |
| A4988 DIR  | D6          |
| TF-Luna SDA| A4          |
| TF-Luna SCL| A5          |

My A4988 has MS1-MS3 pulled high, so it runs at 1/16 microstepping. If yours is
wired differently, change `MICROSTEPS` at the top of the sketch or the angles
will be wrong (the plot will finish a circle before the motor does).

## Arduino

Sketch is in `360sweep_nano/`. Needs the
[TFLuna-I2C](https://github.com/budryerson/TFLuna-I2C) library.

Things you might want to tweak:

- `MICROSTEPS` - match your MS pin setup
- `STEP_DELAY_US` - lower is faster, too low and the motor skips
- `DATA_SEND_INTERVAL` - take a reading every N pulses (8 = every 0.9° at 1/16)

It prints `step,angle_deg,dist_cm,flux` at 57600 baud. Lines starting with `#`
are status messages, and `#SCAN n` marks the start of each revolution.

## Python

```
python3 -m venv .venv
.venv/bin/pip install pyserial numpy matplotlib PyQt6
```

On Ubuntu, Qt also needs `sudo apt install libxcb-cursor0` or the plot window
won't open.

**Live view**

```
.venv/bin/python lidar_live.py [PORT] [OUT.csv]
```

Port defaults to `/dev/ttyUSB0`, output defaults to `scan_<date>_<time>.csv`.
Close the window to stop. Press `p` to keep points from every revolution instead
of clearing each turn; press it again to clear.

Every sample gets logged, including the junk. Only the plot is filtered
(strength under 100 or distance over 8 m gets dropped).

**Looking at old scans**

```
.venv/bin/python csvParse.py [FOLDER]
```

Lists the CSVs, you pick one, it prints per-revolution stats and plots all the
revolutions on top of each other. `load_scan()` can be imported if you want the
data in a notebook.

## CSV format

```
scan,step,theta_deg,dist_cm,strength
0,0,0.0,239,3458
0,8,0.9,239,3458
```

`scan` is the revolution number, `step` is the motor pulse count within that
revolution.
