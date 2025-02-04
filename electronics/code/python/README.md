# README

This is the README for the Python code for the Self-Balancing Robot.

## Setup

```
uv sync
```

This will do the following:
1. Find or download an appropriate Python version to use.
2. Create and set up your environment in the .venv folder.
3. Build your complete dependency list and write to your uv.lock file.
4. Sync your project dependencies into your virtual environment.

## Run the scripts

**NB: this is deprecated! We now use the C++ script `read_serial.cpp`**

Read data over serial:

`uv run plot_serial.py`

Run the wheel speed calibration script:

`uv run plot_pwm_calibration.py`