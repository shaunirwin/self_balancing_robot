# README

This is the README for the Python code for the Self-Balancing Robot.

## Setup

Run the commands below from `software/python`. Set up the environment first:

```sh
uv sync
```

This will do the following:
1. Find or download an appropriate Python version to use.
2. Create and set up your environment in the .venv folder.
3. Build your complete dependency list and write to your uv.lock file.
4. Sync your project dependencies into your virtual environment.

If matplotlib does not plot correctly, try installing the following:
```
sudo apt install libxcb-cursor0
```

## Run the scripts

For live serial data, use the [C++ serial reader](../../firmware/platformio_projects/self_balancing_robot/README.md#to-receive-serial-communication).

Run the wheel speed calibration script:

`uv run plot_pwm_calibration.py`

Run the step response script:

`uv run plot_step_response.py`


### Mujoco simulation

To activate the env:

`source .venv/bin/activate`

Once the environment is activated, run the viewer:

`python -m mujoco.viewer`

See [this tutorial](https://yasunori.jp/en/2024/07/13/mujoco-model-yourself.html) for more info.
