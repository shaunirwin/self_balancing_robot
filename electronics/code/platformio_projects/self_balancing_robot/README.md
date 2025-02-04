# README

This is the README for the Self-Balancing Robot.

## Programming the ESP32 board

Plug the USB cable into the **right** USB slot on the ESP32-S3 WROOM Freenove board to program it and to send/receive serial communication, as shown in (this)[https://www.youtube.com/watch?v=VZDCkARFCPk&ab_channel=Freenove] video.

## To receive serial communication

In the terminal:

```
tio /dev/ttyACM0
```
Check that this is the correct comm port (see `platformio.ini`).

`Ctrl+t q` to quit.

Run the python script to communicate (currently from the conda base environment, but I should fix that):

```
python plot_serial.py
```
