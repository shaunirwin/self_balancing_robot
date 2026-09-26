from matplotlib import pyplot as plt
import numpy as np
import pandas as pd


# read step response of motors
df_motors = pd.read_csv('../platformio_projects/self_balancing_robot/src/motorSpinUp.csv', sep=',', header='infer')


# print(df_motors.head())

# fig, ax = plt.subplots(nrows=2, ncols=1, sharex=True)
# ax[0].plot(df_motors['timeMs'], df_motors['dutyCycle1'], label='M1 duty cycle')
# ax[0].legend()
# ax[0].set_ylabel('duty cycle')
# ax[1].plot(df_motors['timeMs'], -df_motors['motor1EncoderPulsesPerSec'], label='M1 pulses/s')
# ax[1].legend()
# ax[1].set_xlabel('ms')
# ax[1].set_ylabel('pulses/s')
# ax[1].axvline(x=130176, color='r')
# y_settle = 5e3
# ax[1].axhline(y=y_settle, color='g')
# ax[1].axhline(y=(1-1/np.e) * y_settle, color='m')
# ax[1].axvline(x=130215, color='k', label='time constant')

motor_time_constant_sec = (130215 - 130176) / 1000. 
print('motor_time_constant_sec:', motor_time_constant_sec)


# plot step response of robot falling

df_falling = pd.read_csv('../platformio_projects/self_balancing_robot/src/freefall.csv', sep=',', header='infer')

print(df_falling.head())

plt.figure()
plt.plot(df_falling['timeMs'], df_falling['pitch_est deg'], label='pitch angle')
plt.xlabel('ms')
plt.ylabel('pitch angle [deg]')
x_start = 145997
plt.axvline(x=x_start, color='r')  # starts falling
plt.axvline(x=146515, color='g')    # reaches 25 deg pitch angle (this is chosen since we can't recover beyond that)
y_start = 1.6   # pitch angle before starting to fall
y_end = 25     # pitch angle at end
y_time_constant = y_end-(y_end-y_start)*1/np.e    # pitch angle that corresponds to the time constant
# plt.axhline(y=y_settle, color='g')
plt.axhline(y=y_time_constant, color='m')
# plt.axvline(x=146430, color='k', label='time constant')
plt.legend()

falling_time_constant_sec = (146430 - x_start) / 1000. 
print('falling_time_constant_sec:', falling_time_constant_sec)

plt.show()

