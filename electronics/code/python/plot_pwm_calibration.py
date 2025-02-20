import matplotlib
# matplotlib.use('TkAgg')  # Use the Tkinter backend
from matplotlib import pyplot as plt
import numpy as np

# CQRobot motors
"""
40,0.63,0.64
50,0.79,0.81
60,0.96,0.98
90,1.46,1.5
100,1.63,1.68
130,2.13,2.21
150,2.47,2.56
180,2.97,3.09
200,3.31,3.44
220,3.64,3.8
240,3.97,4.13
255,4.24,4.41
"""

duty_cycles = np.array([40,50,60,90,100,130,150,180,200,220,240,255])
wheel_1_speed = np.array([0.63, 0.79, 0.96, 1.46, 1.63, 2.13, 2.47, 2.97, 3.31, 3.64, 3.97, 4.24])
wheel_2_speed = np.array([0.64, 0.81, 0.9, 1.5, 1.68, 2.21, 2.56, 3.09, 3.44, 3.8, 4.13, 4.41])

# DFRobot motors - fwd (measured in pulses/sec)
"""
10,115,0
15,270,255
23,435,425
25,465,445
30,590,565
50,1025,1013
100,2090,2085
150,3155,3135
200,4215,4080
255,5385,5375 = 0.372m/s
"""

pulses_per_rev = 1400
wheel_diameter_m = 0.0618
duty_cycles = np.array([15,23,25,30,50,100,150,200,255])
wheel_1_speed_pulses_per_sec = np.array([270,435,465,590,1025,2090,3155,4215,5385])
wheel_2_speed_pulses_per_sec = np.array([255,425,445,565,1013,2085,3135,4080,5375])

wheel_1_speed = wheel_1_speed_pulses_per_sec * 1. / pulses_per_rev * (wheel_diameter_m / 2 * np.pi)
wheel_2_speed = wheel_2_speed_pulses_per_sec * 1. / pulses_per_rev * (wheel_diameter_m / 2 * np.pi)

wheel_speed_diff = wheel_1_speed - wheel_2_speed

coef1 = np.polyfit(duty_cycles, wheel_1_speed, 1)
coef2 = np.polyfit(duty_cycles, wheel_2_speed, 1)
poly1d_fn1 = np.poly1d(coef1) 
poly1d_fn2 = np.poly1d(coef2) 

m1, b1 = coef1
m2, b2 = coef2

# assuming motor 1 has its duty cycle set explicitly, motor 2 duty cycle according to the following correction:
wheel_2_speed_corrected = (wheel_2_speed - b2) * (m1 / m2) + b1

# dutyCycleCorrected = wheel_2_speed_corrected/wheel_2_speed * dutyCycle ????

# dutyCycleCorrected = ((m1 * dutyCycle + b1) - b2) / m2;

print(m1, b1, m2, b2)     # calculated to be: [ 0.01676954 -0.04664655] [ 0.01765459 -0.09230133]

plt.figure()
plt.plot(duty_cycles, wheel_1_speed, 'bo', duty_cycles, poly1d_fn1(duty_cycles), '--b', label='wheel 1')
plt.plot(duty_cycles, wheel_2_speed, 'ro', duty_cycles, poly1d_fn2(duty_cycles), '--r', label='wheel 2')
plt.plot(duty_cycles, wheel_2_speed_corrected, 'g:', label='wheel 2 corrected')
plt.legend()
plt.grid()
plt.xlabel('Duty cycle')
plt.ylabel('wheel speed [m/s]')
# plt.show()
plt.savefig('pwm_calib.png')


plt.figure()
plt.plot(duty_cycles, wheel_1_speed_pulses_per_sec, 'bo', label='wheel 1')
plt.plot(duty_cycles, wheel_2_speed_pulses_per_sec, 'ro', label='wheel 2')
plt.xlabel('duty cycle')
plt.ylabel('wheel speed [pulses/s]')
plt.show()