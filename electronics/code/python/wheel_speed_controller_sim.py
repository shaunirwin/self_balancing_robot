# simulate wheel speed controller
# uses this tutorial as a basis: https://python-control.readthedocs.io/en/0.10.0/simulating_discrete_nonlinear.html

import numpy as np
import matplotlib.pyplot as plt

import control as ct


def main():
    # continuous-time plant model
    motor_ts = 0.039  # time constant of DFRobot motor [s]
    pulses_per_sec_per_duty_cycle = 5385. / 255.    # [(pusles/sec)/duty_cycle]
    motor_cont = ct.tf(pulses_per_sec_per_duty_cycle, (motor_ts, 1), inputs='u', outputs='y')

    t, y = ct.step_response(motor_cont, 0.1)
    plt.plot(t, y, label='continouous-time model of motor')
    plt.xlabel('Time [sec]')
    plt.ylabel('Wheel speed [pulses/sec]')

    # create discrete-time simulation form assuming a zero-order hold
    simulation_dt = 0.01/2 # time step for numerical simulation ("numerical integration")
    motor_discrete = ct.c2d(motor_cont, simulation_dt, 'zoh')

    t, y = ct.step_response(motor_discrete, 0.1)
    plt.plot(t, y, '.-', label='discrete-time model of motor')
    plt.legend()
    plt.xlabel('time (s)')
    plt.ylabel('Wheel speed [pulses/sec]')

    plt.show()



if __name__ == '__main__':
    main()
