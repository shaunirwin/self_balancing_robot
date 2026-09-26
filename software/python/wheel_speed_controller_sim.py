# simulate wheel speed controller
# uses this tutorial as a basis: https://python-control.readthedocs.io/en/0.10.0/simulating_discrete_nonlinear.html

import numpy as np
import matplotlib.pyplot as plt

import control as ct


def sampled_data_controller(controller, plant_dt):
    """
    Create a (discrete-time, non-linear) system that models the behavior
    of a digital controller.

    The system that is returned models the behavior of a sampled-data
    controller, consisting of a sampler and a digital-to-analog converter.
    The returned system is discrete-time, and its timebase `plant_dt` is
    much smaller than the sampling interval of the controller,
    `controller.dt`, to insure that continuous-time dynamics of the plant
    are accurately simulated. This system must be interconnected
    to a plant with the same dt. The controller's sampling period must be
    greater than or equal to `plant_dt`, and an integral multiple of it.
    The plant that is connected to it must be converted to a discrete-time
    approximation with a sampling interval that is also `plant_dt`. A
    controller that is a pure gain must have its `dt` specified (not None).
    """
    assert ct.isdtime(controller, True), "controller must be discrete-time"
    controller = ct.ss(controller) # convert to state-space if not already
    # the following is used to ensure the number before '%' is a bit larger
    one_plus_eps = 1 + np.finfo(float).eps
    assert np.isclose(0, controller.dt*one_plus_eps % plant_dt), \
        "plant_dt must be an integral multiple of the controller's dt"
    nsteps = int(round(controller.dt / plant_dt))
    step = 0
    def updatefunction(t, x, u, params): # update if it is time to sample
        nonlocal step
        if step == 0:
            x = controller._rhs(t, x, u)
        step += 1
        if step == nsteps:
            step = 0
        return x
    y = np.zeros((controller.noutputs, 1))
    def outputfunction(t, x, u, params): # update if it is time to sample
        nonlocal y
        if step == 0: # last time updatefunction was called was a sample time
            y = controller._out(t, x, u)
        return y
    return ct.ss(updatefunction, outputfunction, dt=plant_dt,
                 name=controller.name, inputs=controller.input_labels,
                 outputs=controller.output_labels, states=controller.state_labels)


def main():
    # continuous-time plant model
    tau_motor = 0.039  # time constant of DFRobot motor [s]
    pulses_per_sec_per_duty_cycle =  5385. / 255.    # [(pusles/sec)/duty_cycle]
    motor_cont = ct.tf(pulses_per_sec_per_duty_cycle, (tau_motor, 1), inputs='u', outputs='y', name='G_motor')

    # t, y = ct.step_response(motor_cont, 0.1)
    # plt.figure()
    # plt.plot(t, y, label='continouous-time model of motor')
    # plt.xlabel('Time [sec]')
    # plt.ylabel('Wheel speed [pulses/sec]')

    # create discrete-time simulation form assuming a zero-order hold
    freq_controller_sample = 100        # Hz
    controller_Ts = 1 / freq_controller_sample # sampling interval of controller
    freq_sim_sample = freq_controller_sample * 5    # Hz
    simulation_dt = 1./freq_sim_sample # time step for numerical simulation ("numerical integration")
    motor_discrete = ct.c2d(motor_cont, controller_Ts, 'zoh')
    motor_sim = ct.c2d(motor_cont, simulation_dt, 'zoh')

    # t, y = ct.step_response(motor_sim, 0.1)
    
    # plt.plot(t, y, '.-', label='discrete-time model of motor')
    # plt.legend()
    # plt.xlabel('time (s)')
    # plt.ylabel('Wheel speed [pulses/sec]')

    # feedforward controller based on plant dynamics
    print(motor_discrete)
    K_ff = 0.1
    controller_ff = K_ff * (1 / motor_discrete) / ct.TransferFunction.z    # shift by one time step to make causal
    controller_ff.set_inputs('r')
    controller_ff.set_outputs('u_ff')
    controller_ff.name = 'C_ff'
    print('ff controller:', controller_ff)

    # create discrete-time PID controller
    Kp = 1. * 0.25 * 0.68 #* 0.3
    Ki = 2.2
    Kd = 0.
    controller_p = ct.tf([Kp], [1], controller_Ts)
    controller_i = ct.tf([Ki * controller_Ts], [1, -1], controller_Ts)
    controller_d = ct.tf([Kd, -Kd], [controller_Ts, 0], controller_Ts)
    controller_pid = controller_p + controller_i + controller_d
    controller_pid.set_inputs('e')
    controller_pid.set_outputs('u_fb')
    controller_pid.name = 'C_pid'

    # create discrete-time lag controller
    controller_p = ct.tf([Kp], [1], controller_Ts)      # TODO: implement lag compensator

    controller_fb = controller_lag

    # create model of controller with a much shorter sampling time for simulation
    controller_simulator = sampled_data_controller(controller_pid, simulation_dt)
    controller_simulator_ff = sampled_data_controller(controller_ff, simulation_dt)

    time = np.arange(0, 1.5, simulation_dt)
    unit_step_input = np.ones_like(time)
    # t, y = ct.input_output_response(controller_simulator, time, unit_step_input)
    # plt.plot(t, y, '.-')

    # simulate closed loop system

    # plantcont = ct.tf(.5, (0.1, 1), inputs='u', outputs='y')
    err_summer  = ct.summing_junction(inputs=['-y', 'r'], outputs='e', name='sum_err')
    u_summer  = ct.summing_junction(inputs=['u_ff', 'u_fb'], outputs='u', name='sum_u')

    # plant_simulator = ct.c2d(motor_cont, simulation_dt, 'zoh')
    # system from r to y
    closed_loop_simulator = ct.interconnect([controller_simulator, motor_sim, err_summer, u_summer], #, controller_simulator_ff],
        inputs='r', outputs=['y', 'u'])

    # closed_loop_simulator = ct.interconnect([controller_pid, 
    #                                          controller_ff, 
    #                                          motor_sim],
    #     inputs='r', outputs=['y', 'u'])
    
    print(closed_loop_simulator.connection_table(show_names=True) )
    
    H = controller_fb * motor_discrete
    # H_sim = controller_simulator * motor_sim
    # closed_loop_tf = ct.feedback(H, 1, sign=-1)

    # s = ct.TransferFunction.s
    # G  = (10)/(s**2 + 2*s)
    # G = motor_cont
    # H = motor_cont / (1 + motor_cont)       # closed loop tf

    # simulate
    setpoint = 700.     # setpoint wheel speed [pulses/sec]
    t, y = ct.input_output_response(closed_loop_simulator, time, unit_step_input * setpoint)
    y, u = y # extract responses
    fig, ax = plt.subplots(nrows=2, ncols=1)
    ax[0].plot(t, y, '.-', label='y')
    ax[0].set_ylabel('pulses/sec')
    ax[0].axhline(y=setpoint, color='r', linestyle='-.')
    ax[0].legend()
    ax[1].plot(t, u, '.-', label='u')
    ax[1].set_ylabel('duty cycle / 255')
    ax[1].set_xlabel('time [sec]')
    ax[1].legend()

    plt.figure()
    # ct.rlocus(motor_cont)
    ct.rlocus(H)
    # ct.root_locus_map(motor_cont) #, gains=[0.1, 1, 10]) #np.linspace(start=0.01, stop=100, num=10))

    # print(ct.poles(motor_cont))
    # print(ct.damp(H))
    # print(ct.margin(motor_cont))

    # print(closed_loop_tf)

    plt.show()



if __name__ == '__main__':
    main()
