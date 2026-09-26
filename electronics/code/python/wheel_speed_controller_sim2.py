import numpy as np
import control as ctl
import matplotlib.pyplot as plt

# Continuous-time first-order plant: G(s) = 1 / (s + 1)
G_s = ctl.tf([1], [1, 1])

# Sampling times
T_p = 0.01  # Fast plant sampling time
T_c = 0.05  # Slow controller sampling time
N = int(T_c / T_p)  # Ratio

# Discretize the plant
Gz = ctl.c2d(G_s, T_p, method='zoh')  # Plant at fast rate

# # Design PID feedback controller
# Kp, Ki, Kd = 1.0, 0.5, 0.1
# C_s = ctl.tf([Kd, Kp, Ki], [1, 0])  # PID controller
# Cz = ctl.c2d(C_s, T_c, method='tustin')  # Discretized at controller rate

# Design Feedforward Controller (Inverse Plant Approximation)
# Ideal inverse Gff = (s + 1), discretized at T_c
Gff_s = ctl.tf([1, 1], [1])  # Continuous inverse plant
Gff_z = ctl.c2d(Gff_s, T_c, method='tustin')  # Discretized feedforward controller

# Time vector
T_final = 2
time_steps = np.arange(0, T_final, T_p)

# Simulation setup
x_p = np.zeros((2, 1))  # Plant state
u_fb, u_ff = 0, 0  # Control signals
y_p, u_hist = [], []  # Logging
ref = 1  # Step reference

# Get discrete state-space data
_, Bz, Cz_mat, _ = ctl.ssdata(Gz)  # Discrete plant
_, Bd, Cd, _ = ctl.ssdata(Cz)  # PID controller
_, Bff, Cff, _ = ctl.ssdata(Gff_z)  # Feedforward controller

xd = np.zeros((Bd.shape[1], 1))  # PID controller state
xff = np.zeros((Bff.shape[1], 1))  # Feedforward controller state

# Simulation loop
for k, t in enumerate(time_steps):
    if k % N == 0:  # Update control at controller rate
        error = ref - x_p[0, 0]

        # Compute feedforward control
        xff = Bff @ np.array([[ref]]) + xff
        u_ff = Cff @ xff  # Feedforward output

        # Compute feedback control (PID)
        xd = Bd @ error + xd
        u_fb = Cd @ xd  # PID output

        # Total control input
        u = u_fb + u_ff

    # Update plant
    x_p = Bz @ x_p + np.array([[u]])  # Plant dynamics

    # Logging
    y_p.append(x_p[0, 0])
    u_hist.append(u)

# Plot results
plt.figure()
plt.plot(time_steps, y_p, label='Plant Output')
plt.step(time_steps[::N], u_hist[::N], where='post', label='Total Control Input')
plt.axhline(ref, linestyle='--', color='gray', label='Reference')
plt.xlabel('Time (s)')
plt.ylabel('Response')
plt.legend()
plt.title('PID + Feedforward Control with Different Sampling Rates')
plt.grid()
plt.show()
