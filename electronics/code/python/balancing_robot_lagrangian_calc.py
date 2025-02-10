# this script is used to check that my derivation of the equations 
# of motion of the two wheeled balancing robot are correct.

import sympy as sp

sp.init_printing(use_unicode=True)

# create variables

t = sp.symbols('t') #, real=True)      # time

theta = sp.Function('theta')(t)
psi = sp.Function('psi')(t)

r, h, g, m_w, m_b, I_w, I_b = sp.symbols('r h g m_w m_b I_w I_b')

# map rotation angles to cartesian coords

x_b = -psi * r + h * sp.sin(theta)
y_b = h * sp.cos(theta)

x_w = -r * psi
y_w = 0

# define first derivatives (velocities)
dx_b = sp.diff(x_b, t)
dy_b = sp.diff(y_b, t)

dx_w = sp.diff(x_w, t)
dy_w = sp.diff(y_w, t)

dtheta = sp.diff(theta, t)
dpsi = sp.diff(psi, t)


# calculate kinetic energy

# body
vel_b = sp.sqrt(dx_b**2 + dy_b**2)      # velocity of the body's center of mass
T_b_trans = 0.5 * m_b * vel_b**2        # translational kinetic energy of body
T_b_rot = 0.5 * I_b * dtheta**2         # rotational kinetic energy of body
T_b = T_b_trans + T_b_rot               # total kinetic energy of the body

# wheel(s)
vel_w = sp.sqrt(dx_w**2 + dy_w**2)                      # velocity of the wheel's center of mass
T_w_trans_L = T_w_trans_R = 0.5 * m_w * vel_w**2        # translational kinetic energy of each wheel
T_w_rot_L = T_w_rot_R = 0.5 * I_w * dpsi**2             # rotational kinetic energy of each wheel
T_w = T_w_trans_L + T_w_trans_R + T_w_rot_L             # total kinetic energy of the body

T = T_b + T_w


# calculate potential energy

V_b = m_b * g * h * sp.cos(theta)       # potential energy of body
V_w_L = V_w_R = 0                       # potential energy of each wheel
V = V_b + V_w_L + V_w_R


# calculate Lagrangian

L = T - V

# plug into Euler-Lagrange equation to obtain system equations

partial_L_by_partial_dtheta = sp.diff(L, dtheta)
partial_L_by_partial_theta = sp.diff(L, theta)
euler_lagrange = sp.diff(partial_L_by_partial_dtheta, t) - partial_L_by_partial_theta


# print(T_b_trans)
# print(T_b_rot)
# print('T_w_L:', T_w_L)
print('L:', L)
print('euler_lagrange:', euler_lagrange)


