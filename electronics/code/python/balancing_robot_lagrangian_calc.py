# this script is used to check that my derivation of the equations 
# of motion of the two wheeled balancing robot are correct.

import sympy as sp

sp.init_printing(use_unicode=True)


def implicit_constraints_method():
    """
    This version involves implicitly including the constraints within the generalised coordinates
    """

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
    m_w = 0     # ETH EduBot uses this approximation
    vel_w = sp.sqrt(dx_w**2 + dy_w**2)                      # velocity of the wheel's center of mass
    T_w_trans_L = T_w_trans_R = 0.5 * m_w * vel_w**2        # translational kinetic energy of each wheel
    T_w_rot_L = T_w_rot_R = 0.5 * I_w * dpsi**2             # rotational kinetic energy of each wheel
    T_w = T_w_trans_L + T_w_trans_R + T_w_rot_L + T_w_rot_R     # total kinetic energy of the body

    T = T_b + T_w


    # calculate potential energy

    V_b = m_b * g * h * sp.cos(theta)       # potential energy of body
    V_w_L = V_w_R = 0                       # potential energy of each wheel
    V = V_b + V_w_L + V_w_R


    # calculate Lagrangian

    L = T - V
    L = sp.simplify(L)

    # plug into Euler-Lagrange equation to obtain system equations

    partial_L_by_partial_dtheta = sp.diff(L, dtheta)
    partial_L_by_partial_theta = sp.diff(L, theta)
    euler_lagrange = sp.diff(partial_L_by_partial_dtheta, t) - partial_L_by_partial_theta
    euler_lagrange = sp.simplify(euler_lagrange)


    # print(T_b_trans)
    # print(T_b_rot)
    print('T_w_trans_L:', T_w_trans_L)
    print('L:', L)
    print('euler_lagrange:', euler_lagrange)


    # answer from sympy: 
    # euler_lagrange = 1.0*I_b*Derivative(theta(t), (t, 2)) - 1.0*g*h*m_b*sin(theta(t)) + 1.0*h**2*m_b*Derivative(theta(t), (t, 2)) - 1.0*h*m_b*r*cos(theta(t))*Derivative(psi(t), (t, 2))
    #    = I_b * d_theta_by_dt2  -  g*h*m_b*sin(theta)  +  h^2 * m_b * d_theta_by_dt2  -  h*m_b*r*cos(theta) * d_psi_by_dt2


def explicit_constraints_method():
    """
    This version involves explicitly specifying the constraints as non-concervative forces
    """

    # create variables

    t = sp.symbols('t') #, real=True)      # time

    theta = sp.Function('theta')(t)
    x_w = sp.Function('x_w')(t)
    y_w = 0

    torque_L = sp.Function('torque_L')(t)
    torque_R = sp.Function('torque_R')(t)
    torque = torque_L + torque_R

    r, h, g, m_w, m_b, I_w, I_b, b_w = sp.symbols('r h g m_w m_b I_w I_b b_w')

    # map rotation angles to cartesian coords

    x_b = x_w + h * sp.sin(theta)
    y_b = h * sp.cos(theta)

    psi = x_w / r

    # x_w = -r * psi
    # y_w = 0

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
    # m_w = 0     # ETH EduBot uses this approximation
    vel_w = sp.sqrt(dx_w**2 + dy_w**2)                      # velocity of the wheel's center of mass
    T_w_trans_L = T_w_trans_R = 0.5 * m_w * vel_w**2        # translational kinetic energy of each wheel
    T_w_rot_L = T_w_rot_R = 0.5 * I_w * dpsi**2             # rotational kinetic energy of each wheel
    T_w = T_w_trans_L + T_w_trans_R + T_w_rot_L + T_w_rot_R     # total kinetic energy of the wheel
    # T_w = T_w_trans_L + T_w_trans_R             # total kinetic energy of the wheel: NB: this excludes rotational energy since that will be handled explicitly later

    T = sp.simplify(T_b + T_w)

    # print('T_b:', sp.simplify(T_b))
    # print('T_w/2:', T_w/2)

    # calculate potential energy

    V_b = m_b * g * h * sp.cos(theta)       # potential energy of body
    V_w_L = V_w_R = 0                       # potential energy of each wheel
    V = V_b + V_w_L + V_w_R


    # calculate Lagrangian

    L = T - V
    L = sp.simplify(L)

    # model dissipated energy from rolling wheels using Rayleigh dissipation function
    D_w = 0.5 * b_w * dpsi**2
    D = 2 * D_w     # two wheels

    # plug into Euler-Lagrange equation to obtain system equations
    # include non-convervative force here, i.e. motor torque

    partial_L_by_partial_dx = sp.diff(L, dx_w)
    partial_L_by_partial_x = sp.diff(L, x_w)
    partial_D_by_partial_dx = sp.diff(D, dx_w)
    Q_x_w = torque / r
    euler_lagrange_1 = sp.diff(partial_L_by_partial_dx, t) - partial_L_by_partial_x - Q_x_w + partial_D_by_partial_dx
    euler_lagrange_1 = sp.simplify(euler_lagrange_1)

    partial_L_by_partial_dtheta = sp.diff(L, dtheta)
    partial_L_by_partial_theta = sp.diff(L, theta)
    partial_D_by_partial_dtheta = sp.diff(D, dtheta)
    Q_theta = -torque       # NB: sign of torque is opposite, since psoitive torque decreases theta
    euler_lagrange_2 = sp.diff(partial_L_by_partial_dtheta, t) - partial_L_by_partial_theta - Q_theta + partial_D_by_partial_dtheta
    euler_lagrange_2 = sp.simplify(euler_lagrange_2)

    # print(T_b_trans)
    # print(T_b_rot)
    # print('T_w_trans_L:', T_w_trans_L)
    # print('L:', L)
    print('euler_lagrange_1:', euler_lagrange_1)
    print('euler_lagrange_2:', euler_lagrange_2)

    # answer from sympy: 
    # euler_lagrange_1: (2.0*I_w*Derivative(x_w(t), (t, 2)) + 2.0*b_w*Derivative(x_w(t), t) + r**2*(-1.0*h*m_b*sin(theta(t))*Derivative(theta(t), t)**2 + 1.0*h*m_b*cos(theta(t))*Derivative(theta(t), (t, 2)) + 1.0*m_b*Derivative(x_w(t), (t, 2)) + 2.0*m_w*Derivative(x_w(t), (t, 2))) - r*(torque_L(t) + torque_R(t)))/r**2
    # = (2 * I_w * d_xw_by_t2 + 2 * b_w * d_xw_by_t + r**2 * (-h * m_b * sin(theta) * (d_theta_by_t)**2 + h * m_b * cos(theta) * d_theta_by_t2 + m_b * d_xw_by_t2 + 2 * m_w * d_xw_by_t2) - r * (torque_L + torque_R)) / r**2


    # euler_lagrange_2: 1.0*I_b*Derivative(theta(t), (t, 2)) - 1.0*g*h*m_b*sin(theta(t)) + 1.0*h**2*m_b*Derivative(theta(t), (t, 2)) + 1.0*h*m_b*cos(theta(t))*Derivative(x_w(t), (t, 2)) + 1.0*torque_L(t) + 1.0*torque_R(t)
    #    = I_b * d_theta_by_t2 - g * h * m_b * sin(theta) + h**2 * m_b * d_theta_by_t2 + h * m_b * cos(theta) * d_xw_by_t2 + torque_L + torque_R
    #    = h * m_b * cos(theta) * d_xw_by_t2 + (h**2 * m_b + I_b) * d_theta_by_t2 - g * h * m_b * sin(theta) + torque_L + torque_R


if __name__ == "__main__":
    explicit_constraints_method()
