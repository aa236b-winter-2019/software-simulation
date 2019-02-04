def HSTdynamics(init_state, t, mu, J, rho):
# HSTdynamics: Contains full ODE dynamics for all spacecraft states.
#
# Inputs:
#      t - time [s]
#      init_state - initial state vector [r; v; omega; q]
#      mu - central body gravitational parameters [km^3/s^2]
#      J - spacecraft inertia matrix
#      rho - gyro momentum
#
# Outputs:
#   x_dot - linear equation containing ODEs for all states

    # TODO: renormalize q
    import numpy as np
    from subroutines import qkin

    if init_state.size == 3: # angular velocity
        # unpack initial state
        om0 = init_state

        # matrix linear equation
        om_dot = -np.dot(np.linalg.inv(J), np.cross(om0, J.dot(om0) + rho))
        x_dot = om_dot
    elif init_state.size == 6: # linear position and velocity
        # unpack initial state
        rvec = init_state[:3]
        vvec = init_state[3:6]
        r = np.linalg.norm(rvec)

        # matrix linear equation
        rv_dot = np.dot(np.block([[np.zeros((3,3)),      np.eye(3)],
                           [-(mu/r**3)*np.eye(3), np.zeros((3,3))]]), init_state)
        x_dot = np.array(rv_dot)
    elif init_state.size == 7: # angular velocity and quaternion
        # unpack initial state
        om0 = init_state[:3]
        q0 = init_state[3:7]

        # matrix linear equation
        om_dot = -np.dot(np.linalg.inv(J), np.cross(om0, J.dot(om0) + rho))
        q_dot = qkin(q0, om0)
        x_dot = np.append(om_dot, q_dot)
    elif init_state.size == 13: # all states
        om0 = init_state[:3]
        q0 = init_state[3:7]
        rvec = init_state[7:10]
        vvec = init_state[10:13]   
        qhat = qmult(q0)
        r = np.linalg.norm(rvec)

        # since flag == 0
        Tau_g = np.zeros((3,1))
        D = np.zeros((3,1))
        Tau_D = np.zeros((3,1))
        D = Q*D/1000 # rotate drag into interial frame
        M = 11100 # mass of HST [kg]

        # matrix linear equation
        om_dot = np.dot(np.linalg.inv(J), Tau_g + Tau_D - np.cross(om0, J.dot(om0) + rho))
        q_dot = (1/2)*np.dot(qhat, np.append(om0, 0))

        rv_dot = np.dot(np.block([[np.zeros((3,3)),      np.eye(3)],
                           [-(mu/r**3)*np.eye(3), np.zeros((3,3))]]), np.append(rvec, vvec)) \
                - np.append(np.zeros((3,3)), np.eye(3), axis=1) * D/M
        x_dot = np.append(om_dot, q_dot, rv_dot)

    return x_dot










