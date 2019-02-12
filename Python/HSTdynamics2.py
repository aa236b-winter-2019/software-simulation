def HSTdynamics2(init_state, t, mu, J, torque):
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
    import numpy as np
    from subroutines import qkin
    from subroutines import qmult

 
    if init_state.size == 6: # linear position and velocity
        # unpack initial state
        rvec = init_state[:3]
        vvec = init_state[3:6]
        r = np.linalg.norm(rvec)

        # matrix linear equation
        rv_dot = np.dot(np.block([[np.zeros((3,3)),      np.eye(3)],
                           [-(mu/r**3)*np.eye(3), np.zeros((3,3))]]), init_state)
        x_dot = np.array(rv_dot)
    elif init_state.size == 7: # angular velocity and quaternion
        om0 = init_state[:3]
        q0 = init_state[3:7]
        torque=torque.reshape((3,))
        om_dot = np.dot(np.linalg.inv(J), -np.cross(om0,np.dot(J,om0))+torque.T).T
 
       # matrix linear equation
        q_dot = qkin(q0, om0)
        x_dot = np.append(om_dot, q_dot)
    return x_dot



