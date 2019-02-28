def dynamics(init_state, t, mu, J, J_inv, B_eci, m_max, m_value, power_max):
# HSTdynamics: Contains full ODE dynamics for all spacecraft states.
#
# Assumptions:
#       THIS ONLY WORKS WITH 1 second time steps
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
    from subroutines import q2rot
#    import pdb

 
    if init_state.size == 6: # linear position and velocity
        # Unpack Initial State
        rvec = init_state[:3]
        vvec = init_state[3:6]
        r = np.linalg.norm(rvec)

        # Matrix Linear Equation
        rv_dot = np.dot(np.block([[np.zeros((3,3)),      np.eye(3)],
                           [-(mu/r**3)*np.eye(3), np.zeros((3,3))]]), init_state)
        x_dot = np.array(rv_dot)
    elif init_state.size == 11: # Angular Velocity, Quaternion, Torque, Power
        # Unpack Initial State
        om0 = init_state[:3]
        q0 = init_state[3:7]
        torque0 = init_state[7:10]
        power0 = init_state[10:]
        
        # Rotate B_eci into Satellite Body Frame
        B_body = np.dot(q2rot(q0),B_eci)
    
        # Compute Torque using B-dot Control Law for Next Epoch
        B_dot = -np.cross(om0,B_body)               # Compute B_dot
        #m_value = -np.multiply(m_max,np.sign(B_dot))*abs(np.tanh(om0))       # Direction of Magnetic Mom.
        
        torque = np.cross(m_value,B_body)      # Compute Torque
        #print('mvalue: ' + str(m_value))
        #torque = torque*(abs(om0)>1*np.pi/180)               # Turn off Torque within Omega Limits
        
        torque=torque.reshape((3,))
        
        power = np.sum((np.square(m_value/m_max) * power_max))
 
        # matrix linear equation
        om_dot = np.dot(J_inv, -np.cross(om0,np.dot(J,om0))+torque.T).T
        q_dot = qkin(q0, om0)
        torque_dot = torque - torque0
        power_dot = power - power0
        x_dot = np.concatenate((om_dot, q_dot, torque_dot, power_dot))
    return x_dot



