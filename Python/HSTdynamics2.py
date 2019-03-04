def HSTdynamics2(init_state, t, mu, J, J_inv, B_eci, m_max, power_max, Sun2Earth):
# HSTdynamics: Contains full ODE dynamics for full spacecraft state.
#
# Inputs:
#      t - time [s]
#      init_state - Initial State Vector [r; v; omega; q, torque, powercon, powergen]
#      mu - Central Body Gravitational Parameters [km^3/s^2]
#      J - Spacecraft Inertia Matrix
#      J_inv - Inverted Inertia Matrix
#      B_eci - Earth's Magnetic Field in ECI
#      m_max - Maximum Magnetic Moment of Torque Coils
#      power_max - Maximum Power Consumption due to Torque Coils
#      Sun2Earth - Unit Vector from Sun to Earth in ECI
#
# Outputs:
#      x_dot - linear equation containing ODEs for all states
    
    import numpy as np
    from subroutines import qkin
    from subroutines import qmult
    from subroutines import q2rot
    from sunflux import sunflux
    import pdb

    # Unpack Initial State
    rvec = init_state[:3]
    vvec = init_state[3:6]
    r = np.linalg.norm(rvec)
    om0 = init_state[6:9]
    q0 = init_state[9:13]
    torque0 = init_state[13:16]
    powercon0 = init_state[16:17]
    powergen0 = init_state[17:]

    # Rotate B_eci into Satellite Body Frame
    B_body = np.dot(q2rot(q0),B_eci)

    # Compute Torque using B-dot Control Law for Next Epoch
    B_dot = -np.cross(om0,B_body)                                               # Compute B_dot
    m_value = -np.multiply(m_max,np.sign(B_dot))*np.linalg.norm(np.tanh(om0))              # Direction of Magnetic Moment   
    torque = np.cross(m_value,B_body)                                           # Compute Torque
    torque = torque*(np.linalg.norm(om0)>1*np.pi/180)                                      # Turn off Torque within Omega Limits 
    
    powercon = np.sum(np.dot(np.square(m_value/m_max),power_max))      
    powergen = sunflux(np.transpose(rvec), Sun2Earth, np.transpose(q0))
 
    # matrix linear equation
    rv_dot = np.dot(np.block([[np.zeros((3,3)),      np.eye(3)],
                   [-(mu/r**3)*np.eye(3), np.zeros((3,3))]]), init_state[:6])
    om_dot = np.dot(J_inv, -np.cross(om0,np.dot(J,om0))+torque)
    q_dot = qkin(q0, om0)
    torque_dot = torque - torque0
    powercon_dot = powercon - powercon0
    powergen_dot = powergen - powergen0
    x_dot = np.concatenate((rv_dot, om_dot, q_dot, torque_dot, powercon_dot, powergen_dot))
    return x_dot



