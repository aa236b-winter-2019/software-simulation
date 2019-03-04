def propagate(hardware, deltaT, current_state):
# propagate: Propagates a specified state vector by a specified time increment
#
# Assumptions:
#       In Earth orbit
#
# Inputs:
#       init_state - Variable initial state vector [r; v; omega; q]
#       deltaT - Time increment to advance [s]
#       mjd - Mean Julian Date at the current epoch
#
# Outputs:
#       state - Propagated State
    import numpy as np
    import matplotlib.pyplot as plt
    from scipy.integrate import odeint
    from igrffx import igrffx
    from dynamics import dynamics
    import julian
    from sunlocate import sunlocate
    from subroutines import q2rot
    from sunflux import sunflux
#    import pdb

    J, J_inv, m_max, power_max = hardware.getHardwareProperties()

    baseline_power = hardware.power_draw_dict[current_state]
    
    init_state = hardware.state
    mjd = hardware.time
    m_value = hardware.m_value

    # Initialize Known Variables and Initial Conditions
    #rv_eci0 = init_state[0:6]                      # Initial Orbit State Vector
    #omq0 = init_state[6:]                          # Initial Angular Velocity (rad/s) and Quaternion    
    mu = 3.986e5                                   # Earth Standard Gravitational Parameter (km^3/s^2)
    
    # Calculate Earth's Magnetic Field in ECI
    t0 = julian.from_jd(mjd, fmt='mjd')            # Convert mjd into seconds
    B_eci = igrffx(init_state[0:3],t0)

    # Find the sun to earth vector
    Sun2Earth = sunlocate(mjd) 
    
    # Propogate State Vector [r, v, om, q] with Applied Torque for 1 Epoch
    # delta time of 1 s which is required by dynamics
    tspan = np.arange(0, deltaT+1, 1)
    #rv_eci = odeint(dynamics, rv_eci0, tspan, args=(mu,J,J_inv,0,0,0, 0))
    #print('mvalue: ' + str(m_value))	
    #omq = odeint(dynamics, omq0, tspan, args=(mu,J,J_inv,B_eci,m_max, m_value, power_max)) # This propagates with torque coils ON (add flag later?)

    state = odeint(dynamics, init_state, tspan, args=(mu,J,J_inv,B_eci,m_max, m_value, power_max, Sun2Earth, baseline_power))
    
    # Update New Initial Conditions and Store Propagated State
    r = state[-1,0:3]
    v = state[-1,3:6]
    om = state[-1,6:9]
    q = state[-1,9:13]
    torque = state[-1, 13:16]
    power = state[-1, 16:18]

    B_body = np.dot(q2rot(q),B_eci)
    torque = np.cross(m_value,B_body)[0]
    
    powercon = np.sum(np.dot(np.square(m_value/m_max),power_max)) + baseline_power      
    powergen = sunflux(np.transpose(r), Sun2Earth, np.transpose(q))
    
    power = [powercon, powergen]
        
    return np.concatenate((r, v, om, q, torque, power))
