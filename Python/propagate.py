def propagate(hardware, deltaT):
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
#    import pdb

    J, J_inv, m_max, power_max = hardware.getHardwareProperties()
    
    init_state = hardware.state
    mjd = hardware.time
    m_value = hardware.m_value

    # Initialize Known Variables and Initial Conditions
    rv_eci0 = init_state[0:6]                      # Initial Orbit State Vector
    omq0 = init_state[6:]                          # Initial Angular Velocity (rad/s) and Quaternion    
    mu = 3.986e5                                   # Earth Standard Gravitational Parameter (km^3/s^2)
    
    # Calculate Earth's Magnetic Field in ECI
    t0 = julian.from_jd(mjd, fmt='mjd')            # Convert mjd into seconds
    B_eci = igrffx(rv_eci0[0:3],t0)*10**-9
    
    # Propogate State Vector [r, v, om, q] with Applied Torque for 1 Epoch
    # delta time of 1 s which is required by dynamics
    tspan = np.arange(0, deltaT+1, 1)
    rv_eci = odeint(dynamics, rv_eci0, tspan, args=(mu,J,J_inv,0,0,0, 0))
    print('mvalue: ' + str(m_value))	
    omq = odeint(dynamics, omq0, tspan, args=(mu,J,J_inv,B_eci,m_max, m_value, power_max)) # This propagates with torque coils ON (add flag later?)
    
    # Update New Initial Conditions and Store Propagated State
    r = rv_eci[-1,0:3]
    v = rv_eci[-1,3:6]
    om = omq[-1,0:3]
    q = omq[-1,3:7]
    torque = omq[-1, 7:10]
    power = omq[-1, 10:11]
        
    return np.concatenate((r, v, om, q, torque, power))
