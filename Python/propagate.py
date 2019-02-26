def propagate(init_state, deltaT, mjd):
# propagate: Propagates a specified state vector by a specified time increment
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
    from HSTdynamics2 import HSTdynamics2
    import julian
    import pdb
    
    # Define Principle Axes of Inertia
    J11 = .01/6              # (kg m^2)
    J22 = .01/6              # (kg m^2)
    J33 = .01/6              # (kg m^2)
    J = np.diag(np.array([J11,J22,J33])) # Diagonalize Principle Axes
    J_inv = np.linalg.inv(J) # Invert J Matrix for Ease of Computation
    
    # Initialize the Magnetorquer Properties
    area_coil = np.array([0.8784,0.8784,0.8784])   # Magnetic area of coils (m sq)
    voltage_max = 8.4                              # max voltage to coils (volts)
    resistance  = np.array([178.4,178.4,135])      # resistance of coils (ohm)
    I_max = np.divide(voltage_max,resistance)      # Maximum current (A)
    m_max = I_max*area_coil                        # Maximum magnetic moment (A.msq)
    m_max = np.reshape(m_max,(1,3))
    power_max = voltage_max*I_max                  # Max power consumed (W)
    
    # Initialize Known Variables and Initial Conditions
    rv_eci0 = init_state[0:6]                      # Initial Orbit State Vector
    omq0 = init_state[6:]                          # Initial Angular Velocity (rad/s) and Quaternion    
    mu = 3.986e5                                   # Earth Standard Gravitational Parameter (km^3/s^2)
    
    # Calculate Earth's Magnetic Field in ECI
    t0 = julian.from_jd(mjd, fmt='mjd')            # Convert mjd into seconds
    B_eci = igrffx(rv_eci0[0:3],t0)*10**-9
    
    # Propogate State Vector [r, v, om, q] with Applied Torque for 1 Epoch	
    tspan = np.arange(0, deltaT, 1)
    rv_eci = odeint(HSTdynamics2, rv_eci0, tspan, args=(mu,J,J_inv,0,0,0))	
    omq = odeint(HSTdynamics2, omq0, tspan, args=(mu,J,J_inv,B_eci,m_max,power_max)) # This propagates with torque coils ON (add flag later?)
    
    # Update New Initial Conditions and Store Propagated State
    r = rv_eci[-1,0:3]
    v = rv_eci[-1,3:6]
    om = omq[-1,0:3]
    q = omq[-1,3:7]
        
    return np.concatenate((r, v, om, q))
