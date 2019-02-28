import numpy as np
import matplotlib.pyplot as plt
import pickle
from scipy.linalg import block_diag # for star sensor
from scipy.integrate import odeint
from mpl_toolkits.mplot3d import Axes3D
import julian
import datetime
from subroutines import *
from HSTdynamics import HSTdynamics
from HSTdynamics2 import HSTdynamics2
from propagate import propagate
from igrffx import igrffx
from sunlocate import sunlocate
from sunflux import sunflux
import pdb
plt.close('all')

# Orbital Elements and Parameters
a = 6917.50                                                                     # Orbit Semi-Major Axis (km)
e = 0.000287                                                                    # Eccentricity
i = 28.47                                                                       # Inclination (deg)
RAAN = 176.23                                                                   # Right Ascension of Ascending Node (deg)
w = 82.61                                                                       # Argument of Perigee (deg)
anom = 319.41                                                                   # Mean Anomaly (deg)
mu = 3.986e5                                                                    # Earth Standard Gravitational Parameter (km^3/s^2)
T = 2*np.pi*np.sqrt((a**3)/mu)                                                  # Orbital Period (s)

# Define Principle Axes of Inertia
J11 = .01/6                                                                     # (kg m^2)
J22 = .01/6                                                                     # (kg m^2)
J33 = .01/6                                                                     # (kg m^2)
J = np.diag(np.array([J11,J22,J33]))                                            # Diagonalize Principle Axes
J_inv = np.linalg.inv(J)                                                        # Invert J Matrix for Ease of Computation

# Convert Orbital ELements to Earth-Centered Inertial Frame Coordinates
r, v = OE2ECI(a, e, i, RAAN, w, anom, mu)                                       # Initial Orbit State Vector
om = 0.25*np.array([1,1,1])                                                     # Initial Angular Velocity (rad/s)
q = np.array([0,0,1,0])                                                         # Initial Quaternion, Identity
torque = np.array([0,0,0])                                                      # Initial Torque
powercon = np.array([0])                                                        # Initial Power Consumption
powergen = np.array([0])                                                        # Initial Power Generation
init_state = np.concatenate((r,v,om,q,torque,powercon,powergen))                # Initial Attitude State Vector
tspan = [0]                                                                     # Start Time (sec)  
n = 100                                                                         # Epochs per Orbital Period
epochs = 100                                                                    # Total Number of Epochs
m_value= np.zeros((1,3))                                                        # Initial magnetic moment

# Calculate Earth's Magnetic Field and Sun's Location in ECI
mjd = 54372.78                                                                  # Mean Julian Date of Initial Epoch
t0 = julian.from_jd(mjd, fmt='mjd')                                             # Convert mjd into seconds
B_eci = igrffx(r,t0)*10**-9                                                     # Earth's Magnetic Field in ECI
Sun2Earth = sunlocate(mjd)                                                      # Unit Vector from Sun to Earth in ECI

# Initialize the Magnetorquer Properties
area_coil = np.array([0.8784,0.8784,0.8784])                                    # Enclosed Coil Area in Each Axis (m^2)
n_coil = np.array([1, 1, 1])                                                    # Number of Coil Turns in Each Axis
voltage_max = 8.4                                                               # Max Battery Voltage (V)
resistance  = np.array([178.4,178.4,135])                                       # Coil Resistance (Ohm)
I_max = np.divide(voltage_max,resistance)                                       # Max Current (A)
m_max = I_max*np.multiply(area_coil,n_coil)                                     # Max Magnetic Moment (A m^2)
power_max = voltage_max*I_max                                                   # Max Power Consumed (W)
energy_consumed = 0                                                             # Initializing total energy consumption (J)

# Initialize Zero Arrays for Time History  
r_hist = np.zeros((1,3))                                                        # Time History of Position
v_hist = np.zeros((1,3))                                                        # Time History of Velocity
om_hist = np.zeros((1,3))                                                       # Time History of Angular Velocity 
q_hist = np.zeros((1,4))                                                        # Time History of Quaternion
torque_hist = np.zeros((1,3))                                                   # Time History of Torque
Pcon_hist = []                                                                  # Time history of Power Consumption
Pgen_hist = []                                                                  # Time history of Power Generation
Tspan = []                                                                      # Time History 

for i in range (1,epochs):
    # Propogate State Vector with Applied Torque for 1 Epoch
    tspan = np.arange(tspan[-1], T/n*i, 1)
    args = (mu,J,J_inv,B_eci,m_max,power_max,Sun2Earth)
    state = odeint(HSTdynamics2, init_state, tspan, args)
    
    # Store Time History for Plotting
    r_hist = np.concatenate((r_hist,state[:,0:3]),axis=0)
    v_hist = np.concatenate((v_hist,state[:,3:6]),axis=0)
    om_hist = np.concatenate((om_hist,state[:,6:9]),axis=0)
    q_hist = np.concatenate((q_hist,state[:,9:13]),axis=0)
    torque_hist = np.concatenate((torque_hist,state[:,13:16]),axis=0)
    Pcon_hist = np.append(Pcon_hist,state[:,16:17])
    Pgen_hist = np.append(Pgen_hist,state[:,17:])
    Tspan = np.concatenate((Tspan,tspan),axis=0)
    
    # Update the Inital Conditions for Next Epoch
    init_state = np.concatenate((r_hist[-1,:],v_hist[-1,:],om_hist[-1,:],
                             q_hist[-1,:],torque_hist[-1,:],((Pcon_hist[-1], Pgen_hist[-1]))))

    # Calculate Earth's Magnetic Field in ECI
    j = tspan.shape[0]-1                                                        # Final Index of tspan
    dt = julian.from_jd(mjd, fmt='mjd')                                         # Convert mjd into seconds
    newtime = dt + datetime.timedelta(seconds=tspan[j])                         # Add time from Epoch 
    mjd_prop = 54372.78 + (tspan[j])/(60*60*24)                                 # newtime in mjd
    B_eci = igrffx(init_state[0:3],newtime)*10**-9                              # New B_eci for Next Time Step 
    Sun2Earth=sunlocate(mjd_prop)                                               # New Sun2Earth for Next Time Step
    

# Plot Orbit 
plt.figure()
ax=plt.axes(projection='3d')
ax.plot3D(r_hist[1:,0],r_hist[1:,1],r_hist[1:,2],'r')
ax.axis('equal')
plt.xlabel('[km]')
plt.ylabel('[km]')
ax.set_title('Detumbling Orbit Dynamics')
plt.show()

# Plot Quaternion
f,(ax1,ax2,ax3,ax4)=plt.subplots(4,1,sharey=True)
plt.rcParams['axes.grid'] = True
ax1.plot(Tspan/60,q_hist[1:,0])
plt.ylabel('q_1')
plt.xlabel('t(min)')
ax1.set_title('Quaternion Dynamics')
ax2.plot(Tspan/60,q_hist[1:,1])
plt.ylabel('q_2')
plt.xlabel('t(sec)')
ax3.plot(Tspan/60,q_hist[1:,2])
plt.ylabel('q_3')
plt.xlabel('t(min)')
ax4.plot(Tspan/60,q_hist[1:,3])
plt.ylabel('q_4')
plt.xlabel('t(min)')
plt.show()

# Plot Omega
f,(ax1,ax2,ax3)=plt.subplots(3,1,sharey=True)
plt.rcParams['axes.grid'] = True
ax1.plot(Tspan/60,om_hist[1:,0])
plt.ylabel('\omega_1')
plt.xlabel('t(min)')
ax1.set_title('Angular Velocity with Bdot controller')
ax2.plot(Tspan/60,om_hist[1:,1])
plt.ylabel('\omega_2')
plt.xlabel('t(min)')
ax3.plot(Tspan/60,om_hist[1:,2])
plt.ylabel('\omega_3')
plt.xlabel('t(min)')
plt.show()

# Plot Torque
Tspan_torque = np.arange(0, T/n*epochs, T/n)
f,(ax1,ax2,ax3)=plt.subplots(3,1,sharey=True)
plt.rcParams['axes.grid'] = True
ax1.plot(Tspan/60,torque_hist[1:,0])
plt.ylabel('Tx()')
plt.xlabel('t(min)')
ax1.set_title('Torque')
ax2.plot(Tspan/60,torque_hist[1:,1])
plt.ylabel('Ty()')
plt.xlabel('t(min)')
ax3.plot(Tspan/60,torque_hist[1:,2])
plt.ylabel('Tz()')
plt.xlabel('t(min)')
plt.show()

# Plot Power and Compute Energy Consumption
energy_consumed = np.trapz(Pcon_hist, Tspan)
print('Energy Consumed = ', energy_consumed)
plt.figure()
plt.plot(Tspan/60,Pcon_hist,'r')
plt.xlabel('Time (min)')
plt.ylabel('Power Consumed (W)')
plt.title('Power Consumption ')
plt.show()

# Plot Power Generation
plt.figure()
plt.plot(Tspan/60,Pgen_hist,'r')
plt.xlabel('Time (min)')
plt.ylabel('Power Generated (W)')
plt.title('Power Generation')
plt.show()
