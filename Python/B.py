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
import pdb
plt.close('all')

# Orbital Elements and Parameters
a = 6917.50              # Orbit Semi-Major Axis (km)
e = 0.000287             # Eccentricity
i = 28.47                # Inclination (deg)
RAAN = 176.23            # Right Ascension of Ascending Node (deg)
w = 82.61                # Argument of Perigee (deg)
anom = 319.41            # Mean Anomaly (deg)
mu = 3.986e5             # Earth Standard Gravitational Parameter (km^3/s^2)
T = 2*np.pi*np.sqrt((a**3)/mu) # Orbital Period (s)

# Define Principle Axes of Inertia
J11 = .01/6              # (kg m^2)
J22 = .01/6              # (kg m^2)
J33 = .01/6              # (kg m^2)
J = np.diag(np.array([J11,J22,J33])) # Diagonalize Principle Axes
J_inv = np.linalg.inv(J) # Invert J Matrix for Ease of Computation

# Convert Orbital ELements to Earth-Centered Inertial Frame Coordinates
r_eci, v_eci = OE2ECI(a, e, i, RAAN, w, anom, mu)

# Initialize Known Variables and Initial Conditions
rv_eci0 = np.append(r_eci, v_eci)   # Initial Orbit State Vector
om = 0.25*np.array([1,1,1])         # Initial Angular Velocity (rad/s)
q = np.array([0,0,1,0])             # Initial Quaternion, Identity
torque = np.array([0,0,0])          # Initial Torque
power = np.array([0])               # Initial Power
omqtp0 = np.concatenate((om,q,torque,power)) # Initial Attitude State Vector    
#m_max = 0.5*np.ones((1,3))*10**-10  # Maximum Magnetic Moment in each Axis
n = 100                             # Epochs per Orbital Period
epochs = 100                        # Total Number of Epochs
tspan = [0]                         # Start Time (sec)
m_value= np.zeros((1,3))            # Initial magnetic moment

# Calculate Earth's Magnetic Field in ECI
mjd = 54372.78                                      # Mean Julian Date of Initial Epoch
t0 = julian.from_jd(mjd, fmt='mjd')                 # Convert mjd into seconds
B_eci = igrffx(rv_eci0[0:3],t0)*10**-9

# Initialize the Magnetorquer Properties
area_coil = np.array([0.8784,0.8784,0.8784])   # Magnetic area of coils (m sq)
voltage_max = 8.4                              # max voltage to coils (volts)
resistance  = np.array([178.4,178.4,135])      # resistance of coils (ohm)
I_max = np.divide(voltage_max,resistance)      # Maximum current (A)
m_max = I_max*area_coil                        # Maximum magnetic moment (A.msq)
m_max = np.reshape(m_max,(1,3))
power_max = voltage_max*I_max                  # Max power consumed (W)
energy_consumed = 0                            # Initializing total energy consumption (J)

# Initialize Zero Arrays for Time History  
r_hist = np.zeros((1,3))            # Time History of Position
v_hist = np.zeros((1,3))            # Time History of Velocity
om_hist = np.zeros((1,3))           # Time History of Angular Velocity 
q_hist = np.zeros((1,4))            # Time History of Quaternion
torque_hist = np.zeros((1,3))       # Time History of Torque
Tspan = []                          # Time History 
P_hist = []                       # Time history of power consumption

for i in range (1,epochs):
    # Propogate State Vector [r, v, om, q] with Applied Torque for 1 Epoch	
    tspan = np.arange(tspan[-1], T/n*i, 1)
    rv_eci = odeint(HSTdynamics2, rv_eci0, tspan, args=(mu,J,J_inv,0,0,0))	
    omqtp = odeint(HSTdynamics2, omqtp0, tspan, args=(mu,J,J_inv,B_eci,m_max,power_max))
    
    # Store Time History for Plotting
    r_hist = np.concatenate((r_hist,rv_eci[:,0:3]),axis=0)
    v_hist = np.concatenate((v_hist,rv_eci[:,3:6]),axis=0)
    om_hist = np.concatenate((om_hist,omqtp[:,0:3]),axis=0)
    q_hist = np.concatenate((q_hist,omqtp[:,3:7]),axis=0)
    torque_hist = np.concatenate((torque_hist,omqtp[:,7:10]),axis=0)
    P_hist = np.append(P_hist,omqtp[:,10:])
    Tspan = np.concatenate((Tspan,tspan),axis=0)
    
    # Update the Inital Conditions for Next Epoch
    rv_eci0 = np.concatenate((r_hist[-1,:], v_hist[-1,:]))
    omqtp0 = np.concatenate((om_hist[-1,:],q_hist[-1,:],torque_hist[-1,:],np.reshape(P_hist[-1],(1,))))
    
    # Compute energy that will be consumed by applying this torque in this epoch
    #power = np.sum(np.dot(np.square(m_value/m_max),power_max))
    #energy_consumed += power*(T/n)
    #P_hist = np.append(P_hist,power)

    # Calculate Earth's Magnetic Field in ECI
    j = tspan.shape[0]-1                                # Final Index of tspan
    mjd = 54372.78                                      # Mean Julian Date of Initial Epoch
    dt = julian.from_jd(mjd, fmt='mjd')                 # Convert mjd into seconds
    newtime = dt + datetime.timedelta(seconds=tspan[j]) # Add time from Epoch
    B_eci = igrffx(rv_eci[j][0:3],newtime)*10**-9       # Look up ECI Magnetic Field (T)
    
    # Rotate B_eci into Satellite Body Frame B_body
    #q_current = q_hist[-1,:]  
    #om_current = om_hist[-1,:]
    #B_body = np.dot(q2rot(q_current),B_eci)
    
    # Compute Torque using B-dot Control Law for Next Epoch
    #B_dot = -np.cross(om_current,B_body)               # Compute B_dot
    #m_value = -np.multiply(m_max,np.sign(B_dot))*abs(np.tanh(om_current))       # Direction of Magnetic Mom.
    
    #torque = np.cross(m_value,B_body)      # Compute Torque
    #torque = torque*(abs(om_current)>1*np.pi/180)               # Turn off Torque within Omega Limits
    #torque_hist = np.concatenate((torque_hist,torque),axis=0) # Store History
    

#print("Total Energy consumed for detumbling: "+str(energy_consumed)+ " J")
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
energy_consumed = np.trapz(P_hist, Tspan)
print(energy_consumed)
plt.figure()
plt.plot(Tspan/60,P_hist,'r')
plt.xlabel('Time (min)')
plt.ylabel('Power consumed (W)')
plt.title('Power consumption ')
plt.show()