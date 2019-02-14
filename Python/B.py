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
omq0 = np.append(om,q)              # Initial Attitude State Vector
om = 0.25*np.ones((1,3)).T          # Initial Angular Velocity (rad/s)
q = np.array([0,0,1,0]).T           # Initial Quaternion, Identity
torque = np.zeros((1,3)).T          # Initial Torque
m_max = 0.5*np.ones((1,3))*10**-10  # Maximum Magnetic Moment in each Axis
n = 100                             # Epochs per Orbital Period
epochs = 100                        # Total Number of Epochs
tspan = [0]                         # Start Time (sec)

# Initialize Zero Arrays for Time History  
r_hist = np.zeros((1,3))            # Time History of Position
v_hist = np.zeros((1,3))            # Time History of Velocity
om_hist = np.zeros((1,3))           # Time History of Angular Velocity 
q_hist = np.zeros((1,4))            # Time History of Quaternion
torque_hist = np.zeros((1,3))       # Time History of Torque
Tspan = []                          # Time History 

for i in range (1,epochs):
    # Propogate State Vector [r, v, om, q] with Applied Torque for 1 Epoch	
    tspan = np.arange(tspan[-1], T/n*i, 1)
    rv_eci = odeint(HSTdynamics2, rv_eci0, tspan, args=(mu,J,J_inv,0))	
    omq = odeint(HSTdynamics2, omq0, tspan, args=(mu,J,J_inv,torque))
    
    # Store Time History for Plotting
    r_hist = np.concatenate((r_hist,rv_eci[:,0:3]),axis=0)
    v_hist = np.concatenate((v_hist,rv_eci[:,3:6]),axis=0)
    om_hist = np.concatenate((om_hist,omq[:,0:3]),axis=0)
    q_hist = np.concatenate((q_hist,omq[:,3:7]),axis=0)
    Tspan = np.concatenate((Tspan,tspan),axis=0)
    
    # Update the Inital Conditions for Next Epoch
    rv_eci0 = np.append(r_hist[-1,:], v_hist[-1,:])
    omq0 = np.append(om_hist[-1,:],q_hist[-1,:])
    
    # Calculate Earth's Magnetic Field in ECI
    j = tspan.shape[0]-1   # Final Index of tspan
    mjd = 54372.78         # Mean Julian Date of Initial Epoch
    dt = julian.from_jd(mjd, fmt='mjd') # Convert mjd into seconds
    newtime = dt + datetime.timedelta(seconds=tspan[j]) # Add time from Epoch
    B_eci = igrffx(rv_eci[j][0:3],newtime) # Look up ECI Magnetic Field
    
    # Rotate B_eci into Satellite Body Frame B_body
    q_current = q_hist[-1,:]  
    om_current = om_hist[-1,:]
    B_body = np.dot(q2rot(q_current),B_eci)
    
    # Compute Torque using B-dot Control Law for Next Epoch
    B_dot = -np.cross(om_current,B_body) # Compute B_dot
    m_value = -np.multiply(m_max,np.sign(B_dot)) # Direction of Magnetic Mom.
    torque = np.cross(m_value,B_body)*np.tanh(w) # Compute Torque
    torque = torque*(abs(w)>1*np.pi/180) # Turn off Torque within Omega Limits
    torque_hist = np.concatenate((torque_hist,torque),axis=0) # Store History

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
ax1.plot(Tspan,q_hist[1:,0])
plt.ylabel('q_1')
plt.xlabel('t(sec)')
ax1.set_title('Quaternion Dynamics')
ax2.plot(Tspan,q_hist[1:,1])
plt.ylabel('q_2')
plt.xlabel('t(sec)')
ax3.plot(Tspan,q_hist[1:,2])
plt.ylabel('q_3')
plt.xlabel('t(sec)')
ax4.plot(Tspan,q_hist[1:,3])
plt.ylabel('q_4')
plt.xlabel('t(sec)')
plt.show()

# Plot Omega
f,(ax1,ax2,ax3)=plt.subplots(3,1,sharey=True)
plt.rcParams['axes.grid'] = True
ax1.plot(Tspan,om_hist[1:,0])
plt.ylabel('\omega_1')
plt.xlabel('t(sec)')
ax1.set_title('Angular Velocity with Bdot controller')
ax2.plot(Tspan,om_hist[1:,1])
plt.ylabel('\omega_2')
plt.xlabel('t(sec)')
ax3.plot(Tspan,om_hist[1:,2])
plt.ylabel('\omega_3')
plt.xlabel('t(sec)')
plt.show()

# Plot Torque
Tspan_torque = np.arange(0, T/n*epochs, T/n)
f,(ax1,ax2,ax3)=plt.subplots(3,1,sharey=True)
plt.rcParams['axes.grid'] = True
ax1.plot(Tspan_torque,torque_hist[:,0])
plt.ylabel('Tx()')
plt.xlabel('t')
ax1.set_title('Torque')
ax2.plot(Tspan_torque,torque_hist[:,1])
plt.ylabel('Ty()')
plt.xlabel('t')
ax3.plot(Tspan_torque,torque_hist[:,2])
plt.ylabel('Tz()')
plt.xlabel('t')
plt.show()
