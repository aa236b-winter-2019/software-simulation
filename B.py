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
from igrffx import igrffx
plt.close('all')

## Initialize and Save Constant Parameters
# Orbital Elements and Parameters
a = 6917.50             # Orbit Semi-Major Axis (km)
e = 0.000287            # Eccentricity
i = 28.47               # Inclination (deg)
RAAN = 176.23           # Right Ascension of Ascending Node (deg)
w = 82.61               # Argument of Perigee (deg)
anom = 319.41           # Mean Anomaly (deg)
mu = 3.986e5            # Earth Standard Gravitational Parameter (km^3/s^2)
T = 2*np.pi*np.sqrt((a**3)/mu) # Orbital Period (s)

# Mass Properties, Principle Axes of Inertia
J11 = 28525.53
J22 = 174815.86
J33 = 181630.81
J=np.array([J11,J22,J33])
J = np.diag(J) # [x-axis z-axis y-axis]

# Compute Perturbed Attitude Dynamics
tol = 1e-6

# Convert to Earth-Centered Inertial Frame Coordinates
r_eci, v_eci = OE2ECI(a, e, i, RAAN, w, anom, mu)
torque = np.array([0,0,0]).T
q = np.array([0,0,0,1]).T # Identity Quaternion
om = np.array([0.25,0.25,2.5]).T # rad/s
tspan=[0]
B=np.zeros((1,3))
r_total = np.array([])
v_total = np.array([])
om_total = np.array([])
q_total = np.array([])

i=1
for i in range (1,10):
	tspan = np.arange(tspan[0], 0.1*T*i, 10)
	
	rv_eci0 = np.append(r_eci[-1], v_eci[-1])
	rv_eci = odeint(HSTdynamics, rv_eci0, tspan, args=(mu,J,torque))
	omq0 = np.append(om[-1],q[-1])
	omq = odeint(HSTdynamics, omq0, tspan, args=(mu,J,torque))
	r_total = np.append(r_total,rv_eci[:,:3])
	v_total = np.append(v_total,rv_eci[:,3:])
	om_total = np.append(om_total,omq_eci[:,:3])
	q_total = np.append(q_total,omq_eci[:,3:])

	q_current = omq[-1,3:7]
	om_current=omq[-1,:3]

	j= tspan.shape[0]-1
	mjd = 54372.78
	dt = julian.from_jd(mjd, fmt='mjd')
	newtime = dt + datetime.timedelta(seconds=tspan[j])
	B=(igrffx(rv_eci[j][0:3],newtime).T)

	u_max=10
	B = np.dot(q2rot(q_current),B)
	B_moment_value = u_max*np.tanh(np.dot(om.T,B))
	B_moment_direction = np.cross(om,B) 
	B_moment_direction = B_moment_direction/np.linalg.norm(B_moment_direction)
	B_moment = B_moment_value * B_moment_direction
	torque = np.cross(B_moment,B)
	torque=np.matmul(np.linalg.inv(J),torque)
	torque = np.array([0,0,0]).T




# Plot Orbit 
plt.figure()
ax=plt.axes(projection='3d')
ax.plot3D(r_total[:,0],r_total[:,1],r_total[:,2],'r')
ax.axis('equal')
plt.xlabel('[km]')
plt.ylabel('[km]')
ax.set_title('Detumbling Orbit Dynamics')
plt.show()


# Plot Omega
plt.figure()
f,(ax1,ax2,ax3)=plt.subplots(3,1,sharey=True)
ax1.plot(tspan,om_total[:,0])
plt.ylabel('\omega_1')
plt.xlabel('t')
ax1.set_title('Angular Velocity wiht Bdot controller')
ax2.plot(tspan,om_total[:,1])
plt.ylabel('\omega_2')
plt.xlabel('t')
ax3.plot(tspan,om_total[:,2])
plt.ylabel('\omega_3')
plt.xlabel('t')
plt.show()
