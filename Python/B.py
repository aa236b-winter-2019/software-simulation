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
J11 = .01/6
J22 = .01/6
J33 = .01/6
J=np.array([J11,J22,J33])
J = np.diag(J) # [x-axis z-axis y-axis]

# Convert to Earth-Centered Inertial Frame Coordinates
r_eci, v_eci = OE2ECI(a, e, i, RAAN, w, anom, mu)
torque = np.zeros((1,3)).T
q = np.zeros((1,4)).T # Identity Quaternion
q[3]=1
om = 0.25*np.ones((1,3)).T # rad/s
tspan=[0]
B=np.zeros((1,3))
r_total = np.zeros((1,3))
v_total = np.zeros((1,3))
om_total = np.zeros((1,3))
q_total = np.zeros((1,4))
Torque=np.zeros((1,3))
Tspan=[]
m_max=5*np.ones((1,3))
i=1
for i in range (1,3):
	print (i)
	tspan = np.arange(tspan[0], 0.05*T*i, 10)
	if i==1:
		rv_eci0 = np.append(r_eci, v_eci)
		omq0 = np.append(om,q)
	else:
		rv_eci0 = np.append(r_total[-1,:], v_total[-1,:])
		omq0 = np.append(om_total[-1,:],q_total[-1,:])

	rv_eci = odeint(HSTdynamics2, rv_eci0, tspan, args=(mu,J,0))	
	omq = odeint(HSTdynamics2, omq0, tspan, args=(mu,J,torque.reshape(1,3)))
	r_total = np.concatenate((r_total,rv_eci[:,0:3]),axis=0)
	v_total = np.concatenate((v_total,rv_eci[:,3:6]),axis=0)
	om_total = np.concatenate((om_total,omq[:,0:3]),axis=0)
	q_total = np.concatenate((q_total,omq[:,3:7]),axis=0)
	Tspan=np.concatenate((Tspan,tspan),axis=0)

	q_current = omq[-1,3:7]
	om_current=omq[-1,:3]

	j= tspan.shape[0]-1
	mjd = 54372.78
	dt = julian.from_jd(mjd, fmt='mjd')
	newtime = dt + datetime.timedelta(seconds=tspan[j])
	B_eci=(igrffx(rv_eci[j][0:3],newtime).T)

 
	B_body = np.dot(q2rot(q_current),B_eci)
	B_dot = -np.cross(om_current,B_body)
	m_value = -np.multiply(m_max,np.sign(B_dot))
	torque = np.cross(m_value,B_body)
	torque=np.matmul(np.linalg.inv(J),torque.reshape(3,1))
#	Torque=np.concatenate((Torque,torque.reshape(1,3)),axis=0)

	#torque = np.zeros((1,3)).T



# Plot Orbit 
plt.figure()
ax=plt.axes(projection='3d')
ax.plot3D(r_total[1:,0],r_total[1:,1],r_total[1:,2],'r')
ax.axis('equal')
plt.xlabel('[km]')
plt.ylabel('[km]')
ax.set_title('Detumbling Orbit Dynamics')
plt.show()


# plt.figure()
f,(ax1,ax2,ax3,ax4)=plt.subplots(4,1,sharey=True)
ax1.plot(Tspan,q_total[1:,0])
plt.ylabel('q_1')
plt.xlabel('t')
ax1.set_title('HST Quaternion Dynamics')
ax2.plot(Tspan,q_total[1:,1])
plt.ylabel('q_2')
plt.xlabel('t')
ax3.plot(Tspan,q_total[1:,2])
plt.ylabel('q_3')
plt.xlabel('t')
ax4.plot(Tspan,q_total[1:,3])
plt.ylabel('q_4')
plt.xlabel('t')
plt.show()

# Plot Omega

f,(ax1,ax2,ax3)=plt.subplots(3,1,sharey=True)
ax1.plot(om_total[1:,0])
plt.ylabel('\omega_1')
plt.xlabel('t')
ax1.set_title('Angular Velocity with Bdot controller')
ax2.plot(om_total[1:,1])
plt.ylabel('\omega_2')
plt.xlabel('t')
ax3.plot(om_total[1:,2])
plt.ylabel('\omega_3')
plt.xlabel('t')
plt.show()
'''
plt.figure()
plt.plot(Torque[:,0], 'r--', Torque, Torque[:,1], 'bs')
plt.show()
'''
