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
#--------------------------- 
#---------------------------
#opts  = odeset('reltol', tol, 'abstol', tol)
tspan = np.linspace(0,5,500)


# Convert to Earth-Centered Inertial Frame Coordinates
r_eci, v_eci = OE2ECI(a, e, i, RAAN, w, anom, mu)
x0 = np.append(r_eci, v_eci)
rho = 0

# ODE45 Solver for Orbital Dynamics
tol = 1e-6
#------------------
#opts  = odeset('reltol', tol, 'abstol', tol)
tspan = np.arange(0, T, 10)
#opts------------------
x = odeint(HSTdynamics, x0, tspan, args=(mu,J,rho))

# Plot Orbit
plt.figure()
ax=plt.axes(projection='3d')
# plt.hold(True)
ax.plot3D(x[:,0],x[:,1],x[:,2],'r')
ax.axis('equal')
plt.xlabel('[km]')
plt.ylabel('[km]')
# plt.zlabel('[km]')
ax.set_title('HST Orbit Dynamics')
#####---------------------------------
#earthPlot(1)
ax.axis('equal')
# plt.hold(False)
'''
Ploting magnetic field
'''

#t0 = np.array([2019,1,28,12,37,20,20])
B=np.zeros((x.shape[0],3))

for j in range(tspan.shape[0]):

	mjd = 54372.78
	dt = julian.from_jd(mjd, fmt='mjd')
	newtime = dt + datetime.timedelta(seconds=tspan[j])
	B[j][:]=(igrffx(x[j][0:3],newtime).T)


    
#print(B)

fig = plt.figure()
ax = plt.axes( projection='3d')
ax.quiver(x[::10,0],x[::10,1],x[::10,2],B[::10,0],B[::10,1],B[::10,2])
ax.set_title('Magnetic field')
ax.axis('equal')
plt.show()
## Model HST Attitude Dynamics

# From Superspin
rho = 1.0e+05*np.array([0,0,3.6326])

# Initial Attitude Conditions
q0 = np.array([0,0,0,1]).T # Identity Quaternion
om0 = np.array([0.25,0.25,2.5]).T # rad/s
x0 = np.append(om0,q0)

# ODE45 Solver for Attitude Dynamics
dt = 0.01
tspan = np.arange(0, 15, dt)
tol = 1e-6

# opts  = odeset('reltol', tol, 'abstol', tol);
x = odeint(HSTdynamics, x0, tspan, args=(mu,J,rho))

# Plot Quaternion
q_true = x[:,3:7]
# plt.figure()
f,(ax1,ax2,ax3,ax4)=plt.subplots(4,1,sharey=True)
ax1.plot(tspan,q_true[:,0])
plt.ylabel('q_1')
plt.xlabel('t')
ax1.set_title('HST Quaternion Dynamics')
ax2.plot(tspan,q_true[:,1])
plt.ylabel('q_2')
plt.xlabel('t')
ax3.plot(tspan,q_true[:,2])
plt.ylabel('q_3')
plt.xlabel('t')
ax4.plot(tspan,q_true[:,3])
plt.ylabel('q_4')
plt.xlabel('t')

# Plot Omega
om_true = x[:,0:3]
# plt.figure()
f,(ax1,ax2,ax3)=plt.subplots(3,1,sharey=True)
ax1.plot(tspan,om_true[:,0])
plt.ylabel('\omega_1')
plt.xlabel('t')
ax1.set_title('HST Angular Velocity')
ax2.plot(tspan,om_true[:,1])
plt.ylabel('\omega_2')
plt.xlabel('t')
ax3.plot(tspan,om_true[:,2])
plt.ylabel('\omega_3')
plt.xlabel('t')

plt.show()
