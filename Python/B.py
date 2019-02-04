import numpy as np
import matplotlib.pyplot as plt
import pickle
from scipy.linalg import block_diag # for star sensor
from scipy.integrate import odeint
from mpl_toolkits.mplot3d import Axes3D
from HSTdynamics import HSTdynamics
from KeplerEq import KeplerEq
from OE2ECI import OE2ECI
from bdot_dynamics import bdot_dynamics 

plt.close('all')
# Initialize and Save Constant Parameters
# Orbital Elements and Parameters
a = 6917.5              # Orbit Semi-Major Axis (km)
e = 0.000287            # Eccentricity
i = 28.47               # Inclination (deg)
RAAN = 176.23           # Right Ascension of Ascending Node (deg)
w = 82.61               # Argument of Perigee (deg)
anom = 319.41           # Mean Anomaly (deg)
mu = 3.986e5            # Earth Standard Gravitational Parameter (km^3/s^2)
T = 2*np.pi*np.sqrt((a**3)/mu) # Orbital Period (s)
orbital_parameters_data = {"a":a,"e":e,"i":i,"RAAN":RAAN,"w":w,"anom":anom,"mu":mu,"T":T} 
pickle.dump(orbital_parameters_data,open("Parameters\orbital.txt","wb"))

# Mass Properties, Principle Axes of Inertia
J11 = 28525.53
J22 = 174815.86
J33 = 181630.81
J=np.array([J11,J22,J33])
J = np.diag(J) # [x-axis z-axis y-axis]
'''
# Dynamics with bdot control
'''

# Initial Conditions
om0 = np.array([0.25,0.25,2.5]).T # rad/s
t0 = np.array([2019,1,31,12,37,20,20])
u_max = 10 
r_eci, v_eci = OE2ECI(a, e, i, RAAN, w, anom, mu)
x0 = np.append(r_eci, v_eci)
x0 = np.append(x0,om0)
tspan = np.arange(0, 3*T, 10)

x = odeint(bdot_dynamics, x0, tspan, args=(t0,u_max,J,mu))


# orbital position 
detumble_position_data = {"x":x,"t":tspan} 
pickle.dump(detumble_position_data,open("Parameters\detumble_position.txt","wb"))

# Plot Orbit 
plt.figure()
ax=plt.axes(projection='3d')
ax.plot3D(x[:,0],x[:,1],x[:,2],'r')
ax.axis('equal')
plt.xlabel('[km]')
plt.ylabel('[km]')
ax.set_title('Detumbling Orbit Dynamics')
plt.show()

# Plot Omega
om_true = x[:,6:9]
plt.figure()
f,(ax1,ax2,ax3)=plt.subplots(3,1,sharey=True)
ax1.plot(tspan,om_true[:,0])
plt.ylabel('\omega_1')
plt.xlabel('t')
ax1.set_title('Angular Velocity wiht Bdot controller')
ax2.plot(tspan,om_true[:,1])
plt.ylabel('\omega_2')
plt.xlabel('t')
ax3.plot(tspan,om_true[:,2])
plt.ylabel('\omega_3')
plt.xlabel('t')
plt.show()
