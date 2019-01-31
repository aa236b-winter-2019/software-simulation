import numpy as np
import matplotlib.pyplot as plt
import pickle
from scipy.linalg import block_diag # for star sensor
from scipy.integrate import odeint
from mpl_toolkits.mplot3d import Axes3D
from HSTdynamics import HSTdynamics
from KeplerEq import KeplerEq
from OE2ECI import OE2ECI
plt.close('all')

## Initialize and Save Constant Parameters
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
mass_properties_data = {"J11":J11,"J22":J22,"J33":J33,"J":J} 
pickle.dump(mass_properties_data,open("Parameters\mass.txt","wb"))


# Sensor Specifications
'''
# Star Tracker (Ball Aerospace CT-601)
error_ST = 300 # Arcseconds
error_ST = error_ST*(1.0/3600)*(np.pi/180) # Radians
V_st = np.diag(np.array((error_ST**2)*np.ones((1,3))))
# Gyro (HST Gas-Bearing Gyros)
gyro_rnd = 0.1*(np.pi/180)  # Rate Noise Density, converted from [deg/s/sqrt(Hz)]
gyro_arw = 0.1*(np.pi/180) # Angle Random Walk, converted from [deg/sqrt(hr)]
W_rnd = (gyro_rnd**2)*np.eye(3,dtype=float) # Gyro Covariance
W_arw = (gyro_arw**2)*np.eye(3,dtype=float) # Bias Covariance
W_gyro = block_diag(W_rnd, W_arw)
'''
# Magnetometer 
# error_mag = 4.0*(np.pi/180) # Radians
# V_mag = (error_mag**2)*np.eye(3,dtype=float)
# sensor_data = {"error_mag":error_mag,"V_mag":V_mag} 
# #sensor_data = {"error_ST":error_ST,"V_st":V_st,"gyro_rnd":gyro_rnd,"gyro_arw":gyro_arw,"W_gyro":W_gyro,"error_mag":error_mag,"V_mag":V_mag}
# pickle.dump(sensor_data,open("Parameters\sensor.txt","wb"))

# Compute Environmental Disturbance Parameters (HST-Specific Values)
# CG = np.array([[-0.01],[ 6.03],[0.00]])
# n1 = np.array([1,0,0]).T
# n2 = np.array([-1,0,0]).T
# n3 = np.array([0,1,0]).T
# n4 = np.array([0,-1,0]).T
# n5 = np.array([0,0,1]).T 
# n6 = np.array([0,0,-1]).T
# c1 = np.array([0.64,6.59,0.00]).T 
# c2 = np.array([-1.00, 6.59, 0.00]).T 
# c3 = np.array([-0.03, 10.28, 0.00]).T
# c4 = np.array([-0.03, 0.72, 0.00]).T
# c5 = np.array([-0.01, 6.28, 1.85]).T 
# c6 = np.array([-0.01, 6.28, -1.85]).T
# S = np.array([122.73, 122.73, 20.97, 20.97, 54.04, 54.04]) #Surface Area of Each Side, m^2
# n = np.column_stack((n1, n2, n3, n4, n5, n6)) # Surface Normal Vectors
# c = np.column_stack((c1, c2, c3, c4, c5, c6)) - CG*np.ones((1,6)) # Surface Centroid Coordinates w.r.t. CG, m
# params = np.vstack((S,n,c))
# # HST Reaction Wheel Actuator Jacobian
# Bw = 1./np.sqrt(3)*np.array([[-1.,1.,1.,-1.],[1.,1.,1.,1.],[-1.,-1.,1.,1.]])
# disturbance_parameters_data = {"Bw":Bw,"params":params} 
# pickle.dump(disturbance_parameters_data,open("Parameters\geometry.txt","wb"))

'''
## Superspin and Dynamic Balance (All Calcs Done in Principal Frame)
load('Parameters\orbital')
load('Parameters\mass')
'''
# Initialize Nominal Spin Conditions
# om_max = 10 # RPM
# dt = J33*om_max
# rho = 0
# om_vec1 = np.array([om_max,0,0]).T
# om_vec1[0] = np.sqrt((dt)**2 - (J22*om_vec1[1])**2 - (J33*om_vec1[2])**2)/J11
# om_vec2 = np.array([0,om_max,0]).T
# om_vec2[1] = np.sqrt((dt)**2 - (J11*om_vec2[0])**2 - (J33*om_vec2[2])**2)/J22
# om_vec3 = np.array([0,0,om_max]).T

# Compute Perturbed Attitude Dynamics
tol = 1e-6
#--------------------------- 
#---------------------------
#opts  = odeset('reltol', tol, 'abstol', tol)
tspan = np.linspace(0,5,500)

# count = 1
# c = 4

# for ii in range(0,c):
#     for jj in range(0,c):      
#         w1 = np.array([0,ii,jj]).T
#         w1[0] = (np.sqrt(dt**2-(J22*w1[1])**2-(J33*w1[2])**2))/J11
#         w2 = np.array([ii,0,jj])
#         w2[1] = np.sqrt((dt)**2-(J11*w2[0])**2-(J33*w2[2])**2)/J22
#         w3 = np.array([0,ii,jj])
#         w3[2] = np.sqrt((dt)**2-(J22*w3[2])**2-(J11*w3[0])**2)/J33
        
#         wt1 = odeint(HSTdynamics, w1, tspan, args=(mu,J,rho))
#         wt2 = odeint(HSTdynamics, w2, tspan, args=(mu,J,rho))
#         wt3 = odeint(HSTdynamics, w3, tspan, args=(mu,J,rho))
        
#         ht1=[[[0 for k in range(3)] for j in range(500)] for i in range(3)]

#         print('shape of J:', J.shape)
#         print('shape of wt1:', wt1.shape)
#         print('shape of prod:', np.dot(J,wt1.T).shape)

#         ht1[:,:,count] = np.dot(J,wt1.T)
#         ht2[:,:,count] = np.dot(J,wt2.T)
#         ht3[:,:,count] = np.dot(J,wt3.T)

#         wt1[:,:,count] = wt1
#         wt2[:,:,count] = wt2
#         wt3[:,:,count] = wt3

#         count = count + 1

# # Plot Perturbed Dynamics
# plt.figure()
# ax=plt.axes(projection='3d')
# plt.hold(True)
# for ii in range(1,count-1):
#     ax.plot3D( ht1[1,:,ii], ht1[2,:,ii], ht1[3,:,ii],color='red')
#     ax.plot3D( ht2[1,:,ii], ht2[2,:,ii], ht2[3,:,ii],color='blue')
#     ax.plot3D( ht3[1,:,ii], ht3[2,:,ii], ht3[3,:,ii],color='black')
#     ax.plot3D(-ht1[1,:,ii],-ht1[2,:,ii],-ht1[3,:,ii],color='red')
#     ax.plot3D(-ht2[1,:,ii],-ht2[2,:,ii],-ht2[3,:,ii],color='blue')
#     ax.plot3D(-ht3[1,:,ii],-ht3[2,:,ii],-ht3[3,:,ii],color='black')

# # Plot Momentum Sphere
# u = np.linspace(0, 2 * np.pi, 100)
# v = np.linspace(0, np.pi, 100)
# X = 10 * np.outer(np.cos(u), np.sin(v))
# Y = 10 * np.outer(np.sin(u), np.sin(v))
# Z = 10 * np.outer(np.ones(np.size(u)), np.cos(v))
# ax.plot_surface(0.6*x, .6*y, .6*z, color='grey')
# ax.plot_surface(0.99*dt*X,0.99*dt*Y,0.99*dt*Z,color='black')

# # Plot Equilibrium Points
# hequil_1 = np.dot(J,om_vec1)
# hequil_2 = np.dot(J,om_vec2)
# hequil_3 = np.dot(J,om_vec3)
# ax.plot3D( hequil_1[1,:], hequil_1[2,:], hequil_1[3,:],'k*')
# ax.plot3D( hequil_2[1,:], hequil_2[2,:], hequil_2[3,:],'k*')
# ax.plot3D( hequil_3[1,:], hequil_3[2,:], hequil_3[3,:],'k*')
# ax.plot3D(-hequil_1[1,:],-hequil_1[2,:],-hequil_1[3,:],'k*')
# ax.plot3D(-hequil_2[1,:],-hequil_2[2,:],-hequil_2[3,:],'k*')
# ax.plot3D(-hequil_3[1,:],-hequil_3[2,:],-hequil_3[3,:],'k*')
# ax.axis('equal')
# ax.set_title('Momentum Sphere w/o Gyrostat')
# ax.legend(['Body X-Axis (Major)','Body Z-Axis (Inter)','Body Y-Axis (Minor)'])


# -------------------------------------------------------------------------
# Safe Mode Rotation (about Intermediate Axis [Body Frame Y-Axis])
# J_eff = 1.2*J33
# om0_equil = np.array(om_vec3)
# rho = (J_eff - J33)*om0_equil
# h_rho = np.linalg.norm(J*om0_equil + rho)

# Integrate Gyrostat Dynamics
#opts-----------------------------------

# om_equil = odeint(HSTdynamics, om0_equil, tspan, args=(mu,J,rho))
# dims = np.shape(om_equil)
# h_equil = np.dot(J, om_equil.T) + np.reshape(rho, (3,1))*np.ones((1,dims[0]))

# om04 = np.array([2,2,10]).T
# om04[2] = np.sqrt((om_max*J33)**2 - (J11*om04[0])**2 - (J22*om04[1])**2)/J33
# #opts-----------------------------
# om4 = odeint(HSTdynamics, om04, tspan, args=(mu,J,rho))
# h4 = np.dot(J, om4.T) + np.reshape(rho, (3,1))*np.ones((1,np.shape(om4)[0]))

# plt.figure()
# ax=plt.axes(projection='3d')
# plt.hold(True)

# # Plot Perturbed Dynamics
# ax.plot3D(h4[0,:],h4[1,:],h4[2,:],color='b')

# # Plot Momentum Sphere
# # ax.plot_surface(0.6*np.multiply(h_rho,x), .6*np.multiply(h_rho,y), .6*np.multiply(h_rho,z), 'k*')

# # Plot Equilibrium Pointh_equil(1,:),h_equil(2,:),_equil(3,:)
# ax.plot3D( h_equil[0,:],h_equil[1,:],h_equil[2,:],'k*')
# ax.set_title('Momentum Sphere w/ Gyrostat')
# ax.legend('Stable Spin about Minor Axis')
# ax.axis('equal')

'''
## Model HST Orbital Dynamics
clc,clearvars,close all
load('Parameters\orbital')
load('Parameters\mass')
'''

# Convert to Earth-Centered Inertial Frame Coordinates
r_eci, v_eci = OE2ECI(a, e, i, RAAN, w, anom, mu)
x0 = np.append(r_eci, v_eci)
rho = 0

# ODE45 Solver for Orbital Dynamics
tol = 1e-6
#------------------
#opts  = odeset('reltol', tol, 'abstol', tol)
tspan = np.arange(0, 3*T, 10)
#opts------------------
x = odeint(HSTdynamics, x0, tspan, args=(mu,J,rho))


# orbital position
orbital_position_data = {"x":x,"t":tspan} 
pickle.dump(orbital_position_data,open("Parameters\orbital_position.txt","wb"))



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
## Model HST Attitude Dynamics
clc,clearvars,close all
load('Parameters\orbital')
load('Parameters\mass')
'''
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
#-----------------------------
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

'''
# Dynamics with bdot control
'''

# Initial Conditions
om0 = np.array([0.25,0.25,2.5]).T # rad/s
t0 = np.array([2019,01,31,12,37,20,20])
u_max = 10 
r_eci, v_eci = OE2ECI(a, e, i, RAAN, w, anom, mu)
x0 = np.append(r_eci, v_eci,om0)
tspan = np.arange(0, 3*T, 10)

x = odeint(bdot_dynamics, x0, tspan, args=(t0,u_max,J))


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
om_true = x[:,6:8]
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
