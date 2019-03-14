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
import matplotlib as mpl


plt.close('all')

# Orbital Elements and Parameters
a = 6917.50                                                                     # Orbit Semi-Major Axis (km)
e = 0.000287                                                                    # Eccentricity
i = 28.47                                                                       # Inclination (deg)
RAAN = 176.23                                                                   # Right Ascension of Ascending Node (deg)
w = 82.61                                                                       # Argument of Perigee (deg)
anom = 100                                                                      # Mean Anomaly (deg)
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
om = 0.25*np.array([1,-1,-.5])                                                  # Initial Angular Velocity (rad/s)
q = np.array([0,0,1,0])                                                         # Initial Quaternion, Identity
init_state = np.concatenate((r,v,om,q))                                         # Initial Attitude State Vector
tspan = [0]                                                                     # Start Time (sec)  
n = 300                                                                         # Epochs per Orbital Period
epochs = 5                                                                      # Total Number of Epochs
m_value= np.zeros((1,3))                                                        # Initial magnetic moment

# Calculate Earth's Magnetic Field and Sun's Location in ECI
mjd = 54372.78                                                                  # Mean Julian Date of Initial Epoch
t0 = julian.from_jd(mjd, fmt='mjd')                                             # Convert mjd into seconds
B_eci = igrffx(r,t0)                                                            # Earth's Magnetic Field in ECI
Sun2Earth = sunlocate(mjd)                                                      # Unit Vector from Sun to Earth in ECI

# Initialize the Magnetorquer Properties
area_coil = np.array([0.294807214285714,0.294807214285714,0.294807214285714])   # Enclosed Coil Area in Each Axis (m^2)
voltage_max = 8                                                                 # Max Battery Voltage (V)
resistance  = np.array([245,245,245])                                           # Coil Resistance (Ohm)
I_max = np.divide(voltage_max,resistance)                                       # Max Current (A)
m_max = I_max*area_coil                                                         # Max Magnetic Moment (A m^2)
power_max = voltage_max*I_max                                                   # Max Power Consumed (W)
energy_consumed = 0                                                             # Initializing total energy consumption (J)

                                                          



max_time = 50																	# Number of seconds for simulation 

state = np.zeros((max_time,13))													# State vector holds everything
torque_hist = np.zeros((max_time-1,3))											# Torque history
B_hist = np.zeros((max_time-1,3))												# B history
sun_fluxes = np.zeros((max_time-1,1))											# Sun fluxes
state[0,:] = init_state															# Set first state entry as the initial state


for i in range (0,max_time-1):


    #Get time for B value
    dt = julian.from_jd(mjd, fmt='mjd')                                         
    newtime = dt + datetime.timedelta(seconds=int(i))

    #Calculate B_eci
    B_eci = B_eci = igrffx(state[i,0:3],newtime) 
    
    #calculate b dot math before ODEint
    B_body = np.dot(q2rot(state[i,9:13]),B_eci)
    B_dot = -np.cross(state[i,6:9],B_body)                                           
    m_value = -np.multiply(m_max,np.sign(B_dot))
    torque = np.cross(m_value,B_body)
    
    #check if we need to turn off the controller
    if np.linalg.norm(state[i,6:9]) < np.deg2rad(3):                                           
        m_max = np.array([0,0,0])
        
    #index the torques and B values
    torque_hist[i,:] = torque
    B_hist[i,:] = B_eci
    
    #Propogate State Vector with Applied Torque for 1 second
    tspan = np.arange(i, i+2, 1)
    args = (mu,J,J_inv,B_eci,m_max,power_max,Sun2Earth,torque)
    state2, test_dict = odeint(HSTdynamics2, state[i,:], tspan, args, full_output=True, rtol = 1e-9,atol = 1e-9)
    
    #Grab the most recent state
    state[i+1,:] = state2[-1,:]
    

    #calculate time for sunlocate
    mjd_prop = 54372.78 + (i/(60*60*24))                                                             
    Sun2Earth=sunlocate(mjd_prop)    

    #sun flux
    sun_fluxes[i,:] = sunflux(state[i+1,0:3], Sun2Earth, state[i+1,9:13])

        
        
#graphing time vectors
graph_T = np.linspace(1,max_time, num = max_time)/60
graph_hist = graph_T[:-1]


# second try orbit plot
fig = plt.figure()
ax1 = fig.add_subplot(111, projection='3d')

# Make data
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)
x = 6378.1 * np.outer(np.cos(u), np.sin(v))
y = 6378.1 * np.outer(np.sin(u), np.sin(v))
z = 6378.1 * np.outer(np.ones(np.size(u)), np.cos(v))

# Plot the surface
ax1.plot_surface(x, y, z, color='b')


mpl.rcParams['legend.fontsize'] = 10


ax2 = fig.gca(projection='3d')
theta = np.linspace(-4 * np.pi, 4 * np.pi, 100)
z = np.linspace(-2, 2, 100)
r = z**2 + 1
x = r * np.sin(theta)
y = r * np.cos(theta)
ax2.plot(state[1:,0],state[1:,1],state[1:,2],'r', label='Panda Sat')
ax2.legend()

plt.show()



#Plot Quaternion
f,(ax1,ax2,ax3,ax4)=plt.subplots(4,1,sharey=True)
plt.rcParams['axes.grid'] = True
ax1.plot(graph_T,state[:,9])
ax1.set_title('Quaternion Dynamics')
ax1.set(ylabel='Q1')
ax2.plot(graph_T,state[:,10])
ax2.set(ylabel='Q2')
ax3.plot(graph_T,state[:,11])
ax3.set(ylabel='Q3')
ax4.plot(graph_T,state[:,12])
ax4.set(xlabel='Time (Minutes)', ylabel='Q4')
plt.show()



#Plot Omega
f,(ax1,ax2,ax3)=plt.subplots(3,1,sharey=True)
plt.rcParams['axes.grid'] = True
ax1.plot(graph_T,abs(np.rad2deg(state[:,6])))
ax1.set_title('Angular Velocity')
ax1.set(ylabel='wx (deg/s)')
ax2.plot(graph_T,abs(np.rad2deg(state[:,7])))
ax2.set(ylabel='wy (deg/s)')
ax3.plot(graph_T,abs(np.rad2deg(state[:,8])))
ax3.set(xlabel='Time (Minutes)', ylabel='wz (deg/s)')
plt.show()

#Plot Torque
f,(ax1,ax2,ax3)=plt.subplots(3,1,sharey=True)
plt.rcParams['axes.grid'] = True
ax1.plot(graph_hist,torque_hist[:,0])
ax1.set_title('Torque')
ax1.set(ylabel='Tx (N*m)')
ax2.plot(graph_hist,torque_hist[:,1])
ax2.set(ylabel='Ty (N*m)')
ax3.plot(graph_hist,torque_hist[:,2])
ax3.set(xlabel='Time (Minutes)', ylabel='Tz (N*m)')
plt.show()

#plot B
lines = plt.plot(graph_hist, B_hist[:,0], graph_hist, B_hist[:,1], graph_hist, B_hist[:,2])
plt.legend(('Bx', 'By', 'Bz'),
           loc='upper right')
plt.title('Magnetic Field')
plt.xlabel("Time (Minutes)")
plt.ylabel("Magnetic Field (Teslas)")
plt.show()

#plot sunflux
lines = plt.plot(graph_hist, sun_fluxes[:,0])

plt.title('Sun Flux')
plt.xlabel("Time (Minutes)")
plt.ylabel("Sun Flux (Watts)")
plt.show()




# Plot Power and Compute Energy Consumption
#energy_consumed = np.trapz(Pcon_hist, Tspan)
#print('Energy Consumed = ', energy_consumed)
#plt.figure()
#plt.plot(Tspan/60,Pcon_hist,'r')
#plt.xlabel('Time (min)')
#plt.ylabel('Power Consumed (W)')
#plt.title('Power Consumption ')
#plt.show()
#
## Plot Power Generation
#plt.figure()
#plt.plot(Tspan/60,Pgen_hist,'r')
#plt.xlabel('Time (min)')
#plt.ylabel('Power Generated (W)')
#plt.title('Power Generation')
#plt.show()
