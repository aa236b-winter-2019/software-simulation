def bdot_dynamics(x0,t,t0,u_max,J,mu):
# bdot_dynamics: Contains  ODE dynamics for all spacecraft states.
#
# Inputs:
#      om - angular velocitty []
#      t - time [s]
#      t0 -  time in [year,month,day,hour,minute,second,microsecond]
#      B - magnetic field vector []
#      u_max - maximum value of control effort (magnetic moment) []
#      J - spacecraft inertia matrix
#
# Outputs:
#      om_dot - linear equation containing ODEs for angular velocity
    
    import numpy as np
    from HSTdynamics import HSTdynamics
    from q2rot import q2rot

    #from igrffx import igrffx

    
    time= np.copy(t0)
    # converting time to required format
    time[2]=t0[2]+t/(3600*12)
    t_rem=t%(3600*12)
    time[3]=t0[3]+t_rem/3600
    t_rem=t%3600
    time[4]=t0[4]+t/60
    time[5]=t%1

    om = x0[:3]
    q = x0[3:7]
    #B=igrffx(1000*eci_vec[0:3],time[0],time[1],time[2],time[3],time[4],time[5],time[6])

 
    B=np.array([1,2,3]).T
    # calculating torque
    B_moment_value = u_max*np.tanh(np.dot(om.T,B))
    B_moment_direction = np.cross(om,B) 
    B_moment_direction = B_moment_direction/np.linalg.norm(B_moment_direction)
    B_moment = B_moment_value * B_moment_direction
    torque = np.cross(B_moment,B)
    torque = np.dot(q2rot(q),torque)
    torque = np.array([0,0,0]).T
    # matrix linear equation

    om_q_dot = HSTdynamics(x0[:7], time, mu, J, torque)
    r_v_dot = HSTdynamics(x0[7:], time, mu, J, torque)
    x_dot = np.append(om_q_dot,r_v_dot)
    
    return x_dot












