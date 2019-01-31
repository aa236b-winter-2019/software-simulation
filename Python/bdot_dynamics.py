def bdot_dynamics(x0,t,t0,u_max,J):
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
    from igrffx import igrffx

    eci_vec= np.copy(x0[0:5])
    om=np.copy(x0[6:8])
    time= np.copy(t0)
    # converting time to required format
    time[2]=t0[2]+t/(3600*12)
    t_rem=t%(3600*12)
    time[3]=t0[3]+t_rem/3600
    t_rem=t%3600
    time[4]=t0[4]+t/60
    time[5]=t%1

    B=igrffx(eci_vec,time[0],time[1],time[2],time[3],time[4],time[5],time[6])

    # calculating torque
    B_moment_value = u_max*np.tanh(np.dot(om,B))
    B_cross = np.array([[0, -B[2], B[1]],
                       [B[2], 0, -B[0]],
                       [-B[1], B[0], 0]])
    B_moment_direction = np.dot(np.linalg.inv(-B_cross),np.cross(om,np.dot(J,om)))  
    B_moment_direction = -B_moment_direction/np.linalg.norm(B_moment_direction)
    B_moment = B_moment_value * B_moment_direction
    torque = np.cross(B_moment,B)

    # matrix linear equation
    om_dot = -np.linalg.inv(J).dot(np.cross(om, J.dot(om) + torque))
    
    eci_vec_dot = HSTdynamics(eci_vec, t, mu, J, torque)
    
    x_dot = np.append(om_dot, eci_vec_dot)
    return x_dot










