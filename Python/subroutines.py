import numpy as np

def qmult2(q1, q2):
	v1 = q1[:3]
	s1 = q1[3]
	v2 = q2[:3]
	s2 = q2[3]

	v = s1*v2 + s2*v1 + np.cross(v1, v2)
	s = s1*s2 - np.dot(v1, v2)

	return np.append(v, s)

def qdag(q):
	q[:3] = -q[:3]
	return q

def qkin(q, w):
	v = q[:3]
	s = q[3]

	vdot = s*w + np.cross(v, w)
	sdot = -np.dot(v, w)

	return (1/2)*np.append(vdot, sdot)

def qmult(q):
	# Quaternion Hat Operator
	# Convert Quaternion into a 4x4 Square Matrix
	v = q[:3];
	s = q[3];

	# cross product operator
	hat_v = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], 
		[-v[1], v[0], 0]])
	# build 4x4 matrix
	qhat = np.block([[s*np.eye(3) + hat_v, v.reshape((v.shape[0], 1))], 
		[v.reshape((1, v.shape[0])), s]])
	return qhat

def KeplerEq(M, e, tol):
# Converts mean anomaly to eccentric anomaly.
#
# Inputs:
#   * mean anomaly
#   * eccentricity
# Output:
#   * eccentric anomaly

	# initial guess
	if ((M > -np.pi) and (M < 0)) or (M > np.pi):
	    E = M - e
	else:
	    E = M + e

	# iteration
	d = -(E - e*np.sin(E) - M)/(1 - e*np.cos(E));
	while np.abs(d) >= tol:
	    E = E + d
	    d = -(E - e*np.sin(E) - M)/(1 - e*np.cos(E))
	return E

def OE2ECI(a, e, i, RAAN, w, anom, mu):
# OE2ECI Converts orbital elements to r, v in inertial frame
#
#   Notes: 
#       In cases of equatorial and/or circular orbits, it is assumed
#       that valid orbital elements are provided as inputs (ie. there is 
#       no back-end validation)
#
# Inputs:
#      a - semi-major axis of orbit [km]
#      e - eccentricity of orbit
#      i - inclination of orbit [deg]
#      RAAN - right ascension of the ascending node [deg]
#      w - argument of periapsis [deg]
#      anom - true anomaly [deg]
#      mu - central body gravitational parameters [km^3/s^2]
#
# Outputs:
#   r_eci - 3x1 vector of radius in ECI frame [km]
#   v_eci - 3x1 vector of velocity in ECI frame [km/s]

	n = np.sqrt(mu/a**3) # rad/s

	E = KeplerEq(np.deg2rad(anom), e, 1e-6) # rad

	# compute radius and velocity of orbit in perifocal coordinates
	rPeri = np.array([a*(np.cos(E) - e), a*np.sqrt(1 - e**2)*np.sin(E), 0])
	vPeriComp = np.array([-np.sin(E), np.sqrt(1 - e**2)*np.cos(E), 0])
	vPeri = (a*n)/(1 - e*np.cos(E))*vPeriComp;

	# develop rotation matrix depending on orbit shape/inclination
	if i == 0 and e != 0:          # equatorial + elliptical
		rotPeri2ECI = rotz(np.deg2rad(w))
	elif e == 0 and i != 0:    # circular + inclined
		rotPeri2ECI = np.dot(rotz(np.deg2rad(RAAN)),rotx(np.deg2rad(i)))
	elif i == 0 and e == 0:     # equatorial + circular
		rotPeri2ECI = 1
	else:                          # elliptical + inclined
		rotPeri2ECI = np.dot(np.dot(rotz(np.deg2rad(RAAN)),rotx(np.deg2rad(i))),rotz(np.deg2rad(w)))

	# rotate vectors into ECI frame
	r_eci = np.array(rotPeri2ECI.dot(rPeri))
	v_eci = np.array(rotPeri2ECI.dot(vPeri))
	return r_eci, v_eci

def rotz(w):
	return np.array([[np.cos(-w),np.sin(-w),0],
					  [-np.sin(-w),np.cos(-w),0],
					  [0,0,1]
					])

def rotx(i):
	return np.array([[1,0,0],
					 [0,np.cos(-i),np.sin(-i)],
					 [0,-np.sin(-i),np.cos(-i)]
					])
def q2rot(q):
	# Converts quaternion to corresponding rotation matrix
	# Inputs:
	#   q - Quaternion (4x1)
	# Outputs:
	#   Q - Rotation Matrix (3x3)
	import numpy as np

	v = q[:3]
	s = q[3]

	# cross product operator
	hat_v = np.array([[0, -v[2], v[1]], 
		             [v[2], 0, -v[0]], 
		             [-v[1], v[0], 0]])

	# calculating rotation matrix
	Q = np.eye(3) + np.dot(2*hat_v,(s*np.eye(3) + hat_v))
	return Q