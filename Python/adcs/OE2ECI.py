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
	import numpy as np

	n = np.sqrt(mu/a**3) # rad/s

	E = KeplerEq(np.deg2rad(anom), e, 1e-6) # rad

	# compute radius and velocity of orbit in perifocal coordinates
	rPeri = np.array([a*(np.cos(E) - e), a*np.sqrt(1 - e**2)*np.sin(E), 0])
	vPeriComp = np.array([-np.sin(E), np.sqrt(1 - e**2)*np.cos(E), 0])
	vPeri = (a*n)/(1 - e*np.cos(E))*vPeriComp;

	# develop rotation matrix depending on orbit shape/inclination
	if i == 0 and e != 0:          # equatorial + elliptical
		rotPeri2ECI = rotz(w)
	elif e == 0 and i != 0:    # circular + inclined
		rotPeri2ECI = rotz(RAAN)*rotx(i)
	elif i == 0 and e == 0:     # equatorial + circular
		rotPeri2ECI = 1
	else:                          # elliptical + inclined
		rotPeri2ECI = rotz(RAAN)*rotx(i)*rotz(w)

	# rotate vectors into ECI frame
	r_eci = np.array(rotPeri2ECI.dot(rPeri))
	v_eci = np.array(rotPeri2ECI.dot(vPeri))
	return np.append(r_eci, v_eci)

def rotz(w):
	import numpy as np
	return np.matrix([[np.cos(w),-np.sin(w),0],
					  [np.sin(w),np.cos(w),0],
					  [0,0,1]
					])

def rotx(i):
	import numpy as np
	return np.matrix([[1,0,0],
					 [0,np.cos(i),-np.sin(i)],
					 [0,np.sin(i),np.cos(i)]
					])