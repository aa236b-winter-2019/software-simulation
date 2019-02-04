def KeplerEq(M, e, tol):
# Converts mean anomaly to eccentric anomaly.
#
# Inputs:
#   * mean anomaly
#   * eccentricity
# Output:
#   * eccentric anomaly
	import numpy as np

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