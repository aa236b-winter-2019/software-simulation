def sunflux(Earth2Sat, Sun2Earth, q):
	# inputs:
	#	Earth2Sat: vector in ECI [km]
	#	Sun2Earth: vector in ECI [km]
	#	q: quaternion from ECI to body frame
	import numpy as np
	from subroutines import q2rot

	areas = 0.6*0.01*np.array([1,1,1,1,1,1]) # 60% of area covered in panels [m]
	r = np.dot(q2rot(q), -Earth2Sat) # position of earth in body frame
	R = np.dot(q2rot(q), -(Sun2Earth + Earth2Sat)) # position of sun in body frame

	eff = 0.28
	Re_sq = 4.058974e7 # earth radius squared [km^2]
	R_hat = R/np.linalg.norm(R) # unit vector to sun
	if np.dot(R_hat,r) > 0:
		d_sq = np.dot(r,r) - np.dot(r,R_hat)**2 # distance squared from center of earth to satellite-sun axis
		if d_sq < Re_sq:
			return 0

	E0 = 3.848e26 # [J]
	J = E0/(4*np.pi*np.dot(R, R)) # scalar flux [J/km^2]
	J *= 1e-6 # scalar flux [J/m^2]
	J_vec = -J*R_hat # vector flux

	faces = np.array([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]])
	fluxes = np.dot(faces, J_vec) # incident flux on each side
	fluxes = np.multiply(fluxes, areas)*eff  # scale by panel size and efficiency
	fluxes = [-np.minimum(0, flux) for flux in fluxes] # remove shaded panels
	return np.sum(fluxes)
