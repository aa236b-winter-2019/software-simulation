def igrffx(eci_vec,time):
	#import igrf12
	import numpy
	import pyIGRF
	import pymap3d
	from pymap3d import ecef2eci
	import navpy
	from navpy import ned2ecef
	import datetime
	from datetime import datetime
	import astropy
	#eci_vec is a xyz vector in ECI  km
	#output B_ECI is a 3 item array in units of T
	

	#get our lat long and alt from ECI 
	geod = pymap3d.eci2geodetic(1000*eci_vec, time, useastropy=True)

	latitude = geod[0][0] #degrees
	longitude = geod[1][0] #degrees
	altitude = geod[2] #meters

	#call igrf to get b vector in NED
	#mag = igrf12.igrf('2019-01-12', glat=latitude, glon=longitude, alt_km=altitude/1000)
	b = pyIGRF.igrf_value(latitude, longitude, altitude/1000, 2019)


	#combine NED components back into an array
	#NED = numpy.array([b_north,b_east,b_down])
	NED = b[3:6]

	#convert from NED to ECEF
	ECEF = navpy.ned2ecef(NED, latitude, longitude, altitude)

	#convert from ECEF to ECI
	B_ECI = (pymap3d.ecef2eci(ECEF, time, useastropy=True))*10**(-9)

	return B_ECI









