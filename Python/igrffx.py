def igrffx(eci_vec,year,month,day,hour,minute,second,microsecond):
	import numpy
	import pyIGRF
	import pymap3d
	from pymap3d import ecef2eci
	import navpy
	from navpy import ned2ecef
	import datetime
	from datetime import datetime
	import astropy
	#eci_vec is a xyz vector in ECI in KM
	#output B_ECI is a 3 item array in units of nT
	
	#get time 
	#datetime(year, month, day, hour, minute, second, microsecond)
	time = datetime(year, month, day, hour, minute, second, microsecond)

	#get our lat long and alt from ECI 
	geod = pymap3d.eci2geodetic(1000*eci_vec, time, useastropy=True)

	latitude = geod[0][0] #degrees
	longitude = geod[1][0] #degrees
	altitude = geod[2] #meters

	#call igrf to get b vector in NED
	#b in NED in nT
	b = pyIGRF.igrf_value(latitude, longitude, altitude/1000, 2019)

	#combine NED components back into an array
	NED = b[3:6]

	#convert from NED to ECEF
	ECEF = navpy.ned2ecef(NED, latitude, longitude, altitude)

	#convert from ECEF to ECI
	B_ECI = pymap3d.ecef2eci(ECEF, time, useastropy=True)

	return B_ECI








