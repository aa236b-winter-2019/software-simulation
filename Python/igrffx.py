
import igrf12
import numpy
import pymap3d
from pymap3d import ecef2eci
import navpy
from navpy import ned2ecef
import datetime
from datetime import datetime
import astropy


def igrffx(eci_vec,year,month,day,hour,minute,second,microsecond):
	#eci_vec is a xyz vector in ECI 


	#get time 
	#datetime(year, month, day, hour, minute, second, microsecond)
	time = datetime(year, month, day, hour, minute, second, microsecond)

	#get our lat long and alt from ECI 
	geod = eci2geodetic(eci_vec, time, useastropy=True)

	latitude = geod[0][0] #degrees
	longitude = geod[1][0] #degrees
	altitude = geod[2] #meters

	#call igrf to get b vector in NED
	mag = igrf12.igrf('2019-01-12', glat=latitude, glon=longitude, alt_km=altitude/1000)

	#pull NED components out of xarray.dataset 
	b_north = mag.north.values[0]
	b_east = mag.east.values[0]
	b_down = mag.down.values[0]

	#combine NED components back into an array
	NED = numpy.array([b_north,b_east,b_down])

	#convert from NED to ECEF
	ECEF = navpy.ned2ecef(NED, latitude, longitude, altitude)

	#convert from ECEF to ECI
	B_ECI = pymap3d.ecef2eci(	ECEF, time, useastropy=True)

	return B_ECI









