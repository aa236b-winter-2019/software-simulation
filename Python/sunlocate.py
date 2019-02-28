def sunlocate(time):
	#inputs
	# time: time in MJD

	#outputs
	#Sun2Earth - unit vector from sun2earth in ECI

	import astropy
	import astropy.coordinates
	import datetime
	import julian
	from astropy.time import Time
	import pymap3d
	from pymap3d import geodetic2ecef
	import decimal
	from astropy import units as u
	from astropy.coordinates import Angle
	import numpy as np


	#get the time in MJD
	#mjd = 58562
	t2 = Time(time, format='mjd')


	#print(t2)


	#get the location of the sun in lat long
	sun_GEOD = astropy.coordinates.get_sun(t2)

	#print(sun_GEOD)

	lon_extra = Angle(sun_GEOD.ra)
	lat_extra = Angle(sun_GEOD.dec)

	#get it as a float
	lon = 1*lon_extra.degree
	lat = 1*lat_extra.degree


	#print(lon)
	#print(lat)
	#print(type(lon))
	#a = Angle(lat)
	#print(a.degree)

	here = type(lon)
	#print(here)

	#convert the geodetic coordinates to ECEF
	sun_ECEF = geodetic2ecef(	lat, lon, 0, ell=None, deg=True)

	#Convert ECEF to ECI
	sun_ECI = pymap3d.ecef2eci(sun_ECEF, t2, useastropy=True)

	#get the norm to normalize it
	norm_sun_ECI = np.linalg.norm(sun_ECI)

	#print(norm_sun_ECI)




	Sun2Earth = 1.496e8* sun_ECI/norm_sun_ECI



	return Sun2Earth


