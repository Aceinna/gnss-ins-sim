"""geomag package
by Christopher Weiss cmweiss@gmail.com

Adapted from the geomagc software and World Magnetic Model of the NOAA
Satellite and Information Service, National Geophysical Data Center
http://www.ngdc.noaa.gov/geomag/WMM/DoDWMM.shtml

Suggestions for improvements are appreciated.

USAGE:
>>> import geomag
>>> geomag.declination(80,0)
-6.1335150785195536
"""

from . import geomag

__singleton__ = geomag.GeoMag()

def declination(*args, **kargs):
    """Calculate magnetic declination in degrees
    dlat = latitude in degrees
    dlon = longitude in degrees
    h = altitude in feet, default=0
    calc_date = date for computing declination, default=today
    """
    mag = __singleton__.GeoMag(*args, **kargs)
    return mag.dec

def mag_heading(hdg, *args, **kargs):
    """Calculates the magnetic heading from a true heading.
    hdg = true heading in degrees
    All other parameters are the same as declination.
    """
    dec = declination(*args, **kargs)
    return (hdg - dec + 360.0) % 360
