###############################################################################
# pyGEtools - Draft/examples/bits and bobs
# Control Google Earth through its API COM interface
# https://docs.google.com/document/d/1A4pHdyX2Rm693zYtXAMKHEhN5Boe4euX-siHKLC4noA/preview
###############################################################################

import win32com.client, time
from math import *

##Some default global variables
# Default Latitude in degrees. Between -90 and 90.
latitude=48.583106
# Default Longitude in degrees. Between -180 and 180.
longitude=7.751436
# Default altitude in meters
altitude=10000 # in meters
# looking to the horizon=90, looking to the center of Earth=0
tilt=0
# looking North=0, East=90, South=180, West=270
azimuth=370
# speed transition. must be >= 0, above 5.0 the transition is instantaneous
speed=5
# If not=0 camera will move backward from "range meters along the camera axis
range=0
#Altitude mode that defines altitude reference origin (1=above ground, 2=absolute)
altMode=2

def ge_connect():
    """ Open communication with Google Earth and start GE if necessary"""
    global GE
    GE =  win32com.client.Dispatch("GoogleEarth.ApplicationGE")
    while not GE.IsInitialized():
        time.sleep(0.5)
        print(">>> Waiting for Google Earth to initialize.")
    print(">>>> Connection established.")
   
def ge_setCamera():
    """ Set the camera to a given place, altitude and viewing angles """
    GE.SetCameraParams( latitude, longitude, altitude, altMode,range,tilt, azimuth, speed)

def ge_getCameraInfo():
    """ Get some informations about the camera"""
    cam=GE.GetCamera(False)
    camInfos={}
    camInfos["focusPointLatitude"]=cam.FocusPointLatitude
    camInfos["focusPointLongitude"]=cam.FocusPointLongitude
    camInfos["focusPointAltitude"]=cam.FocusPointAltitude
    camInfos["focusPointAltitudeMode"]=cam.FocusPointAltitudeMode
    camInfos["range"]=cam.Range
    camInfos["tilt"]=cam.Tilt
    camInfos["azimuth"]=cam.Azimuth
    # compute camera altitude (doesn't work if tilt is very near to 90 degrees)
    camInfos["altitude"]=(cam.Range)*cos(cam.Tilt*(2*pi)/360)
    #experiment
    cam.Azimuth=10
    print(">>> Setting camera now")
    GE.setCamera(cam,1)
    print(camInfos)
    return camInfos

def ge_cam_go_diffazimuth(angle,speed=1):
    """
    Change camera azimuth by angle value (degrees)
    angle can be postitive or negative
    """
    cam=GE.GetCamera(False)
    cam.Azimuth+=angle
    GE.setCamera(cam,speed)
   
def ge_cam_go_azimuth(angle,speed=1):
    """
    Rotate camera to azimuth angle (degrees)
    """
    cam=GE.GetCamera(False)
    cam.Azimuth=angle
    GE.setCamera(cam,speed)

def ge_cam_go_difftilt(angle,speed=1):
    """
    Change camera tilt by angle value (degrees)
    angle can be postitive or negative
    """
    cam=GE.GetCamera(False)
    cam.Tilt+=angle
    GE.setCamera(cam,speed)
   
def ge_cam_go_tilt(angle,speed=1):
    """
    Rotate camera to tilt value (degrees)
    """
    cam=GE.GetCamera(False)
    cam.Tilt=angle
    GE.setCamera(cam,speed)
   
def ge_cam_go_diffrange(length,speed=1):
    """
    Change camera rangge by angle value (degrees)
    angle can be postitive or negative
    """
    cam=GE.GetCamera(False)
    cam.Range+=length
    GE.setCamera(cam,speed)
   
def ge_cam_go_range(length,speed=1):
    """
    Change camera rangge by angle value (degrees)
    angle can be postitive or negative
    """
    cam=GE.GetCamera(False)
    cam.Range=length
    GE.setCamera(cam,speed)

def ge_cam_rotate(azimuth=None,tilt=None,diffazimuth=0,difftilt=0):
    pass

def ge_cam_diffrotate():
    pass
   
def ge_cam_difflook(difflatitude=0,difflongitude=0,speed=1):
    """ Looks like it doesn't work if tilt=0 """
    cam=GE.GetCamera(False)
    cam.FocusPointLatitude+=difflatitude
    cam.FocusPointLongitude+=difflongitude
    GE.setCamera(cam,speed)
   
def ge_getLocation():
    loc=GE.GetPointOnTerrainFromScreenCoords(0,-1)
    print(loc.Latitude)

def ge_loadKmlData():
    """ Load kml data from a string"""
    #Don't add all the kml header => pywintypes errors
    data="""
    <kml>
      <Placemark>
    <name>Simple placemark</name>
    <description>Attached to the ground. Intelligently places itself
       at the height of the underlying terrain.</description>
    <LookAt>
            <longitude>-122.0822035425683</longitude>
            <latitude>37.42228990140251</latitude>
            <range>1000</range>
            <tilt>0</tilt>
            <heading>0</heading>
    </LookAt>
    <Style>
            <IconStyle>
                <Icon>
                    <href>http://farm2.static.flickr.com/1233/1434233189_7137633b87_b.jpg</href>
                </Icon>
            </IconStyle>
        </Style>
    <Point>
      <coordinates>-122.0822035425683,37.42228990140251,0</coordinates>
    </Point>
    </Placemark>
    </kml>
    """
    GE.LoadKmlData(data)
    
   
def ge_openKmlFile(kml_file):
    """ Open an existing kml file by giving the full path with /s """
    GE.OpenKmlFile(kml_file,False)
   
if __name__=="__main__":
    ge_connect()   
   
    ge_cam_go_azimuth(-10)
    ge_cam_go_difftilt(-10)
    ge_cam_go_diffrange(-100000)
   
    ge_cam_difflook(difflatitude=1.1,difflongitude=1.1,speed=5)
   
    ge_getCameraInfo()       
    ge_openKmlFile()
    ge_loadKmlData()
    ge_getLocation()
    ge_getCameraInfo()
   
   
