###############################################################################
# pyGEtools - Draft/examples/bits and bobs
# Control Google Earth through its API COM interface
# https://docs.google.com/document/d/1A4pHdyX2Rm693zYtXAMKHEhN5Boe4euX-siHKLC4noA/preview
###############################################################################

import time
import win32com.client
# from math import *

class GoogleEarth(object):
    '''
    Control Google Earth in Windows.
    '''
    def __init__(self, time_out=5):
        # connect to Google Earth
        self.ge = win32com.client.Dispatch("GoogleEarth.ApplicationGE")
        i = 0
        while not self.ge.IsInitialized():
            i = i + 1
            if i > int(time_out/0.5):
                raise EnvironmentError("Failed to launch Google Earth.")
            time.sleep(0.5)
            print(">>> Waiting for Google Earth to initialize.")
        print(">>>> Connection established.")

    def load_kml(self, kml_file):
        '''
        Open an existing kml file.
        '''
        self.ge.OpenKmlFile(kml_file, False)
