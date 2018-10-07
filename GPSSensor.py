"""
   Copyright 2018 Matthew Bradley

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

 Script: GPSSensor.py
 Author: Matthew Bradley
 Date:   24 June 2018
 Purpose: Reads the GPS state and publishes any changes
"""
import gpsd


import ConfigParser


# install dateutil using apt-get install python-dateutil

import dateutil.parser as dp
import time



class GPSSensor:
    """Represents a GPS sensor available via gpsd"""


    def __init__(self, connections, logger, params, sensors, actuators):
        """Sets the sensor pin to pud and publishes its current value"""

        self.lastpoll = time.time()

        #Initialise to some time ago (make sure it publishes)
        self.lastPublish = time.time() - 1000

        self.logger = logger
        
        try:
            pass
            #if (params("Scale") == 'F'):
            #    self.useF = True
        except ConfigParser.NoOptionError:
            pass

        # connect to gpsd
        
        self.logger.info("----------Connecting to GPS Sensor")     
        gpsd.connect()
        
        

        self.publish = connections

        
        self.checkState()
    


    def checkState(self):
        # our state will always be different because our time will be different.
        
        try:
            self.lastGpsPacket = gpsd.get_current()
             
            self.publishState()
        except Exception as e:
            self.logger.error("----------Error reading from GPS " + str(e))

    def publishStateImpl(self, data, destination):
        for conn in self.publish:
            conn.publish(data, destination)

    def publishState(self):
        """Publishes the current state"""
        didPublish = False
        
        '''
        :var self.mode: Indicates the status of the GPS reception, 0=No value, 1=No fix, 2=2D fix, 3=3D fix
        :var self.sats: The number of satellites received by the GPS unit
        :var self.lon: Longitude in degrees
        :var self.lat: Latitude in degrees
        :var self.alt: Altitude in meters
        :var self.track: Course over ground, degrees from true north
        :var self.hspeed: Speed over ground, meters per second
        :var self.climb: Climb (positive) or sink (negative) rate, meters per second
        :var self.time: Time/date stamp in ISO8601 format, UTC. May have a fractional part of up to .001sec precision.
        :var self.error: GPSD error margin information

        GPSD error margin information
        -----------------------------

        c: ecp: Climb/sink error estimate in meters/sec, 95% confidence.
        s: eps: Speed error estinmate in meters/sec, 95% confidence.
        t: ept: Estimated timestamp error (%f, seconds, 95% confidence).
        v: epv: Estimated vertical error in meters, 95% confidence. Present if mode is 3 and DOPs can be
                calculated from the satellite view.
        x: epx: Longitude error estimate in meters, 95% confidence. Present if mode is 2 or 3 and DOPs
                can be calculated from the satellite view.
        y: epy: Latitude error estimate in meters, 95% confidence. Present if mode is 2 or 3 and DOPs can
                be calculated from the satellite view.
        '''
        
        if (self.lastGpsPacket):
            gpsJson = lastGpsPacket.dumps(lastGpsPacket)
            self.publishStateImpl(gpsJson, self.destination + "/gpsInfo")
            disPublish = True
            
        if (didPublish):
            self.lastPublish = time.time()


    
