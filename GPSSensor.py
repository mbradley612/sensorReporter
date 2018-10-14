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
from gps3 import gps3
import json


import ConfigParser
import traceback


# install dateutil using apt-get install python-dateutil

import dateutil.parser as dp
import time

class GPSSensor:
    """Represents a GPS sensor available via gpsd"""


    def __init__(self, connections, logger, params, sensors, actuators):
        """Sets the sensor pin to pud and publishes its current value"""
        self.poll = float(params("Poll"))
        self.lastpoll = time.time()
        self.destination = params("Destination")

        #Initialise to some time ago (make sure it publishes)
        self.lastPublish = time.time() - 1000

        self.logger = logger
        
        try:
            if (params("Filter")):
                self.filter = params("Filter").split(",")
            else:
                self.filter = None
            pass
            #if (params("Scale") == 'F'):
            #    self.useF = True
        except ConfigParser.NoOptionError:
            pass

        # connect to gpsd
        
        self.logger.info("----------Connecting to GPS Sensor")     
        self.gps_socket = gps3.GPSDSocket()
        
        self.gps_socket.connect()
        self.gps_socket.watch()
        

        self.publish = connections

        
        self.checkState()
    


    def checkState(self):
        # our state will always be different because our time will be different.
        
        # we want to continue to read from the gpsd until we get a TPV packet.
        
        try:
            
            tpvRead = False
            
            while not tpvRead:
                self.gpsJsonPacket = self.gps_socket.next()
                if self.gpsJsonPacket:
                
                    self.gpsPacket = json.loads(self.gpsJsonPacket)
                    
                    self.gpsSentenceType = self.gpsPacket['class']
                    
                    # we'll publish anything that comes out of gpsd, including any SKY sentences.
                    self.publishState()
                    
                    if self.gpsSentenceType == 'TPV':
                        tpvRead = True
                    
            
        except Exception as e:
            self.logger.error("----------Error reading from GPS " + str(e))
            
    def publishStateImpl(self, data, destination):
        for conn in self.publish:
            conn.publish(data, destination)

    def publishState(self):
        """Publishes the current state"""
        didPublish = False
                
        if (self.gpsJsonPacket):
            
            
            # if we have a filter, check that our sentence type is in the filter
            if self.filter and self.gpsSentenceType in self.filter:
                
                self.publishStateImpl(str(self.gpsJsonPacket) , self.destination + "/" + self.gpsSentenceType)
                didPublish = True
                
        if (didPublish):
            self.lastPublish = time.time()