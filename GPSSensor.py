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


    # we currently assume that GPSD is available. A more robust implementation
    # would try and reconnect if we get a connection failure.
    
    def __init__(self, connections, logger, params, sensors, actuators):
        """Sets the sensor pin to pud and publishes its current value"""
        self.poll = float(params("Poll"))
        self.lastpoll = time.time()
        self.destination = params("Destination")

        #Initialise to some time ago (make sure it publishes)
        self.lastPublish = time.time() - 1000

        self.logger = logger
        
        try:
            self.sentenceTypes = params("SentenceTypes").split(",")
            
            self.timeout = float(params("Timeout"))
            
            #if (params("Scale") == 'F'):
            #    self.useF = True
        except ConfigParser.NoOptionError:
            pass
        
        try:
            gpsConnection = params("GpsConnection")
            
            gpsHost, gpsPort = gpsConnection.split(':')
            #gpsConnectionList = gpsConnection.split(':')
            #gpsHost = gpsConnectionList[0]
            #gpsPort = integer(gpsConnectionList[1])
            #if (params("Scale") == 'F'):
            #    self.useF = True
        except ConfigParser.NoOptionError:
            gpsHost, gpsPort = ['localhost',2947]

        # connect to gpsd
        
        self.logger.info("----------Connecting to GPS Sensor")     
        self.gps_socket = gps3.GPSDSocket()
        
        
        
        self.gps_socket.connect(host=gpsHost,port=gpsPort)
        self.gps_socket.watch()
        

        self.publish = connections

        
        self.checkState()
        
        
    def readGpsd(self, sentenceType, timeout=0):
        """Carry on reading from gpsd until we read the target sentencetype, or timeout.
        Arguments:
        timeout: Default timeout=0  range zero to float specifies a time-out as a floating point
        number in seconds.  Will sit and wait for timeout seconds.  When the timeout argument is omitted
        the function blocks until at least one file descriptor is ready. A time-out value of zero specifies
        a poll and never blocks.
        """
        try:
            
            hasReadSentenceType = False
            gpsPacket = None
            remainingTimeout = timeout
            
            # need to calculate the remaining timeout
            while not hasReadSentenceType and (timeout == 0 or remainingTimeout > 0):
                startTime = time.time()
                self.gpsJsonPacket = self.gps_socket.next()#timeout = remainingTimeout)
                endTime = time.time()
                remainingTimeout = remainingTimeout - (endTime - startTime)
                
                if self.gpsJsonPacket:
                
                    gpsPacket = json.loads(self.gpsJsonPacket)
                    
                    if gpsPacket['class'] == sentenceType:
                        hasReadSentenceType = True
            
            return gpsPacket
            
        except Exception as e:
            self.logger.error("----------Error reading from GPS " + str(e))
            
            print(traceback.format_exc())
    


    def checkState(self):
        # we have a logical order to read in, based on the period that the data comes from our sensors
        
        # SKY - every few seconds.
        # TPV - approx 1 second
        # IMU - instant read
        
        # create a dictionary for our data to publish
        
        self.dataToPublish = {}
        
        # take a copy of the sentences that we need to read
        sentencesToRead = list(self.sentenceTypes)
        
        # capture the current time
        startTime = time.time()
        
        # calculate our end time
        endTime = startTime + self.timeout
        
        # we loop reading from gpsd until we're out of time
        while len(sentencesToRead) > 0 and time.time() < endTime:
            
            try:
                
                # read the next packet from gpsd
                gpsJsonPacket = self.gps_socket.next()
                
                             
                if gpsJsonPacket:
                    # convert from Json to dictionary
                    gpsPacket = json.loads(gpsJsonPacket)
                    
                    # figure out what class it is
                    gpsPacketClass = gpsPacket['class']
                    
                    # if we haven't already read this class of packet, remove it from our sentences to read
                    if gpsPacketClass in sentencesToRead:
                        
                        sentencesToRead.remove(gpsPacketClass)
                    
                    # if this is one of our sen
                        
                    # update our data
                    self.dataToPublish[gpsPacketClass] = gpsPacket
                    
            except Exception as e:
                self.logger.error("----------Error reading from GPS " + str(e))
                
                print(traceback.format_exc())
        
        if len(self.dataToPublish) > 0:
        
            self.publishState()   
            
    def publishStateImpl(self, data, destination):
        for conn in self.publish:
            conn.publish(data, destination)

    def publishState(self):
        """Publishes the current state"""
        didPublish = False
        
        dataToPublishJson = json.dumps(self.dataToPublish)
        
        self.publishStateImpl(dataToPublishJson , self.destination )
        didPublish = True
                
        if (didPublish):
            self.lastPublish = time.time()