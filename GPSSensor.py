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
import threading


import ConfigParser
import traceback


# install dateutil using apt-get install python-dateutil

import dateutil.parser as dp
import time

class GPSDReceiver:
    """
    Encapsulates the connection to GPSD
    Reads the stream of sentences from GPSD, and stores the latest version of each for polling.
    
    Runs its own thread for reading the stream from gpsd
    """
    def __init__(self, host="127.0.0.1",port=2937):
        
        self.gps = gps3.GPSDSocket()
        self.isRunning = False
        self.latestSentences = {}
        self.host = host
        self.port = port
        
    def run(self):
        self.gps.connect(host=self.host,port=self.port)
        self.gps.watch()
        
        self.isRunning = True
        
        while self.isRunning:
            # read the next packet, block for max of 0.25 seconds
            gpsJsonPacket = self.gps.next(timeout=0.25)
            
            if gpsJsonPacket:
                
                
                
                # load the json
                gpsPacket = json.loads(gpsJsonPacket)
                # retrieve the sentenceType
                sentenceType = gpsPacket['class']
                
                # update the latest data
                self.latestSentences[sentenceType] = gpsPacket
                
                
    def latestSentences(self):
        # shallow copy our dictionary and return it
        return dict(self.latestSentences)        
            
            
    def stop(self):
        self.isRunning = False
        self.gps.watch(enable=False)
        

class GPSSensor:
    """Represents a GPS sensor available via gpsd"""


    # we currently assume that GPSD is available. A more robust implementation
    # would try and reconnect if we get a connection failure.
    
    def __init__(self, connections, logger, params, sensors, actuators):
        """Sets the sensor pin to pud and publishes its current value"""
        #self.poll = float(params("Poll"))
        
        self.destination = params("Destination")

        #Initialise to some time ago (make sure it publishes)
        self.lastPublish = time.time() - 1000

        self.logger = logger
        
        self.poll = float(params("Poll"))

        
        
        
        
        
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
            
        try:
            self.sentenceTypes = params("SentenceTypes").split(",")
            
        
            
            #if (params("Scale") == 'F'):
            #    self.useF = True
        except ConfigParser.NoOptionError:
            # we default to TPV only
            self.sentenceTypes = "TPV"
        
        
        try:
            # we split the list on , and = characters
            passThruParamsList = params("passThruParams").split(";")
            # create a dictionary
            self.passThruParams = {}
            for param in passThruParamsList:
                
                (key,value) = param.split("=")
                self.passThruParams[key] = value
            
        
        except ConfigParser.NoOptionError:
            # we default to an empty dictionary
            self.passThruParams = {}

        self.publish = connections 
        
        # create our gpsreceiver object
        self.gpsdReceiver = GPSDReceiver(host=gpsHost, port=gpsPort)

        self.gpsdReceiverThread = threading.Thread(target = self.gpsdReceiver.run)
        self.gpsdReceiverThread.daemon=False
        
        self.gpsdReceiverThread.start()
        
        
        self.checkState()
        
        
    def cleanup(self):
        self.gpsdReceiver.isRunning = False

    def checkState(self):
        # strategy is to get the latest data from our receiver, filtered for the sentences that we are interested in
        
        latestSentences = self.gpsdReceiver.latestSentences
        
        self.filteredGpsData = {sentenceType: latestSentences[sentenceType] for sentenceType in self.sentenceTypes if sentenceType in latestSentences}
        
        
        self.publishState()
        
            
    def publishStateImpl(self, data, destination):
        for conn in self.publish:
            conn.publish(data, destination)

    def publishState(self):
        """Publishes the current state"""
        didPublish = False
        
        # add our passThruParams
        self.filteredGpsData.update(self.passThruParams)
        
        dataToPublishJson = json.dumps(self.filteredGpsData)
        
        self.publishStateImpl(dataToPublishJson , self.destination )
        didPublish = True 
                
        if (didPublish):
            self.lastPublish = time.time()