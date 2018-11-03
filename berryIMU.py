#!/usr/bin/python
#
#    This program  reads the angles from the acceleromter, gyrscope
#    and mangnetometeron a BerryIMU connected to a Raspberry Pi.
#
#    This program includes two filters (low pass and mdeian) to improve the
#    values returned from BerryIMU by reducing noise.
#
#
#    http://ozzmaker.com/
#    Both the BerryIMUv1 and BerryIMUv2 are supported
#
#    BerryIMUv1 uses LSM9DS0 IMU
#    BerryIMUv2 uses LSM9DS1 IMU
#


'''
This module provides a multi-client interface to a single object that measures IMU readings and
calculates filtered readings. We can only have one connection to the IMU interface via the I2C
physical interface, and to perform the filtered calculations, we need our own thread.

To create "singleton" type behaviour, we use the "Borg" pattern, see 
https://www.oreilly.com/library/view/python-cookbook/0596001673/ch05s23.html

Each client invokes our start method to start us, and invokes stop to stop us. We use this
to keep track of the number of clients. On first start, we start our thread. On last stop,
we stop our thread.

'''

import sys
import time
import math
import IMU
import datetime
import os
import threading
import abc
from Queue import Queue, Empty
import signal
# If the IMU is upside down (Skull logo facing up), change this value to 1
IMU_UPSIDE_DOWN = 0


RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070  	# [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA =  0.40      	# Complementary filter constant
MAG_LPF_FACTOR = 0.4 	# Low pass filter constant magnetometer
ACC_LPF_FACTOR = 0.4 	# Low pass filter constant for accelerometer
ACC_MEDIANTABLESIZE = 9    	# Median filter table size for accelerometer. Higher = smoother but a longer delay
MAG_MEDIANTABLESIZE = 9    	# Median filter table size for magnetometer. Higher = smoother but a longer delay

# how long to block waiting for a calculation
READ_TIMEOUT=0.5
# how long to sleep in each loop
LOOP_SLEEP=0.1
# internal string to represent a command
COMMAND_READ_IMU = "read_imu"
COMMAND_STOP = "stop"


################# Compass Calibration values ############
# Use calibrateBerryIMU.py to get calibration values
# Calibrating the compass isnt mandatory, however a calibrated
# compass will result in a more accurate heading value.

magXmin =  0
magYmin =  0
magZmin =  0
magXmax =  0
magYmax =  0
magZmax =  0


'''
Here is an example:
magXmin =  -1748
magYmin =  -1025
magZmin =  -1876
magXmax =  959
magYmax =  1651
magZmax =  708
Dont use the above values, these are just an example.
'''



#Kalman filter variables
Q_angle = 0.02
Q_gyro = 0.0015
R_angle = 0.005
y_bias = 0.0
x_bias = 0.0
XP_00 = 0.0
XP_01 = 0.0
XP_10 = 0.0
XP_11 = 0.0
YP_00 = 0.0
YP_01 = 0.0
YP_10 = 0.0
YP_11 = 0.0
KFangleX = 0.0
KFangleY = 0.0





		



def kalmanFilterY ( accAngle, gyroRate, DT):
	y=0.0
	S=0.0
	
	global KFangleY
	global Q_angle
	global Q_gyro
	global y_bias
	global YP_00
	global YP_01
	global YP_10
	global YP_11
	
	KFangleY = KFangleY + DT * (gyroRate - y_bias)
	
	YP_00 = YP_00 + ( - DT * (YP_10 + YP_01) + Q_angle * DT )
	YP_01 = YP_01 + ( - DT * YP_11 )
	YP_10 = YP_10 + ( - DT * YP_11 )
	YP_11 = YP_11 + ( + Q_gyro * DT )
	
	y = accAngle - KFangleY
	S = YP_00 + R_angle
	K_0 = YP_00 / S
	K_1 = YP_10 / S
	
	KFangleY = KFangleY + ( K_0 * y )
	y_bias = y_bias + ( K_1 * y )
	
	YP_00 = YP_00 - ( K_0 * YP_00 )
	YP_01 = YP_01 - ( K_0 * YP_01 )
	YP_10 = YP_10 - ( K_1 * YP_00 )
	YP_11 = YP_11 - ( K_1 * YP_01 )
	
	return KFangleY

def kalmanFilterX ( accAngle, gyroRate, DT):
	x=0.0
	S=0.0
	
	global KFangleX
	global Q_angle
	global Q_gyro
	global x_bias
	global XP_00
	global XP_01
	global XP_10
	global XP_11
	
	
	KFangleX = KFangleX + DT * (gyroRate - x_bias)
	
	XP_00 = XP_00 + ( - DT * (XP_10 + XP_01) + Q_angle * DT )
	XP_01 = XP_01 + ( - DT * XP_11 )
	XP_10 = XP_10 + ( - DT * XP_11 )
	XP_11 = XP_11 + ( + Q_gyro * DT )
	
	x = accAngle - KFangleX
	S = XP_00 + R_angle
	K_0 = XP_00 / S
	K_1 = XP_10 / S
	
	KFangleX = KFangleX + ( K_0 * x )
	x_bias = x_bias + ( K_1 * x )
	
	XP_00 = XP_00 - ( K_0 * XP_00 )
	XP_01 = XP_01 - ( K_0 * XP_01 )
	XP_10 = XP_10 - ( K_1 * XP_00 )
	XP_11 = XP_11 - ( K_1 * XP_01 )
	
	return KFangleX




#
# Internal class
#

class BerryIMUReader:
	def __init__(self):
		
		self.gyroXangle = 0.0
		self.gyroYangle = 0.0
		self.gyroZangle = 0.0
		self.CFangleX = 0.0
		self.CFangleY = 0.0
		self.CFangleXFiltered = 0.0
		self.CFangleYFiltered = 0.0
		self.kalmanX = 0.0
		self.kalmanY = 0.0
		self.oldXMagRawValue = 0
		self.oldYMagRawValue = 0
		self.oldZMagRawValue = 0
		self.oldXAccRawValue = 0
		self.oldYAccRawValue = 0
		self.oldZAccRawValue = 0
		
		self.a = datetime.datetime.now()
		
		
		
		#Setup the tables for the mdeian filter. Fill them all with '1' soe we dont get devide by zero error
		self.acc_medianTable1X = [1] * ACC_MEDIANTABLESIZE
		self.acc_medianTable1Y = [1] * ACC_MEDIANTABLESIZE
		self.acc_medianTable1Z = [1] * ACC_MEDIANTABLESIZE
		self.acc_medianTable2X = [1] * ACC_MEDIANTABLESIZE
		self.acc_medianTable2Y = [1] * ACC_MEDIANTABLESIZE
		self.acc_medianTable2Z = [1] * ACC_MEDIANTABLESIZE
		self.mag_medianTable1X = [1] * MAG_MEDIANTABLESIZE
		self.mag_medianTable1Y = [1] * MAG_MEDIANTABLESIZE
		self.mag_medianTable1Z = [1] * MAG_MEDIANTABLESIZE
		self.mag_medianTable2X = [1] * MAG_MEDIANTABLESIZE
		self.mag_medianTable2Y = [1] * MAG_MEDIANTABLESIZE
		self.mag_medianTable2Z = [1] * MAG_MEDIANTABLESIZE
		
		
		# create a queue for handling commands
		self.commandQueue = Queue()
		# create a queue for reading the latest response
		self.latestImuQueue = Queue()
		
		IMU.detectIMU()     #Detect if BerryIMUv1 or BerryIMUv2 is connected.
		IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass
		
	
	def stop(self):
		self.commandQueue.put(COMMAND_STOP)
	
	
	'''
	return the latestIMU reading, or None if none available
	'''
	def readLatestIMU(self):
		self.commandQueue.put(COMMAND_READ_IMU)
		
		try:
			latestIMU =  self.latestImuQueue.get(timeout=READ_TIMEOUT)
			return latestIMU
		except Empty:
			latestIMU = None
			
		return None

	# this should be the target for a thread to start
	def run(self):
		self.isRunning = True
		
		while self.isRunning:
			#Read the accelerometer,gyroscope and magnetometer values
			ACCx = IMU.readACCx()
			ACCy = IMU.readACCy()
			ACCz = IMU.readACCz()
			GYRx = IMU.readGYRx()
			GYRy = IMU.readGYRy()
			GYRz = IMU.readGYRz()
			MAGx = IMU.readMAGx()
			MAGy = IMU.readMAGy()
			MAGz = IMU.readMAGz()
			
			
			#Apply compass calibration
			MAGx -= (magXmin + magXmax) /2
			MAGy -= (magYmin + magYmax) /2
			MAGz -= (magZmin + magZmax) /2
			
			
			##Calculate loop Period(LP). How long between Gyro Reads
			b = datetime.datetime.now() - self.a
			a = datetime.datetime.now()
			LP = b.microseconds/(1000000*1.0)
			#print "Loop Time | %5.2f|" % ( LP ),
			
			
			
			###############################################
			#### Apply low pass filter ####
			###############################################
			MAGx =  MAGx  * MAG_LPF_FACTOR + self.oldXMagRawValue*(1 - MAG_LPF_FACTOR);
			MAGy =  MAGy  * MAG_LPF_FACTOR + self.oldYMagRawValue*(1 - MAG_LPF_FACTOR);
			MAGz =  MAGz  * MAG_LPF_FACTOR + self.oldZMagRawValue*(1 - MAG_LPF_FACTOR);
			ACCx =  ACCx  * ACC_LPF_FACTOR + self.oldXAccRawValue*(1 - ACC_LPF_FACTOR);
			ACCy =  ACCy  * ACC_LPF_FACTOR + self.oldYAccRawValue*(1 - ACC_LPF_FACTOR);
			ACCz =  ACCz  * ACC_LPF_FACTOR + self.oldZAccRawValue*(1 - ACC_LPF_FACTOR);
			
			#print "mag lpf factor:"+str(MAG_LPF_FACTOR)
			
			self.oldXMagRawValue = MAGx
			self.oldYMagRawValue = MAGy
			self.oldZMagRawValue = MAGz
			self.oldXAccRawValue = ACCx
			self.oldYAccRawValue = ACCy
			self.oldZAccRawValue = ACCz
			
			#########################################
			#### Median filter for accelerometer ####
			#########################################
			# cycle the table
			for x in range (ACC_MEDIANTABLESIZE-1,0,-1 ):
				self.acc_medianTable1X[x] = self.acc_medianTable1X[x-1]
				self.acc_medianTable1Y[x] = self.acc_medianTable1Y[x-1]
				self.acc_medianTable1Z[x] = self.acc_medianTable1Z[x-1]
				
			# end for x
			
			# Insert the lates values
			self.acc_medianTable1X[0] = ACCx
			self.acc_medianTable1Y[0] = ACCy
			self.acc_medianTable1Z[0] = ACCz
			
			# Copy the tables
			self.acc_medianTable2X = self.acc_medianTable1X[:]
			self.acc_medianTable2Y = self.acc_medianTable1Y[:]
			self.acc_medianTable2Z = self.acc_medianTable1Z[:]
			
			# Sort table 2
			self.acc_medianTable2X.sort()
			self.acc_medianTable2Y.sort()
			self.acc_medianTable2Z.sort()
			
			# The middle value is the value we are interested in
			ACCx = self.acc_medianTable2X[ACC_MEDIANTABLESIZE/2];
			ACCy = self.acc_medianTable2Y[ACC_MEDIANTABLESIZE/2];
			ACCz = self.acc_medianTable2Z[ACC_MEDIANTABLESIZE/2];
			
			
			
			#########################################
			#### Median filter for magnetometer ####
			#########################################
			# cycle the table
			for x in range (MAG_MEDIANTABLESIZE-1,0,-1 ):
				self.mag_medianTable1X[x] = self.mag_medianTable1X[x-1]
				self.mag_medianTable1Y[x] = self.mag_medianTable1Y[x-1]
				self.mag_medianTable1Z[x] = self.mag_medianTable1Z[x-1]
			
			# end for x
			
			# Insert the latest values
			self.mag_medianTable1X[0] = MAGx
			self.mag_medianTable1Y[0] = MAGy
			self.mag_medianTable1Z[0] = MAGz
			
			# Copy the tables
			self.mag_medianTable2X = self.mag_medianTable1X[:]
			self.mag_medianTable2Y = self.mag_medianTable1Y[:]
			self.mag_medianTable2Z = self.mag_medianTable1Z[:]
			
			# Sort table 2
			self.mag_medianTable2X.sort()
			self.mag_medianTable2Y.sort()
			self.mag_medianTable2Z.sort()
			
			# The middle value is the value we are interested in
			MAGx = self.mag_medianTable2X[MAG_MEDIANTABLESIZE/2];
			MAGy = self.mag_medianTable2Y[MAG_MEDIANTABLESIZE/2];
			MAGz = self.mag_medianTable2Z[MAG_MEDIANTABLESIZE/2];
			
			
			
			#Convert Gyro raw to degrees per second
			rate_gyr_x =  GYRx * G_GAIN
			rate_gyr_y =  GYRy * G_GAIN
			rate_gyr_z =  GYRz * G_GAIN
			
			
			#Calculate the angles from the gyro.
			self.gyroXangle+=rate_gyr_x*LP
			self.gyroYangle+=rate_gyr_y*LP
			self.gyroZangle+=rate_gyr_z*LP
			
			#Convert Accelerometer values to degrees
			
			if not IMU_UPSIDE_DOWN:
			# If the IMU is up the correct way (Skull logo facing down), use these calculations
				self.AccXangle =  (math.atan2(ACCy,ACCz)*RAD_TO_DEG)
				self.AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG
			else:
			#Us these four lines when the IMU is upside down. Skull logo is facing up
				self.AccXangle =  (math.atan2(-ACCy,-ACCz)*RAD_TO_DEG)
				self.AccYangle =  (math.atan2(-ACCz,-ACCx)+M_PI)*RAD_TO_DEG
			# end if else
			
			
			#Change the rotation value of the accelerometer to -/+ 180 and
			#move the Y axis '0' point to up.  This makes it easier to read.
			if self.AccYangle > 90:
				self.AccYangle -= 270.0
			else:
				self.AccYangle += 90.0
			# end if else
			
			
			#Complementary filter used to combine the accelerometer and gyro values.
			self.CFangleX=AA*(self.CFangleX+rate_gyr_x*LP) +(1 - AA) * self.AccXangle
			self.CFangleY=AA*(self.CFangleY+rate_gyr_y*LP) +(1 - AA) * self.AccYangle
			
			#Kalman filter used to combine the accelerometer and gyro values.
			self.kalmanY = kalmanFilterY(self.AccYangle, rate_gyr_y,LP)
			self.kalmanX = kalmanFilterX(self.AccXangle, rate_gyr_x,LP)
			
			if IMU_UPSIDE_DOWN:
				MAGy = -MAGy      #If IMU is upside down, this is needed to get correct heading.
			# end if
			
			#Calculate heading
			self.heading = 180 * math.atan2(MAGy,MAGx)/M_PI
			
			#Only have our heading between 0 and 360
			if self.heading < 0:
				self.heading += 360
			# end if
			
			
			####################################################################
			###################Tilt compensated heading#########################
			####################################################################
			#Normalize accelerometer raw values.
			if not IMU_UPSIDE_DOWN:
			#Use these two lines when the IMU is up the right way. Skull logo is facing down
				accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
				accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
			else:
			#Us these four lines when the IMU is upside down. Skull logo is facing up
				accXnorm = -ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
				accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
			# end if else
			
			#Calculate pitch and roll
			
			pitch = math.asin(accXnorm)
			roll = -math.asin(accYnorm/math.cos(pitch))
			
			
			#Calculate the new tilt compensated values
			magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
			
			#The compass and accelerometer are orientated differently on the LSM9DS0 and LSM9DS1 and the Z axis on the compass
			#is also reversed. This needs to be taken into consideration when performing the calculations
			if(IMU.LSM9DS0):
				magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)   #LSM9DS0
			else:
				magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)   #LSM9DS1
			# end if else
			
			
			
			#Calculate tilt compensated heading
			self.tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI
			
			if self.tiltCompensatedHeading < 0:
				self.tiltCompensatedHeading += 360
			# end if
			
			############################ END ##################################
			
			
			if 0:			#Change to '0' to stop showing the angles from the accelerometer
				print ("# ACCX Angle %5.2f ACCY Angle %5.2f #  " % (self.AccXangle, self.AccYangle)),
			
			if 0:			#Change to '0' to stop  showing the angles from the gyro
				print ("\t# GRYX Angle %5.2f  GYRY Angle %5.2f  GYRZ Angle %5.2f # " % (self.gyroXangle,self.gyroYangle,self.gyroZangle)),
			
			if 0:			#Change to '0' to stop  showing the angles from the complementary filter
				print ("\t# self.CFangleX Angle %5.2f   self.CFangleY Angle %5.2f #" % (self.CFangleX,self.CFangleY)),
			
			if 0:			#Change to '0' to stop  showing the heading
				print ("\t# HEADING %5.2f  tiltCompensatedHeading %5.2f #" % (self.heading,self.tiltCompensatedHeading)),
			
			if 0:			#Change to '0' to stop  showing the angles from the Kalman filter
				print ("# self.kalmanX %5.2f   self.kalmanY %5.2f #" % (self.kalmanX,self.kalmanY)),
			
			    #print a new line
			if 0:
				print ""
			
			try:
				# check the command queue
				if not self.commandQueue.empty():
					# we have a command, so get it. We should only have one thread reading this queue, 
					# so there should always be something there.
					command = self.commandQueue.get_nowait()
					
					if command==COMMAND_READ_IMU:
						
						latestImu = {
									"accXangle": self.AccXangle,
									"accYangle": self.AccYangle,
									"gyroXangle": self.gyroXangle,
									"gyroYangle": self.gyroYangle,
									"gyroZangle": self.gyroZangle,
									"CFangleX" : self.CFangleX,
									"CFangleY" : self.CFangleY,
									"CFangleXFiltered" : self.CFangleXFiltered,
									"CFangleYFiltered" : self.CFangleYFiltered,
									"heading" : self.heading,
									"tiltCompensatedHeading" : self.tiltCompensatedHeading,
									"kalmanX" : self.kalmanX,
									"kalmanY" : self.kalmanY
									}
						self.latestImuQueue.put(latestImu)
					if command==COMMAND_STOP:
						self.isRunning = False
			except Empty:
				# this shouldn't happen
				pass
					
			
			    
			#slow program down a bit, makes the output more readable
			time.sleep(LOOP_SLEEP)
		print "BerryIMUReader loop ended"
		
'''
This class is for use by the external facing functions.
'''

class BerryIMU():
	

	def disconnect(self):
		global _connectedClientCount
		_connectedClientCount = _connectedClientCount -1
		# if this our last client, tell the reader to stop
		if _connectedClientCount == 0:
			_berryIMUReader.stop()
		
	def readLatestIMU(self):
		return readLatestIMU()



_berryIMUReader = BerryIMUReader()
_connectedClientCount = 0
_runnerThread = threading.Thread(target=_berryIMUReader.run)
_runnerThread.daemon = False

def connect():
	global _connectedClientCount
	global _runnerThread
	
	if _connectedClientCount == 0:
		# if this is our first client, increment count and start our thread
		_connectedClientCount = _connectedClientCount + 1
		_runnerThread.start()
	else:
		# otherwise just increment our count
		_connectedClientCount + _connectedClientCount + 1
	
	# if this is our first client, start our reader
	
	return BerryIMU() 

def readLatestIMU():
	return _berryIMUReader.readLatestIMU()
 
 
def service_shutdown(signum, frame):
	global isRunning
	print "Handling shutdown" 
	isRunning = False


# this code runs if this module is run as the main module	
if __name__ == "__main__":


	signal.signal(signal.SIGTERM, service_shutdown)
	signal.signal(signal.SIGINT, service_shutdown)

	
	# module wide variable. This gets run when the module is loaded
	myIMU = connect()
	myIMU2 = connect()
	isRunning = True
	
	
	
	while isRunning:
	
		#latest = latestIMU()
		latest = myIMU.readLatestIMU()
		latest2 = myIMU2.readLatestIMU()
		if latest:
			print str("1: " + str(latest))
		if latest2:
			print("\n 2:" + str(latest2))
		time.sleep(0.5)

	myIMU.disconnect()
	myIMU2.disconnect()
	
	print "Shutdown complete"