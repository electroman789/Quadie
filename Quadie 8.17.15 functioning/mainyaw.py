#!/usr/bin/python 

##############################################################################################
#                                   Q U A D I E
#   authors: Christian Hinostroza & Alexander Sanchez
#   Code designed for the Engineering Design 2 Project of Florida Atlantic University 
#   term: Winter 2015
#   Date: 3-27-2015
#############################################################################################
import sys
import time
import math
import threading
import logging
from PWM import pwmMain as pwm
from Libraries import FreeIMU as FreeIMU
from Libraries import controller as Controller
from PID import pid as pid
from Libraries import BNO055 as BNO055
#from PING import PingSensor as PING
import Queue
import socket

#------------ TIMER constant for watchdog timeout ----------------------
TIMER = 2000
#------------- BootTime, Sensor filtering needs abotu 60 seconds to reach normality-----
bootTime = 2
#-----------------Inverted boolean, for controller direction--------------------
inverted = True

#--------------------- Constructors and variables------------------------------
activeThreads = True
timer = TIMER

RAD_DEG = 57.29578
M_PI = 3.141592
G_GAIN = 0.070
LP = 0.008
CF_GAIN = 0.80
gyroXangle = 0.0
gyroYangle = 0.0
gyroZangle = 0.0

MOTOR_FL = "1"
MOTOR_FR = "3"
MOTOR_BL = "2"
MOTOR_BR = "4"

rcpit = 0.0
rcroll = 0.0
rcyaw = 0.0
command = ""
yaw = 0.0
pitch = 0.0
roll = 0.0
thr = 0.0
EX = 0.0
EY = 0.0
EZ = 0.0
GX = 0.0
GY = 0.0
GZ = 0.0
controller = Controller.Controller()
pwm = pwm.PWMMotor()
sensors = BNO055.BNO055()
#sensors = FreeIMU.FreeIMU()
#ping = PING.PingSensor()
#####################################################################################
#                                       PID setups                                  #
#####################################################################################
PID_PITCH_RATE = pid.PID()
PID_ROLL_RATE = pid.PID()
PID_PITCH_STAB = pid.PID()
PID_ROLL_STAB = pid.PID()
PID_YAW_RATE = pid.PID()
PID_YAW_STAB = pid.PID()

#__________________ SLOW VALUES______________________________
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#----- Good control 4/28/2015 --------------------------------
#PID_PITCH_RATE.definePID(0.9,0.002,0.001)
#PID_ROLL_RATE.definePID(1.2,0.002,0.001)
#PID_YAW_RATE.definePID(  ,  ,  )
#-------------------------------------------------------------

###############################################################
#                          RATE PIDS
#
#PID_PITCH_RATE.definePID(0.9,0.002,0.001)
#PID_ROLL_RATE.definePID(1.2,0.002,0.001)
#PID_YAW_RATE.definePID(  ,  ,  )
###############################################################

PID_PITCH_RATE.definePID(0.9,0.0,0.0)
PID_ROLL_RATE.definePID(6.0,0.00,0.00)
PID_YAW_RATE.definePID(1.0,0.002,0.001)

pitchRateScaler = 1 #20.0 / (1.1 * 100.0)
rollRateScaler = 1 #20.0 / (1.2 * 100.0)
yawRateScaler = 20.0 / (1.0 * 100.0)
gyroPitchScaler = 0.02
gyroRollScaler = 0.0166
gyroYawScaler = 0.0125

PID_PITCH_RATE.set_windup_bounds(-6,6)
PID_ROLL_RATE.set_windup_bounds(-6,6)
PID_YAW_RATE.set_windup_bounds(-6,6)

##################################################################
#                       STAB PID  
#  
#PID_PITCH_STAB.definePID(0.080,0.0008,0.1)
#PID_ROLL_STAB.definePID(0.085,0.00008,0.1)
#PID_ROLL_STAB.definePID(  ,  ,  )
##################################################################

PID_PITCH_STAB.definePID(0.08,0.01,0.002)
PID_ROLL_STAB.definePID(0.085,0.00008,0.1)
PID_YAW_STAB.definePID(0.085,0.00008,0.1)


PID_PITCH_STAB.set_windup_bounds(-10,10)
PID_ROLL_STAB.set_windup_bounds(-20,20)
PID_YAW_STAB.set_windup_bounds(-20,20)
logging.basicConfig(level=logging.DEBUG,format='(%(threadName)-9s) %(message)s',)

#####################################################################################
#                                   Q U E U E ' S                                   #
#####################################################################################

S_BUF_SIZE = 5
BUF_SIZE = 5
#yq = Queue.Queue(S_BUF_SIZE)
#pq = Queue.Queue(S_BUF_SIZE)
#rq = Queue.Queue(S_BUF_SIZE)
dq = Queue.Queue(BUF_SIZE)
#grq = Queue.Queue(S_BUF_SIZE)
#gpq = Queue.Queue(S_BUF_SIZE)
#aq = Queue.Queue(S_BUF_SIZE)
#baq = Queue.Queue(S_BUF_SIZE)

#####################################################################################
#                             Initialize Sockets                                    #
#####################################################################################

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

##########################################################################################
#                                                                                        #
#                               Constrain Function                                       #
#                                                                                        #
##########################################################################################

def constrain(amt,low,high):
    """
    Constrain value --amt 
    to be within the low and high 
    """
    if (amt < low):
        return low
    elif (amt > high):
        return high
    else:
        return amt

#############################################################################################
#                                                                                           #
#                                   Sensor reading thread                                   #
#                                                                                           #
#############################################################################################

class SensorReading(threading.Thread):
    def __init__(self, group=None,target=None, name=None, args=(), kwargs=None, verbose=None):
        super(SensorReading,self).__init__()
        self.target = target
        self.name = name

    def run(self):
	
	y = 0.0

	RAD_DEG = 57.29578
	M_PI = 3.141592
	G_GAIN = 0.070
	LP = 0.01
	CF_GAIN = 0.90

	gyroXangle = 0.0
	gyroYangle = 0.0
	gyroZangle = 0.0
	CFangleX = 0.0
	CFangleY = 0.0

        while True and activeThreads:
            #y,r,p = sensors.getYawPitchRoll()
            #print "y = %f p = %f r = %f"%(y,p,r)
            apx,ary,ayz,gpx,gry,gyz,mpx,mry,myz = sensors.getValues()
            #print "gyroscope x = %f  y = %f   z = %f"%(gy,gp,gr)
            #baro = sensors.getBaroAlt()
            
	    a = time.clock()

	    AccXangle = (math.atan2(ary, ayz) + M_PI) * RAD_DEG
	    AccYangle = (math.atan2(ayz, apx) + M_PI) * RAD_DEG

	    rate_gyr_x = gpx * G_GAIN
	    rate_gyr_y = gry * G_GAIN
	    rate_gyr_z = gyz * G_GAIN

	    gyroXangle += rate_gyr_x * LP
	    gyroYangle += rate_gyr_y * LP
	    gyroZangle += rate_gyr_z * LP

	    AccXangle = AccXangle - 180.0
	    if (AccYangle > 90.0):
	    	AccYangle -= 270.0
	    else:
	    	AccYangle += 90.0

	    CFangleX = CF_GAIN * (CFangleX + rate_gyr_x * LP) + (1 - CF_GAIN) * AccXangle
	    CFangleY = CF_GAIN * (CFangleY + rate_gyr_y * LP) + (1 - CF_GAIN) * AccYangle
	    
	    #time.sleep(0.3)
	    b = time.clock()
	    c = b - a

	    print "x = %0.2f	y = %0.2f	time = %0.6f" %(CFangleX, CFangleY, c)
	
	    if not yq.full():
		yq.put(y)
                #logging.debug('Putting yaw ' + str(y) + ' : ' + str(yq.qsize()) + 'readings in queue')
            if not pq.full():
                pq.put(CFangleX)
		#pq.put(p)
                #logging.debug('Putting pitch ' + str(p) + ' : ' + str(pq.qsize()) + 'readings in queue')
            if not rq.full():
                pq.put(CFangleY)
		#rq.put(r)
                #logging.debug('Putting roll ' + str(r) + ' : ' + str(rq.qsize()) + 'readings in queue')
            #if not grq.full():
            #    grq.put(gr)
            #if not gpq.full():
            #    gpq.put(gp)
            #if not baq.full():
            #    baq.put(0.0)

        return

#############################################################################################
#                                                                                           #
#                                   Ping sensor reading thread                              #
#                                                                                           #
#############################################################################################

class PingReading(threading.Thread):
    def __init__(self, group=None,target=None, name=None, args=(), kwargs=None, verbose=None):
        super(PingReading,self).__init__()
        self.target = target
        self.name = name

    def run(self):
        while True and activeThreads:
            altitude = ping.GetAlt()
            if not aq.full():
                aq.put(altitude)
                #logging.debug('Putting yaw ' + str(y) + ' : ' + str(yq.qsize()) + 'readings in queue')
        return

#############################################################################################
#                                                                                           #
#                                 Debug print thread                                        #
#                             NOT NEEDED FOR FUNCTIONALITY                                  #
#############################################################################################

class PrintReading(threading.Thread):
    def __init__(self, group=None,target=None, name=None, args=(), kwargs=None, verbose=None):
        super(PrintReading,self).__init__()
        self.target = target
        self.name = name
        return

    def run(self):
        while True:
            if not yq.empty():
                yaw = yq.get()
                #logging.debug('Getting yaw value  ' + str(item) + ' : ' + str(yq.qsize()) + 'readings in queue')
            if not pq.empty():
                pitch = pq.get()
            if not rq.empty():
                roll = rq.get()

            #print "yaw = %d o  pitch = %d o  roll = %d o"%(yaw,pitch,roll)
        return

#############################################################################################
#                                                                                           #
#                                   Tablet Thread                                           #
#                                                                                           #
#############################################################################################

class TabletThread(threading.Thread):
    def __init__(self, group=None,target=None, name=None, args=(), kwargs=None, verbose=None):
        super(TabletThread,self).__init__()
        self.target = target
        self.name = name       
        return
    
    def run(self):
        s.bind(("", 5000))
        print "waiting on port: 5000"
        while True and activeThreads:
            data, addr = s.recvfrom(1024)
            if not dq.full():
                dq.put(data)        
        return
    
    def __del__(self):
        print "It did stop"

#############################################################################################
#                                                                                           #
#                                      Main Thread                                          #
#                                                                                           #
#############################################################################################

if __name__ == '__main__':
    #---------------------------START THREADS---------------------------------------------
    #readings = SensorReading(name='freeimu')
    tabletreadings = TabletThread(name='Android')
    #pingreadings = PingReading(name='PING')
    #printreads = PrintReading(name='printvalues')
    #readings.daemon = True
    tabletreadings.daemon = True
    #pingreadings.daemon = True
    #readings.start()
    time.sleep(2)
    tabletreadings.start()
    time.sleep(2)
    #pingreadings.start()
    #time.sleep(2)
    #printreads.start()
    print "-----------------------------------------------"
    print "_______________________________________________"
    print "						QUADIE   				  "
    print "|||||||||||||||||||||||||||||||||||||||||||||||"
    print "		    Group 4  ENGINEERING DESIGN           "
    print "############         HARDWARE      ############"
    print "  Chancey Kelley , Brain Chau, Chris Daugherty "
    print "$$$$$$$$$$$$       SOFTWARE       $$$$$$$$$$$$$"
    print "    Alexander Sanchez    Christian Hinostroza  "
    print "_______________________________________________"
    print " .............................................."
    print " "
    print "Booting..."
    #+++++++++++++++++++++++++++++++++++++++++++
    #sleep time for sensor readings to stabilize
    #++++++++++++++++++++++++++++++++++++++++++
    for i in range(bootTime):
        time.sleep(1)
        print "\r%d"%(bootTime-i)    
    print "Quadie will now take fly........"

    #######################################################################################
    #                                   MAIN LOOP                                         #
    #######################################################################################

    try:
        while True:
	    a = time.time()
            if timer < 0 :
                thr -= 0.0001

            ##################################################################################
            #                               GET COMMANDS                                     #
            ##################################################################################

            if not dq.empty():
                command = dq.get()
                if ',' in command:
                    rcpit,rcroll = controller.getPitchRoll(command)
                    rcroll = -rcroll    #testing roll
                    #rcyaw = rcroll      #testing yaw
		    #print "rcpit = %f  rcroll = %f"%(rcpit,rcroll)
                else:
                    rcpit = 0.0
                    rcroll = 0.0
                    rcyaw = 0.0
		    if command == "up":
                        thr += 2
                    elif command == "down":
                        thr -= 2
                    elif command == "stop":
                        thr = 0
                        sys.exit("Quaddie was stopped from tablet")
                timer = TIMER
                print "Tablet: %s"%command
            elif dq.empty():
                #timer -= 1    #for testing outside
		timer -= 0
            #print "rcpit = %f  rcroll = %f"%(rcpit,rcroll)
            #print "timer = %d"%timer

            ##################################################################################
            #                               GET POSITIONING                                  #
            ##################################################################################
            #if not yq.empty():
            #    yaw = yq.get()
            #if not pq.empty():
            #    pitch = pq.get()
            #if not rq.empty():
            #    roll = -rq.get()
            #b1,b2,b3,gyropitch,gyroroll,gyroyaw,pn,rn,dn = sensors.getValues()
            #if not aq.empty():
            #    alt = aq.get()
            #if not baq.empty():
            #    b_alt = baq.get()
                #debugging
                #logging.debug('Getting yaw value  ' + str(item) + ' : ' + str(yq.qsize()) + 'readings in queue')
            #print "yaw = %f o  pitch = %f o  roll = %f o"%(yaw,pitch,roll)
            #print "Altitude = %f  "%alt

            #apx,ary,ayz,gpx,gry,gyz,mpx,mry,myz = sensors.getValues()
            EX, EY, EZ = sensors.readEuler()
	    EX /= 16.0
            EY /= 16.0
            EZ /= 16.0
            GX, GY, GZ = sensors.getGyro()
            GX /= 16.0
            GY /= 16.0
            GZ /= 16.0
            
            a = time.clock()

#	    AccXangle = (math.atan2(ary, ayz) + M_PI) * RAD_DEG
#	    AccYangle = (math.atan2(ayz, apx) + M_PI) * RAD_DEG

#	    rate_gyr_x = gpx * G_GAIN
#	    rate_gyr_y = gry * G_GAIN
#	    rate_gyr_z = gyz * G_GAIN

#	    gyroXangle += rate_gyr_x * LP
#	    gyroYangle += rate_gyr_y * LP
#	    gyroZangle += rate_gyr_z * LP

#	    AccXangle = AccXangle - 180.0
#	    if AccYangle > 90.0:
#	    	AccYangle -= 270.0
#	    else:
#	    	AccYangle += 90.0

#	    pitch = CF_GAIN * (pitch + rate_gyr_x * LP) + (1 - CF_GAIN) * AccXangle
#	    roll = CF_GAIN * (roll + rate_gyr_y * LP) + (1 - CF_GAIN) * AccYangle

            pitch = EY
	    roll = EX
            gPitch = GY
            gRoll = GX

	    #time.sleep(0.3)
	    #b = time.clock()
	    #c = b - a

	    print "x = %0.2f	gx = %0.2f	y = %0.2f	gy = %0.2f" %(pitch, gPitch, roll, gRoll)

            ###################################################################################
            #                               ADJUST ACCORDINGLY                                #
            ###################################################################################
            
            #------------------------------STABILIZE------------------------------------------
            #throttle up??
            
            #thr = 20
            #---------------------------- Stabilizer PID -------------------------------------
            #pitch_stab_output = PID_PITCH_STAB.update_pid(rcpit*0.2,pitch,0.0180)
            #roll_stab_output = PID_ROLL_STAB.update_pid(rcroll*0.2,roll,1.0)
            
            #---------------- Testing controller PID's ---------------------------------------
            pitch_output = PID_PITCH_RATE.update_pid(rcpit*pitchRateScaler,gPitch*gyroPitchScaler,1.0)
            roll_output = PID_ROLL_RATE.update_pid(rcroll*rollRateScaler,gRoll*gyroRollScaler,1.0)
	    #yaw_output = PID_YAW_RATE.update_pid(rcyaw*yawRateScaler,gYaw*gyroYawScaler,1.0)
                                  
            #pitch_output = PID_PITCH_STAB.update_pid(rcpit*pitchRateScaler,pitch*gyroPitchScaler,1.0)
            #roll_output = PID_ROLL_STAB.update_pid(rcroll*rollRateScaler,roll*gyroRollScaler,1.0)
	    #yaw_output = PID_YAW_STAB.update_pid(rcyaw*yawRateScaler,gyz*gyroYawScaler,1.0)

            #---------------------------- Rate PID's ----------------------------------------
            #pitch_output = PID_PITCH_RATE.update_pid(pitch_stab_output,gPitch*gyroPitchScaler,0.0180)
            #roll_output = PID_ROLL_RATE.update_pid(roll_stab_output,gRoll*gyroRollScaler,1.0)
            
	    #print "pitch = %0.2f   roll = %0.2f   yaw = %0.2f" %(CFangleX , roll, yaw)
            
            
            #overriding for testing
	    #print "pitch_stab = %f     roll_stab = %f  "%(pitch_stab_output,roll_stab_output)
            #print "pitch stab = %0.2f	pitch = %0.2f	pitch ouput = %0.2f	gyropitch = %0.2f"%(pitch_stab_output, pitch, pitch_output, gpx*gyroPitchScaler)
            print "	   roll_output = %0.2f 		   gyroroll = %0.2f" %(roll_output, gRoll)
            #print "PID pitch = %f  PID roll = %f"%(PID_PITCH_STAB.update_pid(rcpit,pitch,1),PID_ROLL_STAB.update_pid(rcroll,roll,1))
            #print "gyropitch = %f  gyroroll = %f gyroyaw = %f"%(gpx,gyroroll,gy)

            #print "gyroroll = %0.2f	roll output = %0.2f" %(gry*rollRateScaler, roll_output)
            #print "gyroyaw = %0.2f	yaw output = %0.2f" %(gyz*yawRateScaler, yaw_output)
	    #print "gyropitch = %0.2f	pitch output = %0.2f"%(gpx*pitchRateScaler, pitch_output)
           
            #print "                                        roll output = %f"%(roll_output)
            pitch_output = 0.0
            #roll_output = 0.0
            yaw_output = 0.0
            #print "pid p = %f    pid r = %f    pid y = %f"%(pitch_output,roll_output,yaw_output)
            if inverted:    
                #MOTOR FL
                MFL = pwm.setP(MOTOR_FL,constrain(thr - roll_output - pitch_output + yaw_output,0.0,90.0))
                #MOTOR BL
                MBL = pwm.setP(MOTOR_BL,constrain(thr - roll_output + pitch_output - yaw_output,0.0,90.0))
                #MOTOR FR
                MFR = pwm.setP(MOTOR_FR,constrain(thr + roll_output - pitch_output - yaw_output,0.0,90.0))
                #MOTOR BR
                MBR = pwm.setP(MOTOR_BR,constrain(thr + roll_output + pitch_output + yaw_output,0.0,90.0))
            #print " MFL = %f  MBL = %f   MFR = %f   MBR = %f  "%(MFL,MBL,MFR,MBR)

	    print "						time = %0.4f" %(time.clock() - a)

    except (KeyboardInterrupt,SystemExit):
        #Turn motors off
        pwm.setP("1",3)
        pwm.setP("2",3)
        pwm.setP("3",3)
        pwm.setP("4",3)            
        activeThreads = False
        print "Stopping Quaddie...."                  
        #readings.join()
        tabletreadings.join()
        #pingreadings.join()
        print "Threads Exit succesfully"
        sys.exit()   
