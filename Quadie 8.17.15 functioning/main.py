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

MOTOR_FL = "1"
MOTOR_FR = "3"
MOTOR_BL = "2"
MOTOR_BR = "4"

rcpit = 0.0
rcroll = 0.0
command = ""
yaw = 0.0
pitch = 0.0
roll = 0.0
thr = 0.0
controller = Controller.Controller()
sensors = FreeIMU.FreeIMU()
pwm = pwm.PWMMotor()
#ping = PING.PingSensor()
#####################################################################################
#                                       PID setups                                  #
#####################################################################################
PID_PITCH_RATE = pid.PID()
PID_ROLL_RATE = pid.PID()
PID_PITCH_STAB = pid.PID()
PID_ROLL_STAB = pid.PID()
PID_YAW_RATE = pid.PID()
#PID_YAW_STAB = pid.PID()

#__________________ SLOW VALUES______________________________
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#----- Good control 4/28/2015 --------------------------------
#PID_PITCH_RATE.definePID(0.9,0.002,0.001)
#PID_ROLL_RATE.definePID(1.2,0.002,0.001)
#-------------------------------------------------------------

###############################################################
#                          RATE PIDS
###############################################################
PID_PITCH_RATE.definePID(1.5,0.00,0.00)
PID_ROLL_RATE.definePID(1.2,0.002,0.001)
PID_YAW_RATE.definePID(2.0,0.000,0.00)

pitchRateScaler = 20.0 / (0.9 * 100.0)
rollRateScaler = 20.0 / (3.5 * 100.0)
gyroRollScaler = 0.0166

PID_PITCH_RATE.set_windup_bounds(-6,6)
PID_ROLL_RATE.set_windup_bounds(-6,6)
#----------------------------------------------------------------------
#           Notes:
#           1) P minimum value is 0.0000005
#           2) P maximum value is 0.000005
#           3) I minimum value is 0.00000001
#-----------------------------------------------------------------------
##################################################################
#                       STAB PID    
#PID_PITCH_STAB.definePID(0.080,0.0008,0.1)
#PID_ROLL_STAB.definePID(0.085,0.00008,0.1)
##################################################################
PID_PITCH_STAB.definePID(0.080,0.0008,0.1)
PID_ROLL_STAB.definePID(0.085,0.00008,0.1)



PID_PITCH_STAB.set_windup_bounds(-20,20)
PID_ROLL_STAB.set_windup_bounds(-20,20)
logging.basicConfig(level=logging.DEBUG,format='(%(threadName)-9s) %(message)s',)
#####################################################################################
#                                   Q U E U E ' S                                   #
#####################################################################################
S_BUF_SIZE = 2
BUF_SIZE = 10
yq = Queue.Queue(S_BUF_SIZE)
pq = Queue.Queue(S_BUF_SIZE)
rq = Queue.Queue(S_BUF_SIZE)
dq = Queue.Queue(BUF_SIZE)
grq = Queue.Queue(S_BUF_SIZE)
gpq = Queue.Queue(S_BUF_SIZE)
aq = Queue.Queue(S_BUF_SIZE)
baq = Queue.Queue(S_BUF_SIZE)

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
    if amt < low:
        return low
    elif amt > high:
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
        while True and activeThreads:
            y,r,p = sensors.getYawPitchRoll()
            print "y = %f p = %f r = %f"%(y,p,r)
            #b1,b2,b3,gp,gr,gy,pn,rn,dn = sensors.getValues()
            #print "gyroscope x = %f  y = %f   z = %f"%(gy,gp,gr)
            #baro = sensors.getBaroAlt()
            if not yq.full():
                yq.put(y)
                #logging.debug('Putting yaw ' + str(y) + ' : ' + str(yq.qsize()) + 'readings in queue')
            if not pq.full():
                pq.put(p)
                #logging.debug('Putting pitch ' + str(p) + ' : ' + str(pq.qsize()) + 'readings in queue')
            if not rq.full():
                rq.put(r)
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
    readings = SensorReading(name='freeimu')
    tabletreadings = TabletThread(name='Android')
    #pingreadings = PingReading(name='PING')
    #printreads = PrintReading(name='printvalues')
    readings.daemon = True
    tabletreadings.daemon = True
    #pingreadings.daemon = True
    readings.start()
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
            if timer < 0 :
                thr -= 0.0001
            ##################################################################################
            #                               GET COMMANDS                                     #
            ##################################################################################
            if not dq.empty():
                command = dq.get()
                if ',' in command:
                    rcpit,rcroll = controller.getPitchRoll(command)
                    rcroll = -rcroll
                    #print "rcpit = %f  rcroll = %f"%(rcpit,rcroll)
                else:
                    rcpit = 0.0
                    rcroll = 0.0
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
                timer -= 1
            #print "rcpit = %f  rcroll = %f"%(rcpit,rcroll)
            print "timer = %d"%timer
            ##################################################################################
            #                               GET POSITIONING                                  #
            ##################################################################################
            if not yq.empty():
                yaw = yq.get()
            if not pq.empty():
                pitch = pq.get()
            if not rq.empty():
                roll = -rq.get()
            b1,b2,b3,gyropitch,gyroroll,gy,pn,rn,dn = sensors.getValues()
            #if not aq.empty():
            #    alt = aq.get()
            #if not baq.empty():
            #    b_alt = baq.get()
                #debugging
                #logging.debug('Getting yaw value  ' + str(item) + ' : ' + str(yq.qsize()) + 'readings in queue')
            #print "yaw = %f o  pitch = %f o  roll = %f o"%(yaw,pitch,roll)
            #print "Altitude = %f  "%alt
            ###################################################################################
            #                               ADJUST ACCORDINGLY                                #
            ###################################################################################
            
            #------------------------------STABILIZE------------------------------------------
            #throttle up??
            
            #thr = 20
            #---------------------------- Stabilizer PID -------------------------------------
            pitch_stab_output = PID_PITCH_STAB.update_pid(rcpit*0.2,pitch,1.0)
            roll_stab_output = PID_ROLL_STAB.update_pid(rcroll*0.2,roll,1.0)
            
            #---------------- Testing controller PID's ---------------------------------------
            pitch_output = PID_PITCH_RATE.update_pid(rcpit*0.2,gyropitch*0.02,1.0)
            #roll_output = PID_ROLL_RATE.update_pid(rcroll*rollRateScaler,gyroroll*gyroRollScaler,1.0)
            
            #yaw_output = constrain(PID_YAW_RATE.update_pid(0.0,yaw,1.0),-180.0,180.0)
            
            #print " pitch_stab = %f     roll_stab = %f  "%(pitch_stab_output,roll_stab_output)
            #---------------------------- Rate PID's ----------------------------------------
            #pitch_output = PID_PITCH_RATE.update_pid(pitch_stab_output,gyropitch*0.02,1.0)
            #roll_output = PID_ROLL_RATE.update_pid(roll_stab_output,gyroroll*gyroRollScaler,1.0)
            #print "pitch = %0.2f   roll = %0.2f   yaw = %0.2f" %(pitch, roll, yaw)
            
            
            #overriding yaw for testing
            #print "pitch_stab = %f  pitch = %f "%(pitch_stab_output,pitch)
            #print "roll_output = %0.2f gyroroll = %0.2f" %(roll_output, gyroroll*0.02)
            #print "PID pitch = %f  PID roll = %f"%(PID_PITCH_STAB.update_pid(rcpit,pitch,1),PID_ROLL_STAB.update_pid(rcroll,roll,1))
            #print "gyropitch = %f  gyroroll = %f gyroyaw = %f"%(gyropitch,gyroroll,gy)
            #print "Pitches = %f"%PID_PITCH_RATE.update_pid(rcpit,gyropitch,1.0)
            #print "gyroroll = %0.2f  roll output = %0.2f" %(gyroroll, roll_output)
            #print "gyropitch = %f  pitch output = %f"%(gyropitch,pitch_output)
            #print "                                        roll output = %f"%(roll_output)
            #pitch_output = 0.0
            roll_output = 0.0
            yaw_output = 0.0
            #print "pid p = %f    pid r = %f    pid y = %f"%(pitch_output,roll_output,yaw_output)
            if inverted:    
                #MOTOR FL
                MFL = pwm.setP(MOTOR_FL,constrain(thr - roll_output - pitch_output - yaw_output,0.0,90.0))
                #MOTOR BL
                MBL = pwm.setP(MOTOR_BL,constrain(thr - roll_output + pitch_output + yaw_output,0.0,90.0))
                #MOTOR FR
                MFR = pwm.setP(MOTOR_FR,constrain(thr + roll_output - pitch_output + yaw_output,0.0,90.0))
                #MOTOR BR
                MBR = pwm.setP(MOTOR_BR,constrain(thr + roll_output + pitch_output - yaw_output,0.0,90.0))
            #print " MFL = %f  MBL = %f   MFR = %f   MBR = %f  "%(MFL,MBL,MFR,MBR)
    except (KeyboardInterrupt,SystemExit):
        #Turn motors off
        pwm.setP("1",3)
        pwm.setP("2",3)
        pwm.setP("3",3)
        pwm.setP("4",3)            
        activeThreads = False
        print "Stopping Quaddie...."                  
        readings.join()
        tabletreadings.join()
        #pingreadings.join()
        print "Threads Exit succesfully"
        sys.exit()   
