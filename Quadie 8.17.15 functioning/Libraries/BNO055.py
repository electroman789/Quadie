#!/usr/bin/python

##############################################################################################
#   author: Chancey Kelley
#   Code designed for the Engineering Design 2 Project of Florida Atlantic University 
#   term: Winter 2015
#   Date: 6-25-2015
#############################################################################################
from Adafruit_I2C import Adafruit_I2C
from time import sleep


class BNO055:

   """
   	Page id register definition
   """
   BNO055_PAGE_ID_ADDR		= 0X07
   BNO055_PAGE_0 		= 0x00
   BNO055_PAGE_1 		= 0x01
   BNO055_ID			= 0xA0
   
   """
	PAGE0 REGISTER DEFINITION START
   """
   BNO055_CHIP_ID_ADDR		= 0x00
   BNO055_ACCEL_REV_ID_ADDR	= 0x01
   BNO055_MAG_REV_ID_ADDR	= 0x02
   BNO055_GYRO_REV_ID_ADDR	= 0x03
   BNO055_SW_REV_ID_LSB_ADDR	= 0x04
   BNO055_SW_REV_ID_MSB_ADDR	= 0x05
   BNO055_BL_REV_ID_ADDR	= 0x06
   BNO055_DEFAULT_ADDRESS	= 0x28
   BNO055_ALT_ADDRESS		= 0x29

   """
	Accel data register
   """
   BNO055_ACCEL_DATA_X_LSB_ADDR	= 0x08
   BNO055_ACCEL_DATA_X_MSB_ADDR	= 0x09
   BNO055_ACCEL_DATA_Y_LSB_ADDR	= 0x0A
   BNO055_ACCEL_DATA_Y_MSB_ADDR	= 0x0B
   BNO055_ACCEL_DATA_Z_LSB_ADDR	= 0x0C
   BNO055_ACCEL_DATA_Z_MSB_ADDR	= 0x0D

   """
	Mag data register
   """
   BNO055_MAG_DATA_X_LSB_ADDR	= 0X0E
   BNO055_MAG_DATA_X_MSB_ADDR	= 0X0F
   BNO055_MAG_DATA_Y_LSB_ADDR	= 0X10
   BNO055_MAG_DATA_Y_MSB_ADDR	= 0X11
   BNO055_MAG_DATA_Z_LSB_ADDR	= 0X12
   BNO055_MAG_DATA_Z_MSB_ADDR	= 0X13

   """
	Gyro data registers
   """
   BNO055_GYRO_DATA_X_LSB_ADDR	= 0X14
   BNO055_GYRO_DATA_X_MSB_ADDR	= 0X15
   BNO055_GYRO_DATA_Y_LSB_ADDR	= 0X16
   BNO055_GYRO_DATA_Y_MSB_ADDR	= 0X17
   BNO055_GYRO_DATA_Z_LSB_ADDR	= 0X18
   BNO055_GYRO_DATA_Z_MSB_ADDR	= 0X19

   """
	Euler data registers
   """
   BNO055_EULER_X_LSB_ADDR	= 0X1E
   BNO055_EULER_X_MSB_ADDR	= 0X1F
   BNO055_EULER_Y_LSB_ADDR	= 0X1C
   BNO055_EULER_Y_MSB_ADDR	= 0X1D
   BNO055_EULER_Z_LSB_ADDR	= 0X1A
   BNO055_EULER_Z_MSB_ADDR	= 0X1B

   """
	Qaternion data registers
   """
   BNO055_QUATERNION_DATA_W_LSB_ADDR	= 0X20
   BNO055_QUATERNION_DATA_W_MSB_ADDR	= 0X21
   BNO055_QUATERNION_DATA_X_LSB_ADDR	= 0X22
   BNO055_QUATERNION_DATA_X_MSB_ADDR	= 0X23
   BNO055_QUATERNION_DATA_Y_LSB_ADDR	= 0X24
   BNO055_QUATERNION_DATA_Y_MSB_ADDR	= 0X25
   BNO055_QUATERNION_DATA_Z_LSB_ADDR	= 0X26
   BNO055_QUATERNION_DATA_Z_MSB_ADDR	= 0X27

   """
	Linear acceleration data registers
   """
   BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR	= 0X28
   BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR	= 0X29
   BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR	= 0X2A
   BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR	= 0X2B
   BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR	= 0X2C
   BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR	= 0X2D

   """
	Gravity data registers
   """
   BNO055_GRAVITY_DATA_X_LSB_ADDR	= 0X2E
   BNO055_GRAVITY_DATA_X_MSB_ADDR	= 0X2F
   BNO055_GRAVITY_DATA_Y_LSB_ADDR	= 0X30
   BNO055_GRAVITY_DATA_Y_MSB_ADDR	= 0X31
   BNO055_GRAVITY_DATA_Z_LSB_ADDR	= 0X32
   BNO055_GRAVITY_DATA_Z_MSB_ADDR	= 0X33

   """
	Temperature data register
   """
   BNO055_TEMP_ADDR		= 0X34
  
   """
	Status registers
   """
   BNO055_CALIB_STAT_ADDR	= 0X35
   BNO055_SELFTEST_RESULT_ADDR	= 0X36
   BNO055_INTR_STAT_ADDR	= 0X37
   BNO055_SYS_CLK_STAT_ADDR	= 0X38
   BNO055_SYS_STAT_ADDR	= 0X39
   BNO055_SYS_ERR_ADDR	= 0X3A

   """
	Unit selection register
   """
   BNO055_UNIT_SEL_ADDR 	= 0X3B
   BNO055_DATA_SELECT_ADDR 	= 0X3C

   """
	Mode registers
   """
   BNO055_OPR_MODE_ADDR		= 0X3D
   BNO055_PWR_MODE_ADDR		= 0X3E
   BNO055_SYS_TRIGGER_ADDR	= 0X3F
   BNO055_TEMP_SOURCE_ADDR	= 0X40

   """
	System Trigger register
	***Use bitwise and (&) for clock setting
   """
   BNO055_SELF_TEST		= 0x01
   BNO055_CHIP_RESET		= 0x20
   BNO055_RESET_INT		= 0x40
   BNO055_CLOCK_EXTERNAL	= 0x80

   """
	Axis remap registers
   """
   BNO055_AXIS_MAP_CONFIG_ADDR	= 0X41
   BNO055_AXIS_MAP_SIGN_ADDR	= 0X42

   """
	SIC registers
   """
   BNO055_SIC_MATRIX_0_LSB_ADDR	= 0X43
   BNO055_SIC_MATRIX_0_MSB_ADDR	= 0X44
   BNO055_SIC_MATRIX_1_LSB_ADDR	= 0X45
   BNO055_SIC_MATRIX_1_MSB_ADDR	= 0X46
   BNO055_SIC_MATRIX_2_LSB_ADDR	= 0X47
   BNO055_SIC_MATRIX_2_MSB_ADDR	= 0X48
   BNO055_SIC_MATRIX_3_LSB_ADDR	= 0X49
   BNO055_SIC_MATRIX_3_MSB_ADDR	= 0X4A
   BNO055_SIC_MATRIX_4_LSB_ADDR	= 0X4B
   BNO055_SIC_MATRIX_4_MSB_ADDR	= 0X4C
   BNO055_SIC_MATRIX_5_LSB_ADDR	= 0X4D
   BNO055_SIC_MATRIX_5_MSB_ADDR	= 0X4E
   BNO055_SIC_MATRIX_6_LSB_ADDR	= 0X4F
   BNO055_SIC_MATRIX_6_MSB_ADDR	= 0X50
   BNO055_SIC_MATRIX_7_LSB_ADDR	= 0X51
   BNO055_SIC_MATRIX_7_MSB_ADDR	= 0X52
   BNO055_SIC_MATRIX_8_LSB_ADDR	= 0X53
   BNO055_SIC_MATRIX_8_MSB_ADDR	= 0X54

   """
	Accelerometer Offset registers
   """
   ACCEL_OFFSET_X_LSB_ADDR	= 0X55
   ACCEL_OFFSET_X_MSB_ADDR	= 0X56
   ACCEL_OFFSET_Y_LSB_ADDR	= 0X57
   ACCEL_OFFSET_Y_MSB_ADDR	= 0X58
   ACCEL_OFFSET_Z_LSB_ADDR	= 0X59
   ACCEL_OFFSET_Z_MSB_ADDR	= 0X5A

   """
	Magnetometer Offset registers
   """
   MAG_OFFSET_X_LSB_ADDR	= 0X5B
   MAG_OFFSET_X_MSB_ADDR	= 0X5C
   MAG_OFFSET_Y_LSB_ADDR	= 0X5D
   MAG_OFFSET_Y_MSB_ADDR	= 0X5E
   MAG_OFFSET_Z_LSB_ADDR	= 0X5F
   MAG_OFFSET_Z_MSB_ADDR	= 0X60

   """
	Gyroscope Offset registers
   """
   GYRO_OFFSET_X_LSB_ADDR	= 0X61
   GYRO_OFFSET_X_MSB_ADDR	= 0X62
   GYRO_OFFSET_Y_LSB_ADDR	= 0X63
   GYRO_OFFSET_Y_MSB_ADDR	= 0X64
   GYRO_OFFSET_Z_LSB_ADDR	= 0X65
   GYRO_OFFSET_Z_MSB_ADDR	= 0X66

   """
	Radius registers
   """
   ACCEL_RADIUS_LSB_ADDR	= 0X67
   ACCEL_RADIUS_MSB_ADDR	= 0X68
   MAG_RADIUS_LSB_ADDR		= 0X69
   MAG_RADIUS_MSB_ADDR		= 0X6A

   """
	Power Mode
   """
   POWER_MODE_NORMAL 		= 0X00
   POWER_MODE_LOWPOWER		= 0X01
   POWER_MODE_SUSPEND		= 0X02

   """
	Operation mode settings
   """
   OPERATION_MODE_CONFIG	= 0X00
   OPERATION_MODE_ACCONLY	= 0X01
   OPERATION_MODE_MAGONLY	= 0X02
   OPERATION_MODE_GYRONLY	= 0X03
   OPERATION_MODE_ACCMAG	= 0X04
   OPERATION_MODE_ACCGYRO	= 0X05
   OPERATION_MODE_MAGGYRO	= 0X06
   OPERATION_MODE_AMG		= 0X07
   OPERATION_MODE_IMUPLUS	= 0X08
   OPERATION_MODE_COMPASS	= 0X09
   OPERATION_MODE_M4G		= 0X0A
   OPERATION_MODE_NDOF_FMC_OFF	= 0X0B
   OPERATION_MODE_NDOF		= 0X0C

    ####################################################################################
    #										       #
    #					Constructor				       #
    #										       #
    ####################################################################################

   def __init__(self):
      self.bno055 = Adafruit_I2C(self.BNO055_DEFAULT_ADDRESS,1)				#Initialize I2C
      id = self.bno055.readU8(self.BNO055_CHIP_ID_ADDR)					#Test chip status
      print "id = %x" %(id)
      while id != self.BNO055_ID:							#Test till chip responds
         sleep(0.5)
         id = self.bno055.readU8(self.BNO055_CHIP_ID_ADDR)
      cal_again = raw_input("calibrate again? >>> ")					#Ask to calibrate
      status = self.bno055.readU8(self.BNO055_SELFTEST_RESULT_ADDR)			#Check self-test result
      print "status = %x" %(status)
      self.setMode(self.OPERATION_MODE_CONFIG)						#Enter config mode
      sleep(0.1)
      self.bno055.write8(self.BNO055_PAGE_ID_ADDR, self.BNO055_PAGE_0)			#Set config page to 0x00
      sleep(0.01)
      if cal_again == "yes":
         self.bno055.write8(self.BNO055_SYS_TRIGGER_ADDR, self.BNO055_CHIP_RESET)	#Chip reset
         sleep(1)
         id = self.bno055.readU8(self.BNO055_CHIP_ID_ADDR)				#Check chip is on
         print "id2 = %x" %(id)
         while id != self.BNO055_ID:							#Test til chip responds
            sleep(0.1)
            id = self.bno055.readU8(self.BNO055_CHIP_ID_ADDR)
      sleep(0.05)
      self.bno055.write8(self.BNO055_PWR_MODE_ADDR, self.POWER_MODE_NORMAL)		#Set power mode to normal
      sleep(0.01)
      unit = self.bno055.readU8(self.BNO055_UNIT_SEL_ADDR)				#Check unit selections
      unit2 = unit & 0xF9								#Set bit 1 and 2 to False
      print "mask = %x" %(unit2)
      self.bno055.write8(self.BNO055_UNIT_SEL_ADDR, unit2)				#Set units
      sleep(0.01)
      self.bno055.write8(self.BNO055_SYS_TRIGGER_ADDR, self.BNO055_CLOCK_EXTERNAL)	#Set clock to external
      sleep(0.01)
      self.setMode(self.OPERATION_MODE_NDOF)						#Return to NDOF fusion mode
      sleep(0.02)
      cal = self.bno055.readU8(self.BNO055_CALIB_STAT_ADDR)				#Read calibration data
      print "calibration status = %x" %(cal)
      if cal_again == "yes":								#Calibrate again
         cal = self.bno055.readU8(self.BNO055_CALIB_STAT_ADDR)				#Read calibration status
         while cal != 0xFF:								#0xFF = fully calibrated
            print "calibration status = %x" %(cal)
            sleep(1.0)
            cal = self.bno055.readU8(self.BNO055_CALIB_STAT_ADDR)			#Read calibration status

   def setMode(self, mode):
      self.bno055.write8(self.BNO055_OPR_MODE_ADDR, mode)
      sleep(0.03)

   def readEuler (self):
      ex = (self.bno055.readS8(self.BNO055_EULER_X_MSB_ADDR) << 8) | self.bno055.readU8(self.BNO055_EULER_X_LSB_ADDR)
      ey = (self.bno055.readS8(self.BNO055_EULER_Y_MSB_ADDR) << 8) | self.bno055.readU8(self.BNO055_EULER_Y_LSB_ADDR)
      ez = (self.bno055.readS8(self.BNO055_EULER_Z_MSB_ADDR) << 8) | self.bno055.readU8(self.BNO055_EULER_Z_LSB_ADDR)
      return ex, ey, ez

   def getGyro(self):
      gx = (self.bno055.readS8(self.BNO055_GYRO_DATA_X_MSB_ADDR) << 8) | self.bno055.readU8(self.BNO055_GYRO_DATA_X_LSB_ADDR)
      gy = (self.bno055.readS8(self.BNO055_GYRO_DATA_Y_MSB_ADDR) << 8) | self.bno055.readU8(self.BNO055_GYRO_DATA_Y_LSB_ADDR)
      gz = (self.bno055.readS8(self.BNO055_GYRO_DATA_Z_MSB_ADDR) << 8) | self.bno055.readU8(self.BNO055_GYRO_DATA_Z_LSB_ADDR)
      return gx, gy, gz


   def checkSysStatus(self):
      status = self.bno055.readU8(self.BNO055_SYS_STAT_ADDR) 
      print "system status = %x" %(status)
      if status == 0x01:
         status2 = self.bno055.readU8(self.BNO055_SYS_STAT_ADDR)
         print "system error = %x" %(status2)
