#!/usr/bin/python

from FreeIMU import FreeIMU

"""
calibrate.py - Guides user through FreeIMU accelerometer and magnetometer calibration

Copyright (C) 2012 Fabio Varesano <fvaresano@yahoo.it>

Development of this code has been supported by the Department of Computer Science,
Universita' degli Studi di Torino, Italy within the Piemonte Project
http://www.piemonte.di.unito.it/


This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""

count = 30
freeIMU = FreeIMU()
buff = [0.0 for i in range(11)]
acc_file = 'acc.txt'
magn_file = 'magn.txt'
tot_readings = 0
filesopened = False
a_f = open(acc_file,'r+') 
m_f = open(magn_file,'r+')
print "\n\nWelcome to the library calibration routine!Original from Fabio Varesano 2012\n Translated to python by Christian Hinostroza"

print "loading the sensor"
print "..."

accgyro = freeIMU.accgyro
magn = freeIMU.magn
print "sensors loaded"

try:
    a_f = open(acc_file,'r+') 
    m_f = open(magn_file,'r+')
    print "files open\n"
    filesopened = True
    while True:
        if filesopened:
            for j in range(count):
                buff[0],buff[1],buff[2],buff[3],buff[4],buff[5],buff[6],buff[7],buff[8],buff[9],buff[10] = freeIMU.getRawValues()             #log accelerometer values
            readings_line = "%f %f %f\n" % (buff[0], buff[1], buff[2])
            a_f.write(readings_line)

            #log magnetometer values
            readings_line = "%f %f %f\n" % (buff[6], buff[7], buff[8])
            m_f.write(readings_line)

            tot_readings = tot_readings + 1
            if(tot_readings % 200 == 0):
                print "%d readings logged. Hit CTRL + C to interrupt." % (tot_readings)
        else:
            print "file hasn't been opened\n"
except KeyboardInterrupt:
    a_f.close()
    m_f.close()
    print "\n%d values logged to %s and %s" % (tot_readings, acc_file, magn_file)
    filesopened = False




                







