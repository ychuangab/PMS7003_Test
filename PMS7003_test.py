#!/usr/bin/env python
 #encoding=utf-8
import RPi.GPIO as GPIO
import serial, time
import struct
import sys
import smbus2
sys.modules['smbus'] = smbus2
 
from RPLCD.i2c import CharLCD
lcd = CharLCD('PCF8574', address=0x27, port=1, backlight_enabled=True)
#from struct import *

# Number of points for average
read_times_per_test = 16

# pin number for Rx on raspi
pin = 10

# For LCD 1602 display
def lcd_display(PM25, PM10):
    lcd.cursor_pos = (0, 0)
    lcd.write_string("PM25: {} ug/m3".format(PM25))
    lcd.cursor_pos = (1, 0)
    lcd.write_string("PM10: {} ug/m3".format(PM10))
    time.sleep(0.5)

# Read data from PMS7003 to raspi
def read():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.HIGH)

    # open serial 
    #ser = serial.Serial(port="/dev/ttyS0", baudrate=9600)  #method 1
    ser = serial.Serial()
    ser.baudrate = 9600
    ser.port = "/dev/ttyS0"
    ser.open()
    #print(ser.isOpen())

    time.sleep(1)

    cnt = 0
    pm25_list = []
    pm10_list = []
    while True:  
        # number of bits waitted to be transmitted
        count = ser.inWaiting()
        
        if count >= 24:  
            # core of read
            recv = ser.read(count)
            print(count)
            ser.flushInput()
            #print(ser.inWaiting())  #should be zero after flushinput
            
            # Get pm2.5 and pm10 data part
            tmp = recv[4:16] #4~15 , data1~data6; 4~9 CF=1 std particle ; 10~15 atmosphere
            
            #print(recv)  #'BM\x00\x1c\x00\x05\x00\t\x00\t\x00\x05\x00\t\x00\t\x03\xba\x01\x1a\x00D\x00\x03\x00\x00\x00\x00\x98\x00\x02\x90'
            print(tmp)   #'\x00\x05\x00\t\x00\t\x00\x05\x00\t\x00\t'
            
            details = struct.unpack('>hhhhhh', tmp) #h is short : -32768 ~ 32768
            print(details)
            
            print("PM 2.5: %d, PM 10: %d" % (details[1],details[2]))
            pm25_list.append(details[1])
            pm10_list.append(details[2])
            
            lcd_display(PM25=details[1], PM10=details[2])
            
            # b'BM\x00\x1c\x00\x08\x00\x13\x00\x13\x00\x08'
            # (16973, 28, 8, 19, 19, 8)
            # PM 2.5: 28, PM 10: 8

            cnt = cnt + 1
            if cnt >= read_times_per_test:
                break
        # for delay  
        time.sleep(0.1)

    GPIO.output(pin, GPIO.LOW)
    GPIO.cleanup()
    
    #print(sorted(pm25_list)[2:-2])
    
    pm25_avg = sum(sorted(pm25_list)[2:-2]) / (len(pm25_list) - 4)
    pm10_avg = sum(sorted(pm10_list)[2:-2]) / (len(pm10_list) - 4)
    print("Average PM 2.5: %d, PM 10: %d\n (Tested %d times)" % (pm25_avg,pm10_avg, read_times_per_test))
    
    return pm25_avg, pm10_avg

if __name__ == '__main__':  
    try:
        print('按下 Ctrl-C 可停止程式')
        lcd.clear()
        while True:
            read()
            lcd.cursor_pos = (0, 0)
        
    except KeyboardInterrupt: 
        if ser != None: 
            ser.close() 
        GPIO.cleanup()
    finally:
        lcd.clear()
        
