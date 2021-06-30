#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3,Vector3Stamped
from ardrone_autonomy.msg import Navdata
from std_msgs.msg import Int32, Float32,String

from simple_pid import PID
import math
import numpy as np
from sympy import *
from matplotlib import pyplot as plt
from numpy.random import *
import pandas as pd
x_list=magY_list=altd_list=lat_list=lon_list=phase=0

import time
from threading import Thread
a=b=x=y=start=pitch=roll=thrust=yaw=0
lstick_fb=lstick_lr=rstick_fb=rstick_lr=plusbotton_lr=plusbotton_fb=0
lat=lon=altd=battery=magX=magY=magZ=0
goal_altd=goal_magY=goal_lat=goal_lon=0

class controll(object):
    def __init__(self):
        self.pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
        self.pub_land = rospy.Publisher('/ardrone/land', Empty)
        self.pub_reset = rospy.Publisher('/ardrone/reset', Empty)
        self.pub_velocity = rospy.Publisher('/cmd_vel', Twist,queue_size=1)

	self.sub_joy = rospy.Subscriber('joy',Joy,self.callback_joy, queue_size=1)
        self.sub_serial = rospy.Subscriber('Serial_in',String,self.callback_serial)
        self.sub_mag = rospy.Subscriber("/ardrone/mag",Vector3Stamped,self.callback_mag)
        self.sub_altd = rospy.Subscriber("/ardrone/navdata",Navdata,self.callback_altd)

    def callback_joy(self, joy_msg):
        global a,b,x,y,lstick_lr,lstick_fb,rstick_lr,rstick_fb,start,plusbotton_lr,plusbotton_fb,roll,pitch,yaw,thrust
        a = float(joy_msg.buttons[1])
        b = float(joy_msg.buttons[2])
        x = float(joy_msg.buttons[0])
        y = float(joy_msg.buttons[3])
	lb = float(joy_msg.buttons[4])
	rb = float(joy_msg.buttons[5])
	lt = float(joy_msg.buttons[6])
	rt = float(joy_msg.buttons[7])
        start = float(joy_msg.buttons[9])
        select = float(joy_msg.buttons[8])
	lstick_botton = float(joy_msg.buttons[10])
	rstick_botton = float(joy_msg.buttons[11])


        lstick_lr = float(joy_msg.axes[0]) #left_and_right//roll
        lstick_fb = float(joy_msg.axes[1]) #front_and_back//pitch
        rstick_lr = float(joy_msg.axes[2]) #rotation//yaw
        rstick_fb = float(joy_msg.axes[3]) #up_and_down//thrust
        plusbotton_lr = float(joy_msg.axes[4])
        plusbotton_fb = float(joy_msg.axes[5]) 

	pitch = lstick_fb
	roll = lstick_lr
	thrust = rstick_fb
	yaw = rstick_lr	

    def callback_serial(self,serial_data):
        global data,lat,lon
        data = serial_data.data

        #35.7096748,139.5229340,268
	lat = float(int(data[0])*10+int(data[1])*1+int(data[3])*0.1+int(data[4])*0.01+int(data[5])*0.001+int(data[6])*0.0001+int(data[7])*0.00001+int(data[8])*0.000001)
        lon = float(int(data[10])*100+int(data[11])*10+int(data[12])*1+int(data[14])*0.1+int(data[15])*0.01+int(data[16])*0.001+int(data[17])*0.0001+int(data[18])*0.00001+(int(data[19]))*0.000001)
	#print(lat)
        #print(lon)

    def callback_mag(self,mag_msg):
        global magX,magY,magZ
        magX = mag_msg.vector.x
        magY = mag_msg.vector.y
        magZ = mag_msg.vector.z
        #print(mag)

    def callback_altd(self,Navdata_msg):
        global altd,battery
        altd = Navdata_msg.altd
        battery = Navdata_msg.batteryPercent
        #print(altd,battery)

    #def excel(self):
        #df = pd.DataFrame([x_list,magY_list,altd_list,lat_list, lon_list,phase])
        #df.columns = ['time','magY', 'altd', 'lat' ,'lon', 'phase']
        #df = pd.DataFrame([x_list,magY_list,altd_list,phase])
        #df.columns = ['time','magY', 'altd', 'phase']
        #df.index = ['row0', 'row1', 'row2', 'row3', 'row4']
        #df.to_excel('~/desktop/output.xlsx')
#PID 
    def magY(self):
        global goal_magY
        goal_magY = 0.5
        magYController = PID(1.0,0,0, sample_time=0.1, setpoint=goal_magY,output_limits=(-1.0,1.0))
        n=0
        x_list = []
        magY_list = []
        x_list.append(0)
        magY_list.append(0.00)

        while True:
            yaw = magYController(magY)
            n +=1
            #if n==20:
                #n=0
            print(n*0.05,magY,yaw,goal_magY-magY)
            time.sleep(0.05)
            self.pub_velocity.publish(Twist(Vector3(0,0,0),Vector3(0,0,yaw)))
            magY_list.append(magY)
            x_list.append(n*0.05)
            if start== 1:
                plt.hlines([goal_magY], 0, n*0.05, "r", linestyles='dashed')  
                plt.plot(x_list, magY_list, color="r")
                plt.plot(x_list, magY_list)
                plt.ylim(-1.0,1.0)
                plt.xlim(0,20)
                plt.xlabel('time, s')
                plt.ylabel('mag,  ')
                break

    def altd(self):
        global goal_altd
        goal_altd = 2000
        altdController = PID(2.0,0,0, sample_time=0.1, setpoint=goal_altd,output_limits=(-1.0,1.0))
        n=0
        x_list = []
        altd_list = []
        x_list.append(0)
        altd_list.append(0.00)

        while True:
            thrust = altdController(altd)
            n +=1
            #if n==20:
                #n=0
            print(n*0.05,altd,thrust,goal_altd-altd)
            time.sleep(0.05)
            self.pub_velocity.publish(Twist(Vector3(0,0,thrust),Vector3(0,0,0)))
            altd_list.append(altd)
            x_list.append(n*0.05)
            if start== 1:
                plt.hlines([goal_altd], 0, n*0.05, "r", linestyles='dashed')  
                plt.plot(x_list, altd_list, color="r")
                plt.plot(x_list, altd_list)
                plt.ylim(0,5000)
                plt.xlim(0,20)
                plt.xlabel('time, s')
                plt.ylabel('altitude, mm')
                break

    def lat(self):
        global goal_lat
        goal_lat = 35.709674
        latController = PID(1.0,0,0, sample_time=0.1, setpoint=goal_lat,output_limits=(-1.0,1.0))
        n=0
        x_list = []
        lat_list = []
        x_list.append(0)
        lat_list.append(0.00)

        while True:
            pitch = -latController(lat)*1000
            n +=1
            #if n==20:
                #n=0
            print(n*0.05,lat,pitch,goal_lat-lat)
            time.sleep(0.05)
            self.pub_velocity.publish(Twist(Vector3(pitch,0,0),Vector3(0,0,0)))
            lat_list.append(lat)
            x_list.append(n*0.05)
            if start== 1:
                plt.hlines([goal_lat], 0, n*0.05, "r", linestyles='dashed')  
                plt.plot(x_list, lat_list, color="r")
                plt.plot(x_list, lat_list)
                plt.ylim(0,50)
                plt.xlim(0,20)
                plt.xlabel('time, s')
                plt.ylabel('latitude,  ')
                break

    def lon(self):
        global goal_lon
        goal_lon = 139.522940
        lonController = PID(2.0,0,0, sample_time=0.1, setpoint=goal_lon,output_limits=(-1.0,1.0))
        n=0
        x_list = []
        lon_list = []
        x_list.append(0)
        lon_list.append(0.00)

        while True:
            roll = lonController(lon)*10#1000
            n +=1
            #if n==20:
                #n=0
            print(n*0.05,lon,roll,goal_lon-lon)
            time.sleep(0.05)
            self.pub_velocity.publish(Twist(Vector3(0,roll,0),Vector3(0,0,0)))
            lon_list.append(lon)
            x_list.append(n*0.05)
            if start== 1:
                plt.hlines([goal_lon], 0, n*0.05, "r", linestyles='dashed')  
                plt.plot(x_list, lon_list, color="r")
                plt.plot(x_list, lon_list)
                plt.ylim(0,150)
                plt.xlim(0,20)
                plt.xlabel('time, s')
                plt.ylabel('longitude,  ')
                break

#main-------------
    def main(self):
        if a == 1:
            self.pub_takeoff.publish(Empty())
            rospy.sleep(4.0)
            self.pub_velocity.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))

        elif b == 1:
            self.pub_land.publish(Empty())
            self.pub_velocity.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
	elif x == 1:
            self.pub_reset.publish(Empty())
            #self.pub_velocity.publish(Twist(Vector3(1,0,0),Vector3(0,0,0)))
            rospy.sleep(1.0)
        elif y == 1:
            plt.show()
            #self.excel()
        #elif start == 1:
            #self.altd()
        elif plusbotton_fb == 1:
            self.altd()
        elif plusbotton_fb == -1:
            self.magY()
        elif plusbotton_lr == 1:
            self.lat()
        elif plusbotton_lr == -1:
            self.lon()
        else:
            #print(lat,lon,battery,altd,mag)
            print(battery,lat,lon)
            self.pub_velocity.publish(Twist(Vector3(pitch,roll,thrust),Vector3(0,0,yaw)))

if __name__ == '__main__':
    rospy.init_node('moving_node', anonymous=True)
    s = controll()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        s.main()#rosservice call /ardrone/togglecam 
        rate.sleep()#rosrun image_view image_view image:=/arMakar

