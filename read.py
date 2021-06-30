#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3,Vector3Stamped
from ardrone_autonomy.msg import Navdata
from std_msgs.msg import Int32, Float32,String
import pandas as pd
import numpy as np

import time
from threading import Thread
a=b=x=y=pitch=roll=thrust=yaw=0
lstick_fb=lstick_lr=rstick_fb=rstick_lr=0
lat=lon=altd=battery=magX=magY=magZ=0
flag=0

class controll(object):
    def readxlsx(self):
        global data_read#,data_lat,data_lon,data_altd
        num = 1000
        #input_file_name ='traject_gps.xlsx'
        #input_book = pd.ExcelFile(input_file_name) 
        #input_sheet_name = input_book.sheet_names
        #input_sheet_df = input_book.parse(input_sheet_name[0])
        #data_read = input_sheet_df.head(num) 
        #print(data_read)
        df = pd.read_excel('/home/haruki/traject_gps.xlsx')
        #i=0
        if flag == 0:
            for i in range(num):
                data_lon = float(df.iloc[i,0])
                data_lat = float(df.iloc[i,1])
                data_altd = float(df.iloc[i,2])
                print(data_lat,data_lon,data_altd)
                time.sleep(5.0)

if __name__ == '__main__':
    s = controll()
    #rate = rospy.Rate(100)
    s.readxlsx()

