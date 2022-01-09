import numpy as np
from pid_angle2 import *
from track_test import *
from boat_api import *


class TRACK_():
    def __init__(self):
        self.speed = 0
        self.rotation = 0
        self.reach_threthold = 0.5
        self.dis_ahead = 1
        self.pid_func = PID_ANGLE2()
        # self.path = [[[-1,-4],[-0.5,3]],[[-0.5,3],[5,5]]]
        pass

    def track_process(self,path):
        if len(path)!=0:
            for line in path:
                line_start = line[0]
                line_end = line[1]
                line_angle = self.get_line_direction(line_start,line_end)
                self.get_now_status()
                while(not self.if_reach_point(line_start)):
                    self.line_track([self.p_x,self.p_y],line_start)
                self.line_track(line_start, line_end)
        else:
            print("no line to track!")

    def angle_track(self):
        pass

    def line_track(self,start_point,end_point):
        global sendmsg
        line_angle = self.get_line_direction(start_point,end_point)/np.pi*180.0
        x1 = start_point[0]
        y1 = start_point[1]
        x2 = end_point[0]
        y2 = end_point[1]
        line_A = y1-y2
        line_B = x2-x1
        line_C = (y2-y1)*x1+(x2-x1)*y1
        while(not self.if_reach_point(end_point)):
            self.get_now_status()
            dis_to_line = np.abs(line_A*self.p_x+line_B*self.p_y+line_C)/np.sqrt(line_A**2+line_B**2)
            delta_angle = np.arctan(dis_to_line/self.dis_ahead)
            angle0 = self.get_line_direction([self.p_x,self.p_y],end_point)/np.pi*180
            error_temp = angle0 - line_angle
            if error_temp > 0 and error_temp <= 180:
                p_error = error_temp
            elif error_temp > 0 and error_temp > 180:
                p_error = error_temp - 360
            elif error_temp <= 0 and error_temp > -180:
                p_error = error_temp
            else:
                p_error = 360 + error_temp
            if p_error>0:
                aim_angle = line_angle+delta_angle
            else:
                aim_angle = line_angle-delta_angle
            if aim_angle<0:
                aim_angle+=360
            if aim_angle>360:
                aim_angle-=360
            sendmsg["ctl_cmd"]["rotation"] = self.pid_func.process(self.p_seta, aim_angle, self.p_seta_speed)
            sendmsg["ctl_cmd"]["velocity"] = 3


    def curve_track(self):
        pass

    def if_reach_point(self,point):
        self.get_now_status()
        dis = np.sqrt((self.p_x-point[0])**2+(self.p_y-point[1])**2)
        return dis<self.reach_threthold

    def get_now_status(self):
        global gX,gY,gSeta,gSeta_speed
        self.p_x = gX
        self.p_y = gY
        self.p_seta=gSeta
        self.p_seta_speed = gSeta_speed

    def get_line_direction(self,start_point,end_point):
        x = end_point[0] - start_point[0]
        y = end_point[1] - start_point[1]
        if x == 0 and y > 0:
            seta = np.pi / 2
        elif x == 0 and y < 0:
            seta = 3 * np.pi / 2
        elif y == 0 and x > 0:
            seta = 0
        elif y == 0 and x < 0:
            seta = np.pi
        elif (x < 0 and y < 0) or (x < 0 and y > 0):
            seta = np.pi + np.arctan(y / x)
        else:
            seta = np.arctan(y / x)
        if seta<0:
            seta+=2*np.pi
        return seta

