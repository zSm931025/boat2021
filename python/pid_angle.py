
class PID_ANGLE():
    def __init__(self):
        self.k_p = 1
        self.k_i = 0
        self.k_d = 1#1
        self.error = 0
        self.last_error = 0
        self.integral = 0
        self.max_integral = 100  #积分超过这一值不再进行积分
        self.integral_threshold_error  = 10  #大于此角度不用进行积分


    def process(self,now_angle,aim_angle):
        self.now_angle = now_angle
        self.aim_angle = aim_angle
        error_temp = self.aim_angle-self.now_angle
        if error_temp>0 and error_temp<=180:
            self.error = error_temp
        elif error_temp>0 and error_temp>180:
            self.error = error_temp-360
        elif error_temp<=0 and error_temp>-180:
            self.error = error_temp
        else:
            self.error = 360+error_temp


        # if abs(self.error)<3:
        #     self.k_p = 5
        # else:
        #     self.k_p = 1

        if(abs(self.error)<self.integral_threshold_error):
            self.integral+=self.error
            if(self.integral>self.max_integral):
                self.integral = self.max_integral
        else:
            self.integral = 0
        output = self.k_p*self.error+self.k_i*self.integral+self.k_d*(self.error-self.last_error)
        self.last_error = self.error
        if output>100:
            output = 100
        if output<-100:
            output = -100
        #print(self.error,-output,end="")
        return -output  #