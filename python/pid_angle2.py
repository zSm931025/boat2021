
class PID_ANGLE2():
    def __init__(self):
        self.p_p = 3
        self.p_i = 0.00001
        self.p_d = 0#1
        self.p_error = 0
        self.p_last_error = 0
        self.p_integral = 0
        self.p_max_integral = 10000000  #积分超过这一值不再进行积分
        self.p_integral_threshold  = 10  #大于此角度不用进行积分
        self.p_max_output = 100

        self.v_p = 1
        self.v_i = 0
        self.v_d = 0    # 1
        self.v_error = 0
        self.v_last_error = 0
        self.v_integral = 0
        self.v_max_integral = 100  # 积分超过这一值不再进行积分
        self.v_integral_threshold = 10  # 大于此角度不用进行积分
        self.v_max_output = 100

    def process(self,now_angle,aim_angle,now_speed):
        """

        :param now_angle: input current angle
        :param aim_angle: input aim angle
        :return: expected angle speed
        """

        self.now_angle = now_angle
        self.aim_angle = aim_angle
        error_temp = self.aim_angle-self.now_angle
        if error_temp>0 and error_temp<=180:
            self.p_error = error_temp
        elif error_temp>0 and error_temp>180:
            self.p_error = error_temp-360
        elif error_temp<=0 and error_temp>-180:
            self.p_error = error_temp
        else:
            self.p_error = 360+error_temp

        #print("nowAngle-aimAngle%5.2f" % (self.p_error), end="   ")

        if(abs(self.p_error)<self.p_integral_threshold):
            self.p_integral+=self.p_error
            if(self.p_integral>self.p_max_integral):
                self.p_integral = self.p_max_integral
            if (self.p_integral < -self.p_max_integral):
                self.p_integral = -self.p_max_integral
        else:
            self.p_integral = 0
        #print("p_integral：%f" % (self.p_integral), end="   ")

        output = self.p_p*self.p_error+self.p_i*self.p_integral+self.p_d*(self.p_error-self.p_last_error)
        self.p_last_error = self.p_error
        if output>self.p_max_output:
            output = self.p_max_output
        if output<-self.p_max_output:
            output = -self.p_max_output


        self.now_speed = now_speed
        self.aim_speed = output
        self.v_error  = self.aim_speed-self.now_speed
        if(abs(self.v_error)<self.v_integral_threshold):
            self.v_integral+=self.v_error
            if(self.v_integral>self.v_max_integral):
                self.v_integral = self.v_max_integral
            if (self.v_integral < -self.v_max_integral):
                self.v_integral = -self.v_max_integral
        else:
            self.v_integral = 0
        output1 = self.v_p*self.v_error+self.v_i*self.v_integral+self.v_d*(self.v_error-self.v_last_error)
        self.v_last_error = self.v_error
        if output1 > self.v_max_output:
            output1 = self.v_max_output
        if output1 < -self.v_max_output:
            output1 = -self.v_max_output
        #print("aimSpeed:%5.2f"%(output),end="   ")
        #print("aimSpeed-nowSpeed:%5.2f"%(output-now_speed),end="   ")
        return -output1


