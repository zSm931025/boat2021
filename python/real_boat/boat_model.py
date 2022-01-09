import numpy
import numpy as np
import matplotlib.pyplot as plt
import time as time


# m11 = 2652.5199   #变量
# m22 = 2825.5836
# m33 = 4201.9
# d11 = 848.1313
# d22 = 9889.5426
# d33 = 22722.8330






class Model(object):
    def __init__(self):
        self.delta_t = 0.001  #
        self.m11 = m11
        self.m22 = m22
        self.m33 = m33
        self.d11 = d11
        self.d22 = d22
        self.d33 = d33

    def Solve(self,state,u,T):
        """

        :param state: 初始状态 x,y,psi,u,v,r     x,y,z坐标系符合右手原则，z指向地面，默认无人船z一直为0. u,v,r为载体坐标系下的速度，u指向载体系Xb,v指向Yb,r顺时针为正
        :param u: 输入 u(0) 为力，u(1) 为转矩
        :param T: 仿真时常
        :return:
        """

        #  风力干扰：    F = rou*A*V**2   绝对风速Vw   wseta    面积Au     Av
        #  浪力干扰：     不考虑
        #  流干扰  ：       Vf 绝对速度  fseta 绝对流向


        kf = 0
        Vf = 3
        seta = 45*np.pi/180.0

        kw = 0
        rou = 1.293
        Vw = 1
        wseta = 0*np.pi/180.0
        Au = 0.004
        Av = 0.3


        step = np.int(T/self.delta_t)
        x = []
        y = []
        for i in range(step):
            x_dot   =  state[3]*np.cos(state[2])-state[4]*np.sin(state[2])    #+ kf*Vf*np.cos(seta)    #流干扰
            y_dot   =  state[3]*np.sin(state[2])+state[4]*np.cos(state[2])    #+ kf*Vf*np.sin(seta)    #流干扰
            psi_dot =  state[5]
            u_dot   =  (u[0]-self.d11*state[3]+self.m22*state[4]*state[5])/self.m11     #-kw*rou*Au*(-Vw*np.cos(wseta-state[2])- state[3])**2     )/self.m11               #风干扰
            v_dot   =  (0-self.d22*state[4]-self.m11*state[3]*state[5])/self.m22         #-kw*rou*Av*(-Vw*np.sin(wseta-state[2])- state[4])**2     )/self.m22               #风干扰
            r_dot   =   (u[1]-self.d33*state[5]+(self.m11-self.m22)*state[3]*state[4])/self.m33

            state[0] = state[0]+   x_dot*  self.delta_t
            x.append(state[0])
            state[1] = state[1]+   y_dot*  self.delta_t
            y.append(state[1])
            state[2] = state[2]+ psi_dot*  self.delta_t
            state[3] = state[3]+   u_dot*  self.delta_t
            state[4] = state[4]+   v_dot*  self.delta_t
            state[5] = state[5]+   r_dot*  self.delta_t

        state_real = np.copy(state)
        #观测误差  x,y 由GPS 给出，误差error, x_dot,y_dot难以测量，psi 由imu psi_error，    u方向误差，难以测量   v 方向误差难以测量  r 角速度   r_error
        state_error = 1
        state[0] = state[0]+state_error*(np.random.rand()*2-1)*0.03  #x方向误差   <1
        state[1] = state[1]+state_error*(np.random.rand()*2-1)*0.03  #y方向误差   <1
        state[2] = state[2]+state_error*(np.random.rand()*2-1)*1/180.0*np.pi   #角度误差
        return state,state,[x,y]

if __name__=="__main__":
    t = 20
    T = 0.1
    boat_status = [0.5,1.5,0,0,0,0]
    boat_status1 = [0.5,1.5, 0, 0, 0, 0]
    boat = Model()
    count = int(t/T)
    u0 =  20
    u1 =  3
    info_collect = np.zeros((8, count))
    for i in range(count):
        info_collect[0:6, i] = boat_status[:]
        info_collect[6, i] = u0
        info_collect[7, i] = u1
        boat_status1,boat_status,[X,Y] = boat.Solve(boat_status,[u0,u1],T)


    fig, ax = plt.subplots()
    ax.plot(info_collect[1], info_collect[0])
    ax.set(ylim=[0, 4], xlim=[0, 6], xlabel='Y (m)', ylabel='x (m)',
           title='Real Path')    #Simulation
    ax.set_aspect(1)
    plt.show()


