import numpy as np
import serial
import serial.tools.list_ports
import threading
import time
import struct
import socket
import cv2
import matplotlib.pyplot  as plt
from gps_resource import *
from pid_angle2 import *


plot_0 = True
base = [0,0]  #经度， 纬度

# plt.ion()
# fig,axes = plt.subplots()
# axes.set(title = "cutom_map",ylim = [-1,13],xlim =[-1,4],ylabel = "Y-axis",xlabel = "X-axis")
#axes.set(title = "cutom_map",ylim = [-100,100],xlim =[0,100],ylabel = "Y-axis",xlabel = "X-axis")
#axes.set_aspect(1)
# gro_z = [0]*100
# line  = axes.plot(gro_z)

# plt.show()
# plt.pause(0.001)


sendmsg ={"msg_cmd":{"enable":0,"status":0,"id":2,"len":1,"msgset":[0,0,0,0,0,0,0,0]},
          "ctl_cmd":{"enable":1,"status":0,"id":1,"len":5,"mode":1,"velocity":0,"rotation":0,"accelerate":100,"gear":3}}

recvmsg = {"gps":{"status":None,"id":1,"len":15,"lon_label": None, "lon_degree": None, "lon_cent": None, "lon_second": None,
                  "lat_label": None, "lat_degree": None, "lat_cent": None, "lat_second": None,"gps_speed":None,
                  "gps_mode": None, "gps_status": None},
           "tim":{"status":None,"id":6,"len":6,"year": None, "month": None, "day": None, "hour": None, "minute": None,"second": None},
           "acc":{"status":None,"id":2,"len":12,"x": None, "y": None, "z": None},
           "eul":{"status":None,"id":3,"len":12,"x": None, "y": None, "z": None},
           "gro":{"status":None,"id":4,"len":12,"x": None, "y": None, "z": None},
           "qua":{"status":None,"id":5,"len":16,"qua1": None, "qua2": None, "qua3": None,"qua4":None},
           "sys":{"status":None,"id":7,"len":6,"mode":None,"boat_status":None,"true_left":None,"true_right":None}}

def checksum(data):
    ck1=0
    ck2=0
    length = len(data)
    for i in range(length):
        ck1 = ck1 + data[i];
        ck2 = ck1 + ck2;
    return ck1&0xFF,ck2&0xFF

class S_MSG():
    def __init__(self):
        super(S_MSG,self).__init__()
        self.count = 0
        self.send_bytes = [241, 242, 3,self.count]

    def calculate_info(self):
        global sendmsg
        key_list =list(sendmsg.keys())
        key_len = len(key_list)
        msg_id = 0
        msg_id_buf = 0
        while(1):
            while(msg_id<key_len):
                key = key_list[msg_id]
                self.send_bytes = [241, 242, 3, self.count]
                if sendmsg[key]["status"]:
                    self.constructure_info(key)
                    sendmsg[key]["status"] = 0
                elif sendmsg[key]["enable"]:
                    self.constructure_info(key)
                else:
                    msg_id+=1
                    continue
                self.count = (self.count + 1) % 256
                self.send_bytes[3] = self.count

                ck1,ck2 = checksum(self.send_bytes[2:])
                self.send_bytes.extend([ck1,ck2])
                yield self.send_bytes
                msg_id+=1
                msg_id_buf = msg_id
            msg_id = 0
            res  = 0
            while(msg_id<msg_id_buf):
                key = key_list[msg_id]
                self.send_bytes = [241, 242, 3, self.count]
                if sendmsg[key]["status"]:
                    self.constructure_info(key)
                    sendmsg[key]["status"] = 0
                elif sendmsg[key]["enable"]:
                    self.constructure_info(key)
                else:
                    res = 0
                    msg_id+=1
                    continue
                self.count = (self.count + 1) % 256
                self.send_bytes[3] = self.count
                ck1, ck2 = checksum(self.send_bytes[2:])
                self.send_bytes.extend([ck1, ck2])
                yield self.send_bytes
                res=1
                msg_id += 1
                msg_id_buf = msg_id
                break
            if res==0:
                yield None
                msg_id = msg_id_buf = 0


    def constructure_info(self,key):
        global sendmsg
        if key=="msg_cmd":
            self.send_bytes.append(0x21)
            temp = 0
            for i in range(8):
                if sendmsg["msg_cmd"]["msgset"][i]==1:
                    temp+=(1<<i);
            self.send_bytes.append(temp)
            self.send_bytes[2]+= 1
        if key == "ctl_cmd":
            self.send_bytes.append(0x15)
            self.send_bytes.append(sendmsg["ctl_cmd"]["mode"]) #(0:静默，1:数传，2:遥控器)
            self.send_bytes.append(sendmsg["ctl_cmd"]["velocity"] if sendmsg["ctl_cmd"]["velocity"]>=0 else 256+sendmsg["ctl_cmd"]["velocity"])
            self.send_bytes.append(sendmsg["ctl_cmd"]["rotation"] if sendmsg["ctl_cmd"]["rotation"]>=0 else 256+sendmsg["ctl_cmd"]["rotation"])
            self.send_bytes.append(sendmsg["ctl_cmd"]["accelerate"] )
            self.send_bytes.append(sendmsg["ctl_cmd"]["gear"])
            self.send_bytes[2] += 6
            #print("send:",sendmsg["ctl_cmd"]["rotation"])

class R_MSG():
    def __init__(self):
        super(R_MSG, self).__init__()
        self.count=0

    def form_frame(self,frame,head):
        if head in frame:
            addr = frame.index(head)
            frame = frame[addr:]
            if len(frame) > 2:
                length = frame[2]
                if (len(frame) >= length + 3) and length>0:
                    data = frame[:length + 3]
                    last_frame = frame[length + 3:]
                    return data, last_frame
                else:
                    return None, frame
            else:
                return None, frame
        else:
            return None, frame[-1:]

    def check_frame(self,data):
        ck1,ck2 = checksum(data[2:-2])
        return (ck1==data[-2] and ck2==data[-1])

    def calculate_info(self,raw_data):
        self.length = len(raw_data)
        self.data = list(struct.unpack("B" * (self.length), raw_data))
        self.raw_data = raw_data
        read_index = 4
        while read_index < self.length - 2:
            id = self.data[read_index] >> 4
            id_len = self.data[read_index] & 0x0f
            if(id_len==0):
                id_len = 16
            if id == 1:
                self.get_gps_info(self.raw_data[read_index+1:read_index+id_len+1])
            elif id==2:
                self.get_acc_info(self.raw_data[read_index + 1:read_index + id_len + 1])
            elif id==3:
                self.get_eul_info(self.raw_data[read_index + 1:read_index + id_len + 1])
            elif id==4:
                self.get_gro_info(self.raw_data[read_index + 1:read_index + id_len + 1])
            elif id==5:
                self.get_qua_info(self.raw_data[read_index + 1:read_index + id_len + 1])
            elif id==6:
                self.get_tim_info(self.raw_data[read_index + 1:read_index + id_len + 1])
            elif id==7:
                self.get_sys_info(self.raw_data[read_index + 1:read_index + id_len + 1])
            else:
                pass
            read_index += id_len + 1

    def get_gps_info(self,payload):
        global recvmsg
        global plot_0
        global base
        global axes
        global plt
        temp0 = struct.unpack('B', payload[0:1])[0]
        if((temp0&0x01)==0):
            recvmsg["gps"]["lon_label"] = 'W'
        else:
            recvmsg["gps"]["lon_label"] = 'E'

        if ((temp0 & 0x02)>>1 == 0):
            recvmsg["gps"]["lat_label"] = 'S'
        else:
            recvmsg["gps"]["lat_label"] = 'N'

        if ((temp0 & 0x0c)>>2 == 0):
            recvmsg["gps"]["gps_mode"] = 'A'
        elif((temp0 & 0x0c)>>2 == 1):
            recvmsg["gps"]["gps_mode"] = 'D'
        elif ((temp0 & 0x0c) >> 2 == 2):
            recvmsg["gps"]["gps_mode"] = 'E'
        else:
            recvmsg["gps"]["gps_mode"] = 'N'
        if((temp0&0x10)>>4 ==1):
            recvmsg["gps"]["gps_status"] = 'A'
        else:
            recvmsg["gps"]["gps_status"] = 'V'
        recvmsg["gps"]["lon_degree"] = struct.unpack("B", payload[1:2])[0]
        recvmsg["gps"]["lon_cent"] = struct.unpack("B", payload[2:3])[0]
        recvmsg["gps"]["lon_second"] = struct.unpack("f", payload[3:7])[0]
        recvmsg["gps"]["lat_degree"] = struct.unpack("B", payload[7:8])[0]
        recvmsg["gps"]["lat_cent"] = struct.unpack("B", payload[8:9])[0]
        recvmsg["gps"]["lat_second"] = struct.unpack("f", payload[9:13])[0]
        recvmsg["gps"]["gps_speed"] = struct.unpack("h", payload[13:15])[0]
        #print(recvmsg["gps"])





    #print(recvmsg["gps"])
        if not hasattr(self,"gps_start"):
            self.gps_start =time.time()
            self.gps_count = 1
        else:
            self.gps_count+=1
            # print("gps",self.gps_count/(time.time()-self.gps_start+0.001))

    def get_acc_info(self, payload):
        global recvmsg
        recvmsg["acc"]["status"] = 1
        recvmsg["acc"]["x"] = struct.unpack("f", payload[0:4])[0]
        recvmsg["acc"]["y"] = struct.unpack("f", payload[4:8])[0]
        recvmsg["acc"]["z"] = struct.unpack("f", payload[8:12])[0]

        if not hasattr(self,"acc_start"):
            self.acc_start =time.time()
            self.acc_count = 1
        else:
            self.acc_count+=1
            # print("acc",self.acc_count/(time.time()-self.acc_start+1e-10))

    def get_eul_info(self, payload):
        global recvmsg
        recvmsg["eul"]["status"] = 1
        recvmsg["eul"]["x"] = struct.unpack("f", payload[0:4])[0]
        recvmsg["eul"]["y"] = struct.unpack("f", payload[4:8])[0]
        recvmsg["eul"]["z"] = struct.unpack("f", payload[8:12])[0]

        if not hasattr(self, "eul_start"):
            self.eul_start = time.time()
            self.eul_count = 1
        else:
            self.eul_count += 1
            #print("eul", self.eul_count / (time.time() - self.eul_start+1E-10))
        # print("eul_Z:",recvmsg["eul"]["z"])

    def get_gro_info(self, payload):
        global recvmsg
        recvmsg["gro"]["status"] = 1
        recvmsg["gro"]["x"] = struct.unpack("f", payload[0:4])[0]
        recvmsg["gro"]["y"] = struct.unpack("f", payload[4:8])[0]
        recvmsg["gro"]["z"] = struct.unpack("f", payload[8:12])[0]

        if not hasattr(self, "gro_start"):
            self.gro_start = time.time()
            self.gro_count = 1
        else:
            self.gro_count += 1
            #print("gro", self.gro_count / (time.time() - self.gro_start+1E-10))
        #print("gro_z:",recvmsg["gro"]["z"])




    def get_qua_info(self, payload):
        global recvmsg
        recvmsg["qua"]["status"] = 1
        recvmsg["qua"]["qua1"] = struct.unpack("f", payload[0:4])[0]
        recvmsg["qua"]["qua2"] = struct.unpack("f", payload[4:8])[0]
        recvmsg["qua"]["qua3"] = struct.unpack("f", payload[8:12])[0]
        recvmsg["qua"]["qua4"] = struct.unpack("f", payload[12:16])[0]

        if not hasattr(self, "qua_start"):
            self.qua_start = time.time()
            self.qua_count = 1
        else:
            self.qua_count += 1
            # print("qua", self.qua_count / (time.time() - self.qua_start+1E-10))

    def get_tim_info(self, payload):
        global recvmsg
        recvmsg["tim"]["status"] = 1
        temp0 = struct.unpack("H" , payload[0:2])[0]
        recvmsg["tim"]["year"] = temp0 >> 9
        recvmsg["tim"]["month"] = (temp0&0x01E0)>>5
        recvmsg["tim"]["day"] = temp0&0x001F
        recvmsg["tim"]["hour"] = struct.unpack("B" , payload[2:3])[0]
        recvmsg["tim"]["minute"] = struct.unpack("B" , payload[3:4])[0]
        recvmsg["tim"]["second"] = struct.unpack("H", payload[4:6])[0] / 100.0


        if not hasattr(self, "tim_start"):
            self.tim_start = time.time()
            self.tim_count = 1
        else:
            self.tim_count += 1
            # print("tim", self.tim_count / (time.time() - self.tim_start))

    def get_sys_info(self, payload):
        global recvmsg
        recvmsg["sys"]["mode"] = struct.unpack("B",payload[0:1])[0]
        recvmsg["sys"]["boat_status"] = struct.unpack("B",payload[1:2])[0]
        recvmsg["sys"]["true_left"] = struct.unpack("H",payload[2:4])[0]
        recvmsg["sys"]["true_right"] = struct.unpack("H",payload[4:6])[0]

        if not hasattr(self, "sys_start"):
            self.sys_start = time.time()
            self.sys_count = 1
        else:
            self.sys_count += 1
            # print("sys", self.sys_count / (time.time() - self.sys_start+1e-10))




class BOAT_():
    def __init__(self):
        super(BOAT_,self).__init__()

        self.stopEvent = threading.Event()
        self.stopreadEvent = threading.Event()
        self.stopsendEvent = threading.Event()
        self.stopEvent.clear()
        self.stopreadEvent.clear()
        self.stopsendEvent.clear()
        self.loseConnectEvent = threading.Event()
        self.loseConnectEvent.clear()


        self.com_status = 0
        self.wifi = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.wifi_host  = "192.168.16.254"
        self.wifi_port = 9999

        self.ser=serial.Serial()
        self.ser_port     = "com5"
        self.ser_baudrate = "115200"
        self.ser_bytesize = 8
        self.ser_stopbits = 1
        #self.ser_partity = None
        self.ser_timeout = 0.00001

    def config_serial(self,config_dict):
        self.ser_port = config_dict["port"]
        self.ser_baudrate = config_dict["baudrate"]
        self.ser_bytesize = config_dict["bytesize"]
        self.ser_stopbits = config_dict["stopbits"]
        # self.ser_partity  = None
        self.ser_timeout = config_dict["timeout"]

    def detect_serial(self):
        Com_Dict = {}
        port_list = list(serial.tools.list_ports.comports())
        for port in port_list:
            Com_Dict["%s" % port[0]] = "%s" % port[1]
        com_list =list(Com_Dict.keys())
        if(len(com_list)!=0):
            self.ser_port = list(Com_Dict.keys())[0]
        return Com_Dict

    def open_serial(self):
        if self.com_status == 1:
            return False
        ser          = self.ser #serial.Serial()
        ser.port     = self.ser_port
        ser.baudrate = self.ser_baudrate
        ser.bytesize = self.ser_bytesize
        ser.stopbits = self.ser_stopbits
        ser.timeout  = self.ser_timeout
        try:
            ser.open()
        except:
            return False
        if ser.isOpen():
            self.com_status = 1
            read_s = threading.Thread(target=self.read_mcu_serial,args = (ser,))
            send_s = threading.Thread(target=self.send_mcu_serial,args = (ser,))
            protect_s = threading.Thread(target=self.protect_serial,args= (ser,))
            read_s.start()
            send_s.start()
            protect_s.start()
            return True
        else:
             return False

    def close_serial(self):
        if self.com_status == 0:
            print("已经是关闭的")
            return False
        print("start to close serial!")
        self.stopreadEvent.clear()
        self.stopsendEvent.clear()
        self.stopEvent.set()
        self.stopreadEvent.wait()
        self.stopsendEvent.wait()
        self.ser.close()
        self.com_status = 0
        print("close serial complete!")
        return True

    #@staticmethod
    def protect_serial(self,serHandle):
        while not self.stopEvent.is_set():
            self.loseConnectEvent.wait()
            serHandle.close()
            try:
                serHandle.open()
                self.loseConnectEvent.clear()
            except:
                print("wrong--------------------------------!")
        print("close protect thread!")

    # @staticmethod
    def read_mcu_serial(self,READ):
        read_msg = R_MSG()
        head = bytes([241, 242])
        last_frame = bytes([])
        first_recv_flag = 1
        last_count = 0
        all_lose_nums = 0
        print("开始 read_thread")
        while not self.stopEvent.is_set():
            if not self.loseConnectEvent.is_set():
                time.sleep(0.001)
                raw_frame = bytes([])
                try:
                    raw_frame = READ.read(1024)
                except:
                    self.loseConnectEvent.set()
                    continue
                now_frame = raw_frame
                while(len(raw_frame)==1024):
                    try:
                        raw_frame = READ.read(1024)
                    except:
                        self.loseConnectEvent.set()
                        break
                    now_frame += raw_frame
                if self.loseConnectEvent.is_set():
                    continue
                if last_frame != bytes([]):
                    frame = last_frame + now_frame
                else:
                    frame = now_frame
                if(len(frame)!=0):
                    data, last_frame = read_msg.form_frame(frame, head)
                else:
                    continue
                while(data!=None):
                    if read_msg.check_frame(data):
                        if(first_recv_flag):
                            last_count = data[3]
                            first_recv_flag = 0
                        else:
                            last_count+=1
                            if(last_count==256):
                                last_count = 0
                        now_count = data[3]
                        if (last_count != now_count):
                            # print("lose frame ! all lose nums:", all_lose_nums, "now_count:", now_count, "last_count+1:",
                            #       last_count)
                            all_lose_nums += 1
                            print("丢帧！")
                            last_count = now_count
                        read_msg.calculate_info(data)
                    else:
                        all_lose_nums+=1
                        print("校验失败！")
                    if(last_frame!=None):
                        data, last_frame = read_msg.form_frame(last_frame, head)
                    else:
                        data = None
            else:
                time.sleep(0.1)
        self.stopreadEvent.set()
        print("close read thread!")

    # @staticmethod
    def send_mcu_serial(self,SEND):
        send_msg = S_MSG()
        get_bytes = send_msg.calculate_info()
        # count = 0
        # time1 =  time.time()
        while not self.stopEvent.is_set():
            if not self.loseConnectEvent.is_set():
                time.sleep(0.01)
                send_bytes= next(get_bytes)
                if send_bytes==None:
                    continue
                # count+=1
                # #print("len:",len(send_bytes),"frequence:",count/(time.time()-time1),send_bytes)
                try:
                    SEND.write(bytes(send_bytes))
                except:
                    self.loseConnectEvent.set()
            else:
                time.sleep(0.1)
        self.stopsendEvent.set()
        print("close send thread!")



    def config_wifi(self,config_dict):
        self.wifi_host = config_dict["host"]
        self.wifi_port = config_dict["port"]

    def open_wifi(self):
        if self.com_status == 1:
            return False
        wifi = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = self.wifi_host
        port = int(self.wifi_port)
        wifi.settimeout(1)
        try:
            wifi.connect((host, port))
            # self.wifi.settimeout(None)
        except:
            print("process has die")
            return False
        self.com_status = 1
        read_w = threading.Thread(target=self.read_mcu_wifi, args=(self.wifi,))
        send_w = threading.Thread(target=self.send_mcu_wifi, args=(self.wifi,))
        protect_w = threading.Thread(target=self.protect_wifi, args=(wifi,))
        read_w.start()
        send_w.start()
        protect_w.start()
        return True

    def close_wifi(self):
        if self.com_status == 0:
            print("已经是关闭的")
            return False
        self.stopreadEvent.clear()
        self.stopsendEvent.clear()
        self.stopEvent.set()
        self.stopreadEvent.wait()
        self.stopsendEvent.wait()
        self.wifi.close()
        self.com_status = 0
        print("close wifi end")
        return True
    # def ping_wifi(self):
    #     process = subprocess.Popen(self.wifi_host, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    #     command_output = process.stdout.read().decode('GB2312')
    #     print(command_output)
    #     return command_output

    # @staticmethod
    def protect_wifi(self,wifiHandle):
        while not self.stopEvent.is_set():
            self.loseConnectEvent.wait()
            wifiHandle.close()
            try:
                wifiHandle.open()
                self.loseConnectEvent.clear()
            except:
                print("wrong--------------------------------!")
        print("close protect thread!")

    # @staticmethod
    def read_mcu_wifi(self, READ):
        read_msg = R_MSG()
        head = bytes([241, 242])
        last_frame = bytes([])
        first_recv_flag = 1
        last_count = 0
        all_lose_nums = 0
        print("开始 read_thread")
        while not self.stopEvent.is_set():
            if not self.loseConnectEvent.is_set():
                time.sleep(0.01)
                raw_frame = bytes([])
                try:
                    raw_frame = READ.recv(1024)
                except:
                    self.loseConnectEvent.set()
                    continue
                    #print("超时", len(raw_frame))
                now_frame = raw_frame
                while (len(raw_frame) == 1024):
                    try:
                        raw_frame = READ.recv(1024)
                    except:
                        self.loseConnectEvent.set()
                        break
                        #print("超时", len(raw_frame))
                    now_frame += raw_frame
                if self.loseConnectEvent.is_set():
                    continue
                if last_frame != bytes([]):
                    frame = last_frame + now_frame
                else:
                    frame = now_frame
                if (len(frame) != 0):
                    data, last_frame = read_msg.form_frame(frame, head)
                else:
                    continue
                while (data != None):
                    if read_msg.check_frame(data):
                        if (first_recv_flag):
                            last_count = data[3]
                            first_recv_flag = 0
                        else:
                            last_count += 1
                            if (last_count == 256):
                                last_count = 0
                        now_count = data[3]
                        if (last_count != now_count):
                            # print("lose frame ! all lose nums:", all_lose_nums, "now_count:", now_count, "last_count+1:",
                            #       last_count)
                            all_lose_nums += 1
                            print("丢帧！")
                            last_count = now_count
                        read_msg.calculate_info(data)
                    else:
                        all_lose_nums += 1
                        print("校验失败！")
                    if (last_frame != None):
                        data, last_frame = read_msg.form_frame(last_frame, head)
                    else:
                        data = None
            else:
                time.sleep(0.1)
        self.stopreadEvent.set()
        print("close read thread!")

    # @staticmethod
    def send_mcu_wifi(self, SEND):
        send_msg = S_MSG()
        get_bytes = send_msg.calculate_info()
        count = 0
        time1 = time.time()
        while not self.stopEvent.is_set():
            if not self.loseConnectEvent.is_set():
                time.sleep(0.01)
                send_bytes = next(get_bytes)
                if send_bytes == None:
                    continue
                count += 1
                # print("len:",len(send_bytes),"frequence:",count/(time.time()-time1),send_bytes)
                # SEND.send(bytes(send_bytes))
                try:
                    SEND.sendall(bytes(send_bytes))
                except:
                    self.loseConnectEvent.set()
            else:
                time.sleep(0.1)
        self.stopsendEvent.set()
        print("close send thread!")





if __name__=="__main__":
    demo = BOAT_()
    print(demo.detect_serial())
    print("\n\n\n\n##########################################################\n\n\n\n")
    # time.sleep(3)
    demo.open_serial()
    #demo.open_wifi()
    method_angle = PID_ANGLE2()
    aim_angle =30
    if aim_angle < 0:
        aim_angle += 360
    while True:
        if recvmsg["eul"]["z"]!=None and recvmsg["gro"]["z"]!=None:
            now_angel = recvmsg["eul"]["z"]
            if now_angel<0:
                now_angel+=360
            now_speed = recvmsg["gro"]["z"]
            #print("nowAngle-aimAngle%5.2f"%(aim_angle-now_angel), end="   ")
            res  = method_angle.process(now_angel,aim_angle,now_speed)
            #sendmsg["ctl_cmd"]["rotation"]  = int(res)
            #print("output:%5.2f"%(res))

            #print("now:",now_angel,"   aim:",aim_angle,"  rotation:",res)
        # if recvmsg["gps"]["lon_degree"]!= None:
        #     # longitude_ = float(recvmsg["gps"]["lon_degree"]) + float(recvmsg["gps"]["lon_cent"]) / 60.0 + float(
        #     #     recvmsg["gps"]["lon_second"]) / 3600.0
        #     # latitude_ = float(recvmsg["gps"]["lat_degree"]) + float(recvmsg["gps"]["lat_cent"]) / 60.0 + float(
        #     #     recvmsg["gps"]["lat_second"]) / 3600.0
        #     longitude_ = recvmsg["gps"]["lon_degree"] + recvmsg["gps"]["lon_cent"]/ 60.0 + recvmsg["gps"]["lon_second"]/ 3600.0
        #     latitude_ = recvmsg["gps"]["lat_degree"] + recvmsg["gps"]["lat_cent"] / 60.0 + recvmsg["gps"]["lat_second"] / 3600.0
        #         plot_0 = False
        #         base = [longitude_, latitude_]
        #     else:
        #         x, y = calculate_coordinate_of_point_based_on_my_coordinate([longitude_, latitude_],base,-np.pi/7)
        #         axes.scatter(x, y, s=2, c="r", marker=".")
        # plt.show()
        # plt.pause(0.02)
        #time.sleep(0.3)
            # axes.lines.remove(line[0])
            # gro_z = gro_z[1:]
            # gro_z.append(recvmsg["gro"]["z"])
            # plt.show()
            # t1 = time.time()
            # min_ = min(gro_z)-5
            # max_ = max(gro_z)+5
            # axes.set(ylim=[min_, max_])
            # line  = axes.plot(gro_z,c="g")
        # plt.pause(0.001)
        # plt.show()


    demo.read.join()
    demo.send.join()
    print("hhh---------------------------------------------------------")
