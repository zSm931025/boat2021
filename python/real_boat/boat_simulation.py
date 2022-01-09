import threading
import socket
import time
import traceback
import struct

def checksum(data):
    ck1=0
    ck2=0
    length = len(data)
    for i in range(length):
        ck1 = ck1 + data[i];
        ck2 = ck1 + ck2;
    return ck1&0xFF,ck2&0xFF


class REAL_BOAT():
    def __init__(self):
        self.m11 = 5.13
        self.m22 = 10000000
        self.m33 = 13.6
        self.d11 = 19.338
        self.d22 = 10000000
        self.d33 = 6.2544


        self.com_status = 0
        self.wifi = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.wifi_host = "localhost"
        self.wifi_port = 8083


    def start_simulation(self):
        wifi_ = threading.Thread(target=self.run_wifi,args=(),name="run wifi")
        wifi_.start()

    def send_wifi(self,client,addr):
        try:
            while(1):
                time.sleep(2)
                send_data=[1,2,3,4,5,5,6,7]
                client.send(bytes(send_data))

        except Exception as e:
            print("send wifi:{}".format(e))

    def read_wifi(self,client,addr):
        first_recv_flag = 1
        last_count = 0
        try:
            while(1):
                raw_frame = client.recv(1024)
                if (len(raw_frame) >= 4):
                    ck1, ck2 = checksum(raw_frame[2:-2])
                    if (ck1 == raw_frame[-2] and ck2 == raw_frame[-1]):
                        if (first_recv_flag):
                            last_count = raw_frame[3]
                            first_recv_flag = 0
                        else:
                            last_count += 1
                            if (last_count == 256):
                                last_count = 0
                        now_count = raw_frame[3]
                        if (last_count != now_count):
                            # print("lose frame ! all lose nums:", all_lose_nums, "now_count:", now_count, "last_count+1:",
                            #       last_count)
                            print("丢帧！")
                            last_count = now_count
                        print(raw_frame)
                    else:
                        print("校验失败！")
                else:
                    print("no data")

        except Exception as e:
            print("read wifi:{}".format(e))

    def run_wifi(self):
        self.doConnect()
        while True:
            try:
                client,addr = self.wifi.accept()
                threading.Thread(target=self.send_wifi,args=(client,addr),name = "send_wifi").start()
                threading.Thread(target=self.read_wifi,args=(client,addr),name = "read_wifi").start()
                print(threading.enumerate())
            except socket.error:
                traceback.print_exc()
                print("socket connect error doing connect 2s. host:{}/port:{} stared listen..".format(self.wifi_host,self.wifi_port))
                time.sleep(2)
            except Exception as e:
                print("other error occur:{}".format(e))
                time.sleep(2)

    def doConnect(self):
        while True:
            try:
                self.wifi.bind((self.wifi_host,self.wifi_port))
                self.wifi.listen(1)
                print("--------------------")
                print("server host:{}/port:{} stared listen..".format(self.wifi_host,self.wifi_port))
                print("--------------------")
                break
            except Exception as e:
                time.sleep(1)
                print("start ws server error:{}".format(str(e)))
                traceback.print_exc()

    def excute_boat(self):
        pass



if __name__=="__main__":
    real_boat = REAL_BOAT()
    real_boat.start_simulation()
    while(True):
        time.sleep(5)
        print(threading.enumerate())
        print("loop")