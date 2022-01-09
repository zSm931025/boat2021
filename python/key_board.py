from PyQt5.QtWidgets import QApplication,QMainWindow,QWidget
from PyQt5 import QtGui
from PyQt5.QtCore import  Qt,QTimer
from boat_2021 import Ui_MainWindow
from config_serial import Ui_SERIAL
from config_wifi import Ui_WIFI
import cv2
from boat_api import *
import os
from get_video import *

# definition
# car_V:velocity(0-100)
# car_R:rotation(0-100)
# car_G:gear(0,1,2,3)
# car_Acc:accelerate(1-100)


# keys:
# Q: car_V+
# Z: car_V-
# W: car_R+
# X: car_R-
# E: car_Acc+
# C: car_ACC-
# R: car_G+
# V: car_G-
# U: left_foward
# I: forward
# O: right_foward
# J: left
# K: no move
# L: right
# M: backward_left
# <: backward
# >: backward_right
# 1: led_enable
# 2: led_head_mode
# 3: led_head_brightness+i
# 4: led_head_brightness-
# 5: led_tail_mode
# 6: led_tail_brightness+
# 7: led_tail_brithtness-


class config_WIF(QWidget,Ui_WIFI):
    def __init__(self,port):
        super(config_WIF,self).__init__()
        self.setupUi(self)
        self.port = port

    def config_ok(self):
        ip = self.ip_info.text()
        port = self.port_info.text()
        self.port.wifi_host = ip
        self.port.wifi_port = port
        self.close()

    def ip_config(self):
        cmd = "ipconfig"
        res = os.popen(cmd)
        output_str = res.read()  # 获得输出字符串
        self.show_res.setText(output_str)

    def ping(self):
        ip = self.ip_info.text()
        cmd ="ping "+ip
        res = os.popen(cmd)
        output_str = res.read()  # 获得输出字符串
        self.show_res.setText(output_str)

class config_SERIAL(QWidget,Ui_SERIAL):
    def __init__(self,port):
        super(config_SERIAL,self).__init__()
        self.setupUi(self)
        self.port = port

    def config_ok(self):
        self.port.ser_port = self.serial_select.currentText()
        self.port.ser_baudrate = int(self.serial_baudrate.currentText())
        self.port.ser_bytesize = int(self.serial_databits.currentText())
        self.port.ser_stopbits = int(self.serial_stopbits.currentText())
        self.ser_partity  = self.serial_parity.currentText()
        self.close()

    def detect_serial_fun(self):
        self.Com_Dict = {}
        port_list = list(serial.tools.list_ports.comports())
        self.serial_select.clear()
        for port in port_list:
            self.Com_Dict["%s" % port[0]] = "%s" % port[1]
            self.serial_select.addItem(port[0])
        if len(self.Com_Dict) == 0:
            print("无串口设备连接")
        self.serial_select.clearFocus()

lock = threading.RLock()

class key_ctl(QMainWindow,Ui_MainWindow):
    def __init__(self):
        super(key_ctl,self).__init__()
        self.setupUi(self)
        self.set_decorate()

        self.init_data()
        self.msg_show()
        self.label.grabKeyboard()

        self.demo = BOAT_()
        self.demo.detect_serial()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.msg_show)
        self.timer.start(1)

    def init_data(self):
        global sendmsg
        self.cmd_boat_velocity = 0
        self.cmd_boat_rotation = 0
        self.cmd_boat_gear  = 1
        self.cmd_boat_accelerate = 1
        self.set_v.setText(str(self.cmd_boat_velocity))
        self.set_r.setText(str(self.cmd_boat_rotation))
        self.set_a.setText(str(self.cmd_boat_accelerate))
        self.set_g.setText(str(self.cmd_boat_gear))

    def msg_show(self):
        global recvmsg
        self.msg_gps_mode.setText(recvmsg["gps"]["gps_mode"])
        self.msg_gps_status.setText(recvmsg["gps"]["gps_status"])
        self.msg_gps_speed.setText(str(recvmsg["gps"]["gps_speed"]))
        self.msg_gps_lon_label.setText(recvmsg["gps"]["lon_label"])
        self.msg_gps_lon_degree.setText(str(recvmsg["gps"]["lon_degree"]))
        self.msg_gps_lon_cent.setText(str(recvmsg["gps"]["lon_cent"]))
        self.msg_gps_lon_second.setText(str(recvmsg["gps"]["lon_second"]))
        self.msg_gps_lat_label.setText(recvmsg["gps"]["lat_label"])
        self.msg_gps_lat_degree.setText(str(recvmsg["gps"]["lat_degree"]))
        self.msg_gps_lat_cent.setText(str(recvmsg["gps"]["lat_cent"]))
        self.msg_gps_lat_second.setText(str(recvmsg["gps"]["lat_second"]))

        self.msg_gps_time_year.setText(str(recvmsg["tim"]["year"]))
        self.msg_gps_time_month.setText(str(recvmsg["tim"]["month"]))
        self.msg_gps_time_day.setText(str(recvmsg["tim"]["day"]))
        self.msg_gps_time_hour.setText(str(recvmsg["tim"]["hour"]))
        self.msg_gps_time_minute.setText(str(recvmsg["tim"]["minute"]))
        self.msg_gps_time_second.setText(str(recvmsg["tim"]["second"]))

        self.msg_boat_mode.setText(str(recvmsg["sys"]["mode"]))
        self.msg_boat_status.setText(str(recvmsg["sys"]["boat_status"]))
        self.msg_boat_true_left.setText(str(recvmsg["sys"]["true_left"]))
        self.msg_boat_true_right.setText(str(recvmsg["sys"]["true_right"]))

        self.msg_acc_x.setText(str(recvmsg["acc"]["x"]))
        self.msg_acc_y.setText(str(recvmsg["acc"]["y"]))
        self.msg_acc_z.setText(str(recvmsg["acc"]["z"]))
        self.msg_agl_x.setText(str(recvmsg["eul"]["x"]))
        self.msg_agl_y.setText(str(recvmsg["eul"]["y"]))
        self.msg_agl_z.setText(str(recvmsg["eul"]["z"]))
        self.msg_gyr_x.setText(str(recvmsg["gro"]["x"]))
        self.msg_gyr_y.setText(str(recvmsg["gro"]["y"]))
        self.msg_gyr_z.setText(str(recvmsg["gro"]["z"]))
        self.msg_quarter_1.setText(str(recvmsg["qua"]["qua1"]))
        self.msg_quarter_2.setText(str(recvmsg["qua"]["qua2"]))
        self.msg_quarter_3.setText(str(recvmsg["qua"]["qua3"]))
        self.msg_quarter_4.setText(str(recvmsg["qua"]["qua4"]))

    def set_decorate(self):
        path = "oubo.png"  #"C:\\Users\91242\desktop\oubo.png"
        picture = cv2.imread(path)
        picture = cv2.resize(picture, (500, 300))
        show2 = cv2.cvtColor(picture, cv2.COLOR_BGR2RGB)  # 视频色彩转换回RGB，这样才是现实的颜色
        showImage1 = QtGui.QImage(show2.data, picture.shape[1], picture.shape[0],
                                          QtGui.QImage.Format_RGB888)  # 把读取到的视频数据变成QImage形式
        self.picture_1.setPixmap(QtGui.QPixmap.fromImage(showImage1))  # 往显示视频

    def keyPressEvent(self,event):
        global sendmsg
        if event.key() == Qt.Key_Q:
            if(self.cmd_boat_velocity+1<100):
                self.cmd_boat_velocity+=1
            else:
                self.cmd_boat_velocity = 100
            self.set_v.setText(str(self.cmd_boat_velocity))

        if event.key() == Qt.Key_Z:
            if (self.cmd_boat_velocity - 1 >0):
                self.cmd_boat_velocity -=1
            else:
                self.cmd_boat_velocity = 0
            self.set_v.setText(str(self.cmd_boat_velocity))

        if event.key() == Qt.Key_W:
           if(self.cmd_boat_rotation+1<100):
                self.cmd_boat_rotation+=1
           else:
                self.cmd_boat_rotation=100
           self.set_r.setText(str(self.cmd_boat_rotation))

        if event.key() == Qt.Key_X:
            if (self.cmd_boat_rotation - 1 >0):
                self.cmd_boat_rotation -= 1
            else:
                self.cmd_boat_rotation = 0
            self.set_r.setText(str(self.cmd_boat_rotation))

        if event.key() == Qt.Key_E:
            if (self.cmd_boat_accelerate+1<100):
                self.cmd_boat_accelerate+=1
            else:
                self.cmd_boat_accelerate = 100
            sendmsg["ctl_cmd"]["accelerate"] = self.cmd_boat_accelerate
            self.set_a.setText(str(self.cmd_boat_accelerate))

        if event.key() == Qt.Key_C:
            if (self.cmd_boat_accelerate - 1 >1):
                self.cmd_boat_accelerate -= 1
            else:
                self.cmd_boat_accelerate = 1
            sendmsg["ctl_cmd"]["accelerate"] = self.cmd_boat_accelerate
            self.set_a.setText(str(self.cmd_boat_accelerate))

        if event.key() == Qt.Key_R:
            if(self.cmd_boat_gear<3):
                self.cmd_boat_gear+=1
            else:
                self.cmd_boat_gear=3
            sendmsg["ctl_cmd"]["gear"] = self.cmd_boat_gear
            self.set_g.setText(str(self.cmd_boat_gear))

        if event.key() == Qt.Key_V:
            if (self.cmd_boat_gear > 1):
                self.cmd_boat_gear -= 1
            else:
                self.cmd_boat_gear = 1
            sendmsg["ctl_cmd"]["gear"] = self.cmd_boat_gear
            self.set_g.setText(str(self.cmd_boat_gear))



        if event.key() == Qt.Key_O:
            # if(sendmsg["ctl_cmd"]["velocity"]<self.cmd_boat_velocity):
            #     if (sendmsg["ctl_cmd"]["velocity"]+self.cmd_boat_accelerate >= self.cmd_boat_velocity):
            #         sendmsg["ctl_cmd"]["velocity"]=self.cmd_boat_velocity
            #     else:
            #         sendmsg["ctl_cmd"]["velocity"] = sendmsg["ctl_cmd"]["velocity"]+self.cmd_boat_accelerate
            # else:
            #     if(sendmsg["ctl_cmd"]["velocity"]-self.cmd_boat_accelerate<=self.cmd_boat_velocity):
            #         sendmsg["ctl_cmd"]["velocity"] = self.cmd_boat_velocity
            #     else:
            #         sendmsg["ctl_cmd"]["velocity"] = sendmsg["ctl_cmd"]["velocity"]-self.cmd_boat_accelerate
            sendmsg["ctl_cmd"]["velocity"] = self.cmd_boat_velocity
            sendmsg["ctl_cmd"]["rotation"] = self.cmd_boat_rotation
            self.statusbar.showMessage("now send velocity:"+str(sendmsg["ctl_cmd"]["velocity"])+" \
            rotation:"+str(sendmsg["ctl_cmd"]["rotation"]),1000)

        if event.key() == Qt.Key_I:
            # if (sendmsg["ctl_cmd"]["velocity"] < self.cmd_boat_velocity):
            #     if (sendmsg["ctl_cmd"]["velocity"] + self.cmd_boat_accelerate >= self.cmd_boat_velocity):
            #         sendmsg["ctl_cmd"]["velocity"] = self.cmd_boat_velocity
            #     else:
            #         sendmsg["ctl_cmd"]["velocity"] = sendmsg["ctl_cmd"]["velocity"] + self.cmd_boat_accelerate
            # else:
            #     if (sendmsg["ctl_cmd"]["velocity"] - self.cmd_boat_accelerate <= self.cmd_boat_velocity):
            #         sendmsg["ctl_cmd"]["velocity"] = self.cmd_boat_velocity
            #     else:
            #         sendmsg["ctl_cmd"]["velocity"] = sendmsg["ctl_cmd"]["velocity"] - self.cmd_boat_accelerate
            sendmsg["ctl_cmd"]["velocity"] = self.cmd_boat_velocity
            sendmsg["ctl_cmd"]["rotation"] = 0
            self.statusbar.showMessage("now send velocity:" + str(sendmsg["ctl_cmd"]["velocity"]) + " \
                       rotation:" + str(sendmsg["ctl_cmd"]["rotation"]),1000)

        if event.key() == Qt.Key_U:
            # if (sendmsg["ctl_cmd"]["velocity"] < self.cmd_boat_velocity):
            #     if (sendmsg["ctl_cmd"]["velocity"] + self.cmd_boat_accelerate >= self.cmd_boat_velocity):
            #         sendmsg["ctl_cmd"]["velocity"] = self.cmd_boat_velocity
            #     else:
            #         sendmsg["ctl_cmd"]["velocity"] = sendmsg["ctl_cmd"]["velocity"] + self.cmd_boat_accelerate
            # else:
            #     if (sendmsg["ctl_cmd"]["velocity"] - self.cmd_boat_accelerate <= self.cmd_boat_velocity):
            #         sendmsg["ctl_cmd"]["velocity"] = self.cmd_boat_velocity
            #     else:
            #         sendmsg["ctl_cmd"]["velocity"] = sendmsg["ctl_cmd"]["velocity"] - self.cmd_boat_accelerate
            sendmsg["ctl_cmd"]["velocity"] = self.cmd_boat_velocity
            sendmsg["ctl_cmd"]["rotation"] = -self.cmd_boat_rotation
            self.statusbar.showMessage("now send velocity:" + str(sendmsg["ctl_cmd"]["velocity"]) + " \
                       rotation:" + str(sendmsg["ctl_cmd"]["rotation"]),1000)

        if event.key() == Qt.Key_L:
            sendmsg["ctl_cmd"]["velocity"] = 0
            sendmsg["ctl_cmd"]["rotation"] = self.cmd_boat_rotation
            self.statusbar.showMessage("now send velocity:" + str(sendmsg["ctl_cmd"]["velocity"]) + " \
                       rotation:" + str(sendmsg["ctl_cmd"]["rotation"]),1000)

        if event.key() == Qt.Key_K:
            sendmsg["ctl_cmd"]["velocity"] = 0
            sendmsg["ctl_cmd"]["rotation"] = 0
            self.statusbar.showMessage("now send velocity:" + str(sendmsg["ctl_cmd"]["velocity"]) + " \
                       rotation:" + str(sendmsg["ctl_cmd"]["rotation"]),1000)

        if event.key() == Qt.Key_J:
            sendmsg["ctl_cmd"]["velocity"] = 0
            sendmsg["ctl_cmd"]["rotation"] = -self.cmd_boat_rotation
            self.statusbar.showMessage("now send velocity:" + str(sendmsg["ctl_cmd"]["velocity"]) + " \
                       rotation:" + str(sendmsg["ctl_cmd"]["rotation"]),1000)

        if event.key() == Qt.Key_M:
            # if (sendmsg["ctl_cmd"]["velocity"] < -self.cmd_boat_velocity):
            #     if (sendmsg["ctl_cmd"]["velocity"] + self.cmd_boat_accelerate >= -self.cmd_boat_velocity):
            #         sendmsg["ctl_cmd"]["velocity"] = -self.cmd_boat_velocity
            #     else:
            #         sendmsg["ctl_cmd"]["velocity"] = sendmsg["ctl_cmd"]["velocity"] + self.cmd_boat_accelerate
            # else:
            #     if (sendmsg["ctl_cmd"]["velocity"] - self.cmd_boat_accelerate <= -self.cmd_boat_velocity):
            #         sendmsg["ctl_cmd"]["velocity"] = -self.cmd_boat_velocity
            #     else:
            #         sendmsg["ctl_cmd"]["velocity"] = sendmsg["ctl_cmd"]["velocity"] - self.cmd_boat_accelerate
            sendmsg["ctl_cmd"]["velocity"] = -self.cmd_boat_velocity
            sendmsg["ctl_cmd"]["rotation"] = self.cmd_boat_rotation
            self.statusbar.showMessage("now send velocity:" + str(sendmsg["ctl_cmd"]["velocity"]) + " \
                       rotation:" + str(sendmsg["ctl_cmd"]["rotation"]),1000)

        if event.key() == 44:
            # if (sendmsg["ctl_cmd"]["velocity"] < -self.cmd_boat_velocity):
            #     if (sendmsg["ctl_cmd"]["velocity"] + self.cmd_boat_accelerate >= -self.cmd_boat_velocity):
            #         sendmsg["ctl_cmd"]["velocity"] = -self.cmd_boat_velocity
            #     else:
            #         sendmsg["ctl_cmd"]["velocity"] = sendmsg["ctl_cmd"]["velocity"] + self.cmd_boat_accelerate
            # else:
            #     if (sendmsg["ctl_cmd"]["velocity"] - self.cmd_boat_accelerate <= -self.cmd_boat_velocity):
            #         sendmsg["ctl_cmd"]["velocity"] = -self.cmd_boat_velocity
            #     else:
            #         sendmsg["ctl_cmd"]["velocity"] = sendmsg["ctl_cmd"]["velocity"] - self.cmd_boat_accelerate
            sendmsg["ctl_cmd"]["velocity"] = -self.cmd_boat_velocity
            sendmsg["ctl_cmd"]["rotation"] =0
            self.statusbar.showMessage("now send velocity:" + str(sendmsg["ctl_cmd"]["velocity"]) + " \
                       rotation:" + str(sendmsg["ctl_cmd"]["rotation"]),1000)

        if event.key() == 46:
            # if (sendmsg["ctl_cmd"]["velocity"] < -self.cmd_boat_velocity):
            #     if (sendmsg["ctl_cmd"]["velocity"] + self.cmd_boat_accelerate >= -self.cmd_boat_velocity):
            #         sendmsg["ctl_cmd"]["velocity"] = -self.cmd_boat_velocity
            #     else:
            #         sendmsg["ctl_cmd"]["velocity"] = sendmsg["ctl_cmd"]["velocity"] + self.cmd_boat_accelerate
            # else:
            #     if (sendmsg["ctl_cmd"]["velocity"] - self.cmd_boat_accelerate <= -self.cmd_boat_velocity):
            #         sendmsg["ctl_cmd"]["velocity"] = -self.cmd_boat_velocity
            #     else:
            #         sendmsg["ctl_cmd"]["velocity"] = sendmsg["ctl_cmd"]["velocity"] - self.cmd_boat_accelerate
            sendmsg["ctl_cmd"]["velocity"] = -self.cmd_boat_velocity
            sendmsg["ctl_cmd"]["rotation"] = -self.cmd_boat_rotation
            self.statusbar.showMessage("now send velocity:" + str(sendmsg["ctl_cmd"]["velocity"]) + " \
                       rotation:" + str(sendmsg["ctl_cmd"]["rotation"]),1000)

    def keyReleaseEvent(self,event):

        if event.key() == Qt.Key_U:
            if event.isAutoRepeat():
                return
            sendmsg["ctl_cmd"]["velocity"] = 0
            sendmsg["ctl_cmd"]["rotation"] = 0
            self.statusbar.showMessage("now send velocity:" + str(sendmsg["ctl_cmd"]["velocity"]) + " \
                                   rotation:" + str(sendmsg["ctl_cmd"]["rotation"]), 1000)

        if event.key() == Qt.Key_I:
            if event.isAutoRepeat():
                return
            sendmsg["ctl_cmd"]["velocity"] = 0
            sendmsg["ctl_cmd"]["rotation"] = 0
            self.statusbar.showMessage("now send velocity:" + str(sendmsg["ctl_cmd"]["velocity"]) + " \
                                   rotation:" + str(sendmsg["ctl_cmd"]["rotation"]), 1000)

        if event.key() == Qt.Key_O:
            if event.isAutoRepeat():
                return
            sendmsg["ctl_cmd"]["velocity"] = 0
            sendmsg["ctl_cmd"]["rotation"] = 0
            self.statusbar.showMessage("now send velocity:" + str(sendmsg["ctl_cmd"]["velocity"]) + " \
                                   rotation:" + str(sendmsg["ctl_cmd"]["rotation"]), 1000)

        if event.key() == Qt.Key_J:
            if event.isAutoRepeat():
                return
            sendmsg["ctl_cmd"]["velocity"] = 0
            sendmsg["ctl_cmd"]["rotation"] = 0
            self.statusbar.showMessage("now send velocity:" + str(sendmsg["ctl_cmd"]["velocity"]) + " \
                                   rotation:" + str(sendmsg["ctl_cmd"]["rotation"]), 1000)

        if event.key() == Qt.Key_K:
            if event.isAutoRepeat():
                return
            sendmsg["ctl_cmd"]["velocity"] = 0
            sendmsg["ctl_cmd"]["rotation"] = 0
            self.statusbar.showMessage("now send velocity:" + str(sendmsg["ctl_cmd"]["velocity"]) + " \
                                   rotation:" + str(sendmsg["ctl_cmd"]["rotation"]), 1000)

        if event.key() == Qt.Key_L:
            if event.isAutoRepeat():
                return
            sendmsg["ctl_cmd"]["velocity"] = 0
            sendmsg["ctl_cmd"]["rotation"] = 0
            self.statusbar.showMessage("now send velocity:" + str(sendmsg["ctl_cmd"]["velocity"]) + " \
                                   rotation:" + str(sendmsg["ctl_cmd"]["rotation"]), 1000)

        if event.key() == Qt.Key_M:
            if event.isAutoRepeat():
                return
            sendmsg["ctl_cmd"]["velocity"] = 0
            sendmsg["ctl_cmd"]["rotation"] = 0
            self.statusbar.showMessage("now send velocity:" + str(sendmsg["ctl_cmd"]["velocity"]) + " \
                                   rotation:" + str(sendmsg["ctl_cmd"]["rotation"]), 1000)

        if event.key() == 44:
            if event.isAutoRepeat():
                return
            sendmsg["ctl_cmd"]["velocity"] = 0
            sendmsg["ctl_cmd"]["rotation"] = 0
            self.statusbar.showMessage("now send velocity:" + str(sendmsg["ctl_cmd"]["velocity"]) + " \
                                   rotation:" + str(sendmsg["ctl_cmd"]["rotation"]), 1000)

        if event.key() == 46:
            if event.isAutoRepeat():
                return
            sendmsg["ctl_cmd"]["velocity"] = 0
            sendmsg["ctl_cmd"]["rotation"] = 0
            self.statusbar.showMessage("now send velocity:" + str(sendmsg["ctl_cmd"]["velocity"]) + " \
                                   rotation:" + str(sendmsg["ctl_cmd"]["rotation"]), 1000)

    def c_wifi(self):
        self.statusbar.showMessage("config wifi port")
        self.configWifi = config_WIF(self.demo)
        self.configWifi.show()

    def c_serial(self):
        self.statusbar.showMessage("config serial port")
        self.configSerial = config_SERIAL(self.demo)
        self.configSerial.show()

    def start_over(self):
        if(self.demo.com_status==0):
            print("open start")
            if(self.set_communication_mode.currentText()=="serial"):
                if(self.demo.open_serial()):
                    self.statusbar.showMessage("open serial:successful")
                    self.open_close.setText("close")
                    self.set_communication_mode.setDisabled(1)
                else:
                    self.statusbar.showMessage("open serial:failed")
            else:
                if(self.demo.open_wifi()):
                    self.statusbar.showMessage("open wifi:successful")
                    self.open_close.setText("close")
                    self.set_communication_mode.setDisabled(1)
                else:
                    self.statusbar.showMessage("open wifi:failed")

        else:
            print("close start")
            if (self.set_communication_mode.currentText() == "serial"):
                if(self.demo.close_serial()):
                    self.statusbar.showMessage("close serial:successful")
                    self.open_close.setText("open")
                    self.set_communication_mode.setEnabled(1)

                else:
                    self.statusbar.showMessage("close serial:failed")
            else:
                if(self.demo.close_wifi()):
                    self.statusbar.showMessage("close wifi:successful")
                    self.open_close.setText("open")
                    self.set_communication_mode.setEnabled(1)

                else:
                    self.statusbar.showMessage("close wifi:failed")

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        if(self.demo.com_status==1):
            a0.ignore()
            self.statusbar.showMessage("请先关闭wifi/serial,再退出")
        else:
            pass


def main():
    app = QApplication(sys.argv)
    ex = key_ctl()
    ex.show()
    sys.exit(app.exec_())


if __name__=="__main__":
    main()


