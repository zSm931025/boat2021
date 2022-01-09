from boat_api import *
from track_pid import *

gX = 0
gY = 0
gSeta = 0
gSeta_speed = 0


if __name__=="__main__":
    # load boat api
    path = [[[-1,-4],[-0.5,3]],[[-0.5,3],[5,5]]]
    boat_ = BOAT_()
    track_ = TRACK_()
    print(boat_.detect_serial())
    boat_.open_serial()

    track_thread = threading.Thread(target=track_.track_process,args=(path,))
    track_thread.start()
    print("Jj")
    base = None
    base_angle = -np.pi/7.0
    while(True):
        # get x y in customer map
        if recvmsg["gps"]["lon_degree"]!= None:
            longitude_ = recvmsg["gps"]["lon_degree"] + recvmsg["gps"]["lon_cent"]/ 60.0 + recvmsg["gps"]["lon_second"]/ 3600.0
            latitude_ = recvmsg["gps"]["lat_degree"] + recvmsg["gps"]["lat_cent"] / 60.0 + recvmsg["gps"]["lat_second"] / 3600.0
            if base==None:
                base = [longitude_, latitude_]
            else:
                if recvmsg["eul"]["z"] != None and recvmsg["gro"]["z"] != None:
                    gX,gY = calculate_coordinate_of_point_based_on_my_coordinate([longitude_, latitude_],base,base_angle)
                    gSeta = recvmsg["eul"]["z"]-base_angle
                    gSeta_speed = recvmsg["gro"]["z"]
                    #draw boat


    # load track algorithm