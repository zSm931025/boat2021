import numpy as np
R = 6378137



def calculate_the_distance_two_point(point1,point2):
    """
    :param point1: [lon1,lat1]
    :param point2: [lon2,lat2]
    :return:
    """
    lat1 = point1[1]*np.pi/180
    lon1 = point1[0]*np.pi/180
    lat2 = point2[1]*np.pi/180
    lon2 = point2[0]*np.pi/180
    return 2*R*np.arcsin(np.sqrt(np.sin((lat1-lat2)/2)**2+np.cos(lat1)*np.cos(lat2)*np.sin((lon1-lon2)/2)**2))

def calculate_coordinate_of_point2_based_on_point1(point1,point2):
    """

    :param point1: take the point1 as the origin, and X axis point to the east with y axis pointing to the North
    :param point2: 
    :return: 
    """
    lon1 = point1[0]
    lat1 = point1[1]
    lon2 = point2[0]
    lat2 = point2[1]
    l_x = calculate_the_distance_two_point([lon1,lat1],[lon2,lat1])
    if lon1<lon2:
        x = l_x
    else:
        x = -l_x
    l_y = calculate_the_distance_two_point([lon1,lat1],[lon1,lat2])
    if lat1<lat2:
        y = l_y
    else:
        y = -l_y
    return x,y

def calculate_the_angle_two_point(point1,point2):
    x,y  = calculate_coordinate_of_point2_based_on_point1(point1,point2) # 以point1 为原点，y 轴指向正北方向，x 轴指向正东方向
    if x==0 and y>0:
        seta = np.pi/2
    elif x==0 and y<0:
        seta = 3*np.pi/2
    elif y==0 and x>0:
        seta = 0
    elif y==0 and x<0:
        seta = np.pi
    elif (x<0 and y<0) or (x<0 and y>0):
        seta = np.pi+np.arctan(y/x)
    else:
        seta = np.arctan(y/x)
    return seta

def calculate_coordinate_of_point_based_on_my_coordinate(aim_point,center_point,angle_realated_NED):
    """
    两个坐标系的夹角，逆时针为正
    :param aim_point:
    :param center_point:
    :param angle_realated_NED:
    :return:
    """
    #print(angle_realated_NED*180/np.pi)
    x,y = calculate_coordinate_of_point2_based_on_point1(center_point,aim_point)
    x_new = x*np.cos(angle_realated_NED)+y*np.sin(angle_realated_NED)
    y_new = y*np.cos(angle_realated_NED)-x*np.sin(angle_realated_NED)
    return x_new,y_new