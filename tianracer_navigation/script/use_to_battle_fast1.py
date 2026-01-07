#! /usr/bin/env python3
#coding=utf-8

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from visualization_msgs.msg import Marker
import copy
import os
robot_name = os.getenv("TIANRACER_NAME", "tianracer")

global last_angle, last_max_dir_index
last_angle = 0
last_max_dir_index = 0
DIR_DETECT_THRESHOLD = 2.5 # 方向探测距离,用于方向判断
OBS_DETECT_THRESHOLD = 5.0 # 障碍探测距离,用于障碍物判断

MAX_SPEED_RATE = 2.0

THRESHOLD_obs = 0.5
THRESHOLD_TURN = 0.5 # 转弯阈值
START_ANGLE = -60
END_ANGLE = 60
GO_STARIGHT = 0 # 是否进入直道路段
TRANSITION = 0 # 是否进入过渡路段
last_in_normol = False   # 记录上一次循环是否进入隐藏款
last_in_straight = False  # 记录上一次循环是否进入直道路段
speed_rate = 1.0         # 速度比例因子，初始为1.0
straight_cnt = 0
MIN_OBS_SPEED = 2.0
Follow = False
turn_rate = 1.0 # 转向比例因子
P = 1.1
D = 0.2
dynamic_obs = False
chaoche = False
def publish_arrow_marker(max_dir_index):

    marker = Marker()
    marker.header.frame_id = robot_name + "/laser"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "direction_arrow"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    
    # 起点 (原点)
    marker.points = []
    from geometry_msgs.msg import Point
    p0 = Point(0, 0, 0)
    # 计算箭头终点，假定箭头长度为1
    angle_rad = math.radians(max_dir_index)
    p1 = Point(math.sin(angle_rad), math.cos(angle_rad), 0)
    marker.points.append(p0)
    marker.points.append(p1)

    # 箭头外观
    marker.scale.x = 0.1  
    marker.scale.y = 0.1  
    marker.scale.z = 0.2  
    marker.color.a = 1.0
    marker.color.r = 0.1
    marker.color.g = 1.0  # 绿色
    marker.color.b = 0.1

    # 持续时间
    marker.lifetime = rospy.Duration(0.1) 

    # 发布一次（循环发布请用 while）
    marker_pub.publish(marker)

def get_dis(data, angle, deg=True, return_inten = True):
    if deg:
        angle = np.deg2rad(angle)
    dis = 0
    intensities = None
    temp = int((angle - data.angle_min) / data.angle_increment)

    data_tmp = data.ranges[temp-2:temp+2]
    inten_tmp = data.intensities[temp-2:temp+2]
    data_tmp = np.sort(data_tmp)

    inten_tmp = np.sort(inten_tmp)

    dis = data_tmp[2]
    intensities = inten_tmp[2]
    if return_inten:
      return dis,intensities
    else:
      return dis


def get_range(data, start_angle, end_engle,return_inten = False):
    all_dis = []
    all_inten = []
    for angle in range(start_angle,end_engle):
      tmp = get_dis(data, angle, return_inten = return_inten)
      all_dis.append(tmp[0])
      all_inten.append(tmp[1])
    if return_inten:
      return all_dis,all_inten
    else:
      return all_dis


def fill_zeros_with_neighbors(data):
    """
    Replace zero values in the list with the nearest non-zero neighbor:
    - Prefer the left (previous) value if available
    - Otherwise, use the right (next) value
    
    Args:
        data (list): Input list of floats with possible zero values.
    
    Returns:
        list: A list with zero values replaced by neighbor values.
    """
    result = data.copy()
    n = len(result)

    for i in range(n):
        if result[i] == 0:
            # Try to use the left neighbor
            left = next((result[j] for j in range(i - 1, -1, -1) if result[j] != 0), None)
            if left is not None:
                result[i] = left
                continue

            # Otherwise try to use the right neighbor
            right = next((result[j] for j in range(i + 1, n) if result[j] != 0), None)
            if right is not None:
                result[i] = right
                continue

            # If no non-zero neighbors found (all zeros), keep 0
            result[i] = 0
            print("Warning: No non-zero neighbors found for index", i)
    
    return result

def filter_obstacles_by_variance(Left_obs_orig, dis_90, variance_threshold=1.0):
    """
    Filters the obstacle regions based on the variance of the distance measurements.
    
    Args:
        Left_obs_orig (list): List of indices representing the start and end of obstacle regions.
        dis_90 (list or np.array): Laser scan distance data.
        variance_threshold (float): The threshold for the variance. Regions with variance below this threshold are considered valid.
    
    Returns:
        list: A list containing the filtered obstacle regions.
    """
    Left_obs = []  # Initialize list to store valid obstacle regions
    
    if len(Left_obs_orig) > 0:
        for i in range(0, int(len(Left_obs_orig) / 2), 1):
            # Get the middle value of the current obstacle range
            dis_obs_middle = dis_90[int((Left_obs_orig[2 * i] + Left_obs_orig[2 * i + 1]) / 2)]
            
            # Extract the range of values for the current obstacle
            obstacle_range = dis_90[Left_obs_orig[2 * i]: Left_obs_orig[2 * i + 1]]
            
            # Compute the variance of the obstacle range
            dis_obs_var = np.var(obstacle_range)

            # If the variance is smaller than the threshold, consider it a valid obstacle range
            # if dis_obs_var < variance_threshold:
            print("in filter_obstacles_by_variance,方差：",dis_obs_var)
            Left_obs.append(Left_obs_orig[2 * i])
            Left_obs.append(Left_obs_orig[2 * i + 1])
    
    return Left_obs


def filter_anomalous_values(data, max_distance=4, angle_range=2):
    # Convert data to numpy array for easier manipulation
    data = np.array(data)
    
    # Loop through the data, starting from the second value to the second-last
    for i in range(1, len(data) - 1):
        # Check if the current value is an outlier based on the surrounding values
        if data[i] != max_distance:
            # Check if both neighboring values (within the specified angle range) are equal to max_distance (4)
            if all(data[j] == max_distance for j in range(i - angle_range, i + angle_range + 1) if 0 <= j < len(data)):
                # Replace the outlier with the average of its neighbors
                data[i] = (data[i-1] + data[i+1]) / 2
    
    # Return the filtered list
    return data.tolist()


def filter_small_obstacles(Left_obs, min_obstacle_size=2):
    # Mark small obstacles by checking the distance between start and end points
    for i in range(int(len(Left_obs) / 2)):
        if abs(Left_obs[2 * i] - Left_obs[2 * i + 1]) <= min_obstacle_size:
            Left_obs[2 * i] = -1  # Mark start of small obstacle
            Left_obs[2 * i + 1] = -1  # Mark end of small obstacle
    # Remove the small obstacles from the list
    Left_obs_temp = Left_obs
    Left_obs = [x for x in Left_obs_temp if x != -1]
    
    return Left_obs


  
def pub_scan(dis_90,msgs):
    scan_msg = LaserScan()

    # Set the header information
    scan_msg.header = msgs.header

    # Set the angle information for the scan
    scan_msg.angle_min =  np.pi / 2  # 90 degrees
    scan_msg.angle_max = -np.pi / 2    # 90 degrees
    scan_msg.angle_increment =  -np.pi/180  # 1 degree in radians

    # Set the distance measurements (dis_90)
    scan_msg.ranges = dis_90  # This should be a list of 180 distance values
    scan_msg.intensities = []  # Optional, if you don't have intensity data
    scan_msg.range_max = 100

    # Publish the LaserScan message
    scan_pub.publish(scan_msg)

def detect_outliers(data, threshold=1.5):
  """
  Detect outliers using the interquartile range (IQR) method.
  Returns indices of outliers in the input data.
  """
  q1 = np.percentile(data, 25)
  q3 = np.percentile(data, 75)
  iqr = q3 - q1
  lower_bound = q1 - threshold * iqr
  upper_bound = q3 + threshold * iqr
  outlier_indices = np.where((data < lower_bound) | (data > upper_bound))[0]
  return outlier_indices

def find_non_obs_values(inten_list, obs_start=73, obs_end=116):
  return [value for idx, value in enumerate(inten_list) 
          if not (obs_start <= idx <= obs_end)]

def DynamicObastcle(dis_list,inten_list, max_dir_num, obs):
  max_range = [max_dir_num[0], max_dir_num[-1]]
  obs_range = [max_dir_num[1], max_dir_num[2]]
  range_list = inten_list[max_range[0]:max_range[1]]
  obs_intensity = inten_list[obs_range[0]:obs_range[1]]
  average_obs_intensity = np.mean(obs_intensity)
  average_intensity = np.mean(range_list)
  abs_tmp = abs(average_obs_intensity - average_intensity)
  if abs_tmp > 5:
    print('detect dynamic obs abs_tmpabs_tmpabs_tmpabs_tmp!!!!',abs_tmp)
    return True
  else:
    return False 


def middle_line_callback(data):
    ####参数初始化####
    global last_angle
    global last_max_dir_index
    global GO_STARIGHT, TRANSITION, last_in_normol,Follow ,speed_rate,straight_cnt ,turn_rate,chaoche ,last_in_straight ,P, D, dynamic_obs
    dynamic_obs = False
    chaoche = False
    Follow = False

    D = 0.2
    print("###########################################################")
    dis_90,inten_90 = get_range(data, -89,91, True)# 前方180度雷达数据 从左边到右边
    dis_90 = dis_90[::-1]
    inten_90 = inten_90[::-1]
    dis_obs_90 = copy.deepcopy(dis_90)
    lenth_dis = len(dis_90)
    
    #方向变量
    left = 0
    right = 0
    Left_obs_orig = []
    Left_obs = []
    max_dis_num = []
    max_dir_num = []
    max_dir_num_obs = []
    max_dis = 0 
    max_dir_range = 0
    max_dis_index = 0
    max_dir_index = 0
    pub_scan(dis_90,data)
    #去除0值，传感器噪声
    
    dis_90 = fill_zeros_with_neighbors(dis_90) #去除0值
    inten_90 = fill_zeros_with_neighbors(inten_90) #去除0值
    dis_obs_90 = fill_zeros_with_neighbors(dis_obs_90) #去除0值
    # print('dis_90_',dis_90[75:105])
    dis_90_copy = tuple(dis_90)
    # 找到最大距离的方向，并且对超过最大探测距离的角度进行限制幅度
    for i in range(0,lenth_dis,1):
        if dis_90[i]>max_dis and i>20 and i<160:  #以激光雷达坐标从左20到右160度
           max_dis = dis_90[i]
           max_dis_index = i 
        if  dis_90[i] > DIR_DETECT_THRESHOLD:
          dis_90[i] = DIR_DETECT_THRESHOLD 
        if dis_obs_90[i] > OBS_DETECT_THRESHOLD:
          dis_obs_90[i] = OBS_DETECT_THRESHOLD 
    ##去除异常点，连续多个最大距离范围中有零星近值
    dis_90 = filter_anomalous_values(dis_90, max_distance = DIR_DETECT_THRESHOLD, angle_range=2)
    dis_obs_90 = filter_anomalous_values(dis_obs_90, max_distance = OBS_DETECT_THRESHOLD, angle_range=2)
    #以下代码为了判断前方方向（根据最远距离的角度下标进行判断
    if max_dis_index < 89:
      left = 1
    else:
      right = 1
    ############################、
    #障碍物数据获取+滤波
    for i in range(len(dis_90)):
      if dis_90[i]==0:
        print("zero is ???????",i)

    for i in range(0,lenth_dis-2,1):
       if dis_obs_90[i]-dis_obs_90[i+1] > THRESHOLD_obs and len(Left_obs_orig)%2 ==0:
         #print("进点是",i)
         Left_obs_orig.append(i+1)
       elif dis_obs_90[i+1]-dis_obs_90[i] > THRESHOLD_obs and len(Left_obs_orig)%2 ==1:
         #print("出点是",i)
         Left_obs_orig.append(i)
    #障碍范围总是成对出现进行一次滤波
    if len(Left_obs_orig)%2 == 1:
      print('error!!!!!!!!!!!!')
      Left_obs_orig.pop()
    
    left_obs_copy = copy.deepcopy(Left_obs_orig)
    # #滤除小障碍物
    Left_obs = filter_small_obstacles(Left_obs_orig,min_obstacle_size = 2)
    #障碍范围总是成对出现进行二次滤波 计算所有障碍区间的方（区间所有数据与区间最中间值方差）  小于variance_threshold1才看作障碍
    Left_obs = filter_obstacles_by_variance(Left_obs, dis_obs_90, variance_threshold=1.0)
    
    
    
    #以下代码为 障碍膨胀 将原来的障碍区间进行膨胀再找最大可行方向 留一定的余量
    if len(Left_obs)>0 :#有障碍才进入
      for i in range(0,int(len(Left_obs)/2),1):
        obs_middle = dis_obs_90[int((Left_obs[2*i]+Left_obs[2*i+1])/2)]
        for j in range(int(max(Left_obs[2*i]-min((Left_obs[2*i+1]-Left_obs[2*i])/2*(4-obs_middle),10),0))\
          ,int(min(Left_obs[2*i+1]+min((Left_obs[2*i+1]-Left_obs[2*i])/2*(4-obs_middle),10),lenth_dis-1)),1):
            dis_obs_90[j] = obs_middle
        Left_obs[2*i]=int(max(Left_obs[2*i]-min((Left_obs[2*i+1]-Left_obs[2*i])/2*(4-obs_middle),10),0))
        Left_obs[2*i+1]=int(min(Left_obs[2*i+1]+min((Left_obs[2*i+1]-Left_obs[2*i])/2*(4-obs_middle),10),lenth_dis-1))
      print("有障碍物，障碍物是",Left_obs)
    else:
      print("没有障碍物")
    #以下代码为找有障碍
    if len(Left_obs)>0 :
      for i in range(0,int(len(Left_obs)/2)+1,1):
        if i == 0:
          for j in range(Left_obs[0]-1,0,-1):
            if dis_obs_90[j]<=dis_obs_90[Left_obs[0]+1]:
              
              max_dis_num.append(j)
              max_dis_num.append(Left_obs[0]+1)
              break
          if len(max_dis_num)==0:
            for j in range(0,Left_obs[0]-1,1):
              if dis_obs_90[j]>=dis_obs_90[Left_obs[0]+1]:
                max_dis_num.append(j)
                max_dis_num.append(Left_obs[0]+1)
                break
        elif i < int(len(Left_obs)/2):
          max_dis_num.append(Left_obs[2*i-1])
          max_dis_num.append(Left_obs[2*i])
        elif i == int(len(Left_obs)/2):
          for j in range(Left_obs[2*i-1]+1,lenth_dis-1,1):
            if dis_obs_90[j]<=dis_obs_90[Left_obs[2*i-1]-1]:
              max_dis_num.append(Left_obs[2*i-1]-1)
              max_dis_num.append(j)
              break

      # print("max_dis_num is",max_dis_num)
      # if len(max_dis_num) ==6:
      #   print(11)
      max_dis_val = 0
      max_dis_index_temp = max_dis_index
      #以下代码基于上面代码块计算的障碍间隙数组进行处理  计算目标方向
      for i in range(0,int(len(max_dis_num)/2),1):
        if max_dis_val < max_dis_num[2*i+1] - max_dis_num[2*i] :
          max_dis_val = max_dis_num[2*i+1] - max_dis_num[2*i]
          max_dis_index = (max_dis_num[2*i+1] + max_dis_num[2*i])/2
        if left == 1 and max_dis_index < 90 and dis_obs_90[0]<dis_obs_90[lenth_dis-1]-1:
          max_dis_index = max_dis_index+5*abs(dis_obs_90[lenth_dis-1]-dis_obs_90[0])#作补偿 因为目标方向和最大距离方向不一致  可以进行限幅
        elif right == 1 and max_dis_index > 90 and dis_obs_90[0]-1>dis_obs_90[lenth_dis-1]:
          max_dis_index = max_dis_index-5*abs(dis_obs_90[lenth_dis-1]-dis_obs_90[0])#作补偿 因为目标方向和最大距离方向不一致  可以进行限幅
      #print("最远方向",max_dis_index)
      #print("有障碍 max dis index",max_dis_index)

      #比中间
      print("max_dis_index_temp",max_dis_index_temp)
      if len(Left_obs) ==2 :#一个障碍:
        if max_dis_index_temp >=89:#右转
          print("turn right")
          if Left_obs[0] >=89:
            max_dis_index = int((89+Left_obs[0])/2)
          elif Left_obs[1] <=89:
            max_dis_index = max_dis_index_temp
          else:#正前方情况，不太合理
            max_dis_index = max_dis_index_temp
        else:#左转
          #print("turn left")
          if Left_obs[0] >=89:
            print("1")
            max_dis_index = max_dis_index_temp
          elif Left_obs[1] <=89:
            print("2")
            max_dis_index = int((89+Left_obs[1])/2)
          else:
            print("3")
            max_dis_index = max_dis_index_temp
      if len(Left_obs) ==4: #两个障碍
        middle_temp = int((Left_obs[1]+Left_obs[2])/2)
        if max_dis_index_temp >=89:#右转
          if Left_obs[0] >= 89:
            max_dis_index = int((89+Left_obs[0])/2)
          elif Left_obs[1] <= 89 and middle_temp >89:
            #max_dis_index = int((Left_obs[1]+Left_obs[2])/2)
            max_dis_index = int((Left_obs[1]+max_dis_index_temp)/2)
          elif middle_temp <= 89 and Left_obs[2] >89:
            max_dis_index = middle_temp
          else:
            max_dis_index = max_dis_index_temp
        else:
          print("turn left")
          if Left_obs[0] > 89:
            max_dis_index = max_dis_index_temp
          elif Left_obs[0] <= 89 and middle_temp >89:
            max_dis_index = middle_temp
          elif middle_temp <= 89 and Left_obs[3] >89:
            max_dis_index = middle_temp#int((Left_obs[2]+max_dis_index_temp)/2)
          else:
            max_dis_index = int((Left_obs[3] + 89)/2)
      #print("after my compute",max_dis_index)
      # print("after compute max_dis_index is",max_dis_index)

    #以下代码寻找最大距离（阈值距离）区域
    for i in range(0,lenth_dis-2,1):
      if dis_90[i]<DIR_DETECT_THRESHOLD and dis_90[i+1] == DIR_DETECT_THRESHOLD and len(max_dir_num)%2==0:
        max_dir_num.append(i+1)
      elif dis_90[i]==DIR_DETECT_THRESHOLD and dis_90[i+1] < DIR_DETECT_THRESHOLD and len(max_dir_num)%2==1:
        max_dir_num.append(i)

    if len(max_dir_num) % 2 == 1 and len(max_dir_num)!= 1:
      rospy.logerr("出现单个不封闭区域，请检查障碍物检测逻辑")
      for i in range(0,lenth_dis-2,1):
        rospy.logerr("dis_90[%d] is %f" % (i, dis_90[i]))
    


    ##转弯阶段
    if len(max_dir_num) == 1:
      if max_dir_num[0] < 90:     
        max_dir_index = int((max_dir_num[0])/2)
        print("\033[32m转弯阶段1左转，最大距离朝向: %s\033[0m" % (max_dir_index - lenth_dis/2))
      elif max_dir_num[0] > 90:
        max_dir_index = int((max_dir_num[0]+lenth_dis-2)/2)
        print("\033[32m转弯阶段1右转，最大距离朝向: %s\033[0m" % (max_dir_index - lenth_dis/2))
      GO_STARIGHT = 0
    #如果找到最大距离（阈距离）区域   取最大距离（阈值距离）区域中间的下标作为前进方向
      
    if len(max_dir_num)==2:
      print("找到 %d 个最大距离区域，区域大小 %d" % ((int(len(max_dir_num)/2)), max_dir_num[1]-max_dir_num[0]))
      # if max_dir_num[1]-max_dir_num[0]>4:
      max_dir_index = int((max_dir_num[0]+max_dir_num[1])/2)
      max_dir_range = max_dir_num[1]-max_dir_num[0]
      # else:
      #   rospy.logerr("最大距离区域过小，可能是噪声或障碍物误判，跳过该区域")


    if len(max_dir_num)>2:
      print("有多个最大距离区域，障碍物个数为:",len(Left_obs)/2)

      if len(Left_obs) > 0:
        dynamic_obs = DynamicObastcle(dis_list=dis_90,inten_list = inten_90, max_dir_num = max_dir_num, obs = Left_obs)
      
      cand_space = [] #候选区域
      cand_dirs = [] #候选方向
      for i in range(0,int(len(max_dir_num)/2),1):
        cand_space.append(max_dir_num[2*(i)+1]-max_dir_num[2*(i)])
        cand_dirs.append((max_dir_num[2*(i)+1]+max_dir_num[2*(i)])/2)
      cand_dir_id = np.where(np.array(cand_space)>18)[0] #选取大于18的区域的id
      if len(cand_dir_id)!=0:
        selected_dirs = np.array(cand_dirs)[cand_dir_id].tolist()
        max_dir_idx = np.argmin(selected_dirs)
        selected_ranges = np.array(cand_space)[cand_dir_id].tolist()
        max_dir_index = selected_dirs[max_dir_idx]
        max_dir_range = selected_ranges[max_dir_idx]
        cand_dir_chaoche_idx = np.where(np.array(cand_space)>30)[0]
        if dynamic_obs:
          if cand_dir_chaoche_idx.size:
            chaoche = True
          else:
            Follow = True
      else:
        cand_dir_id = np.argmax(np.array(cand_space))
        max_dir_index = cand_dirs[cand_dir_id]
        max_dir_range = cand_space[cand_dir_id]
        if dynamic_obs:
          max_dir_index = int((max_dir_num[1] + max_dir_num[2]) / 2)
          max_dir_range = max_dir_num[-1] - max_dir_num[0]
          Follow = True

      if max_dir_index < 90:
        max_dir_index -= 2
      else:
        max_dir_index += 2

    print(max_dir_num,P)
    if max_dir_index >= 75 and max_dir_index <= 105:
        mean_straight = np.mean(dis_90_copy[80:100])
        # print('mean_straight',mean_straight,dis_90_copy[75:105])
        GO_STARIGHT = 1
        TRANSITION = 0      
        if last_in_straight and max_dir_range > 20:
          print("在直道路段保持加速，最大距离朝向:", (max_dir_index - len(dis_90) / 2)," 加速空间范围：",max_dir_range)  
          speed_rate *= 1.05   # 连续进入，递增
          if mean_straight > 11 and len(Left_obs) == 0:
              limit_rate = 3.0
          elif mean_straight > 8 and len(Left_obs) == 0:
              limit_rate = 2.5
          elif mean_straight > 7 and len(Left_obs) == 0:
              limit_rate = 2.2
          else:
              limit_rate = 2.0
          if speed_rate > limit_rate:
              speed_rate = limit_rate
          
          print('速度增益', speed_rate)
        else:
            speed_rate = 1.1    # 第一次进入，直接1.1
            
        last_in_straight = True    
    elif max_dir_index < 75 and max_dir_index > 0:
      print("\033[32m转弯阶段2左转，最大距离朝向: %s\033[0m" % (max_dir_index - lenth_dis/2))
      P = 1.5
      speed_rate = 1.0
      turn_rate = 0.8
      last_in_straight = False
    elif max_dir_index > 105:
      print("\033[32m转弯阶段2右转，最大距离朝向: %s\033[0m" % (max_dir_index - lenth_dis/2))
      P = 1.5
      speed_rate = 1.0
      turn_rate = 0.8
      last_in_straight = False
    
    #过渡路段或转弯路段
    normol = 1
    if len(max_dir_num) == 0:
      if GO_STARIGHT == 1 or TRANSITION == 1:
        for i in range(0,lenth_dis-2,1):
          if dis_90[i+1] - dis_90[i] > THRESHOLD_TURN:
            max_dir_index = int((i+1+len(dis_90)/2)/2)
            rospy.logwarn("进入过渡路段，前方左转，最大距离朝向: %f", (max_dir_index-len(dis_90)/2))
            P = 0.8
            normol = 0
          elif dis_90[i] - dis_90[i+1] > THRESHOLD_TURN:
            max_dir_index = int((i+len(dis_90)/2)/2)
            rospy.logwarn("进入过渡路段，前方右转，最大距离朝向: %f", (max_dir_index-len(dis_90)/2))
            P = 0.8
            normol = 0
        if normol == 1:
            max_dir_index = last_max_dir_index
            print("\033[38;5;208m隐藏款，无法判断方向，保持上一次动作并减速，最大距离朝向: %f\033[0m" % (max_dir_index-len(dis_90)/2))
            
            if last_in_normol:
                speed_rate *= 0.9         # 连续进入，递减
                turn_rate *= 1.2
                if speed_rate < 0.5:
                    speed_rate = 0.5
                if turn_rate > 2.5:
                    turn_rate = 2.5
            else:
                speed_rate = 0.9          # 第一次进入，先减一次
                turn_rate = 1.2
            last_in_normol = True
        else:
            speed_rate = 1.0              # 没有进入则恢复
            turn_rate = 1.0
            last_in_normol = False

        TRANSITION = 1
        GO_STARIGHT = 0

    #print("dis_90[0] is",dis_90[0])    
    #print("dis_90[lenth_dis-1] is",dis_90[lenth_dis-1])
    dis_90[0] = dis_90[0] + 0.00001
    print("视野中最大距离是",max_dis)
    # print("max dis index used for compute is",max_dis_index)
    # print("dis_90[0] and dis_90[len-1] is",[dis_90[0],dis_90[lenth_dis-1]])
    dis_90[lenth_dis-1] = dis_90[lenth_dis-1] +0.00001
    
    if max_dir_index != 0:
      term1 = -max(math.exp(-max_dis/DIR_DETECT_THRESHOLD),0.7)*(max_dir_index-90)/360 *math.pi
      term2 = (dis_90[0]-dis_90[lenth_dis-1])/(dis_90[0]+dis_90[lenth_dis-1])
      print(f'term1:{term1}, term2:{term2}')
      if dis_90[0]/dis_90[lenth_dis-1]>3 or dis_90[lenth_dis-1]/dis_90[0]>3:
        D = 0.5
        print(f'边界！！！！！！！！！！！！！！！！')
        angle = 1.0 * term1 + 0.05 * term2
      else:
        angle = 1.0 * term1 + 0.02 * term2

    else:
      if dis_90[0]/dis_90[lenth_dis-1]>3 or dis_90[lenth_dis-1]/dis_90[0]>3: 
        angle = -max(math.exp(-max_dis/DIR_DETECT_THRESHOLD),0.7)*(max_dir_index-90)/360 *math.pi + 0.1*(dis_90[0]-dis_90[lenth_dis-1])/(dis_90[0]+dis_90[lenth_dis-1])
      else:
        angle = -max(math.exp(-max_dis/DIR_DETECT_THRESHOLD),0.7)*(max_dir_index-90)/360 *math.pi + 0.05*(dis_90[0]-dis_90[lenth_dis-1])/(dis_90[0]+dis_90[lenth_dis-1]) 

    

    steering_angle = P* angle + D *(angle-last_angle)
    last_angle = angle

    #speed can be set to 0.5 to 3.5 m/s, 3 by default
    speed = 2.4*(0.3*math.exp(-np.clip(abs(angle),0,0.5))+0.7)
    #absangle = abs(angle)
    angleabs = abs(angle)
    
    angle_filter = steering_angle
    #print("speed is ",speed)
    max_dis = []
    print("max_dir_index",max_dir_index)
  
    steering_angle = turn_rate*steering_angle
    steering_angle = np.clip(steering_angle, -math.pi/4, math.pi/4)  # 限制转向角度在[-45, 45]度之间
    if steering_angle > 0:    
      print("转向左转%f度"%(abs(steering_angle*180/math.pi)))
    else:
      print("转向右转%f度"%(abs(steering_angle*180/math.pi)))
    # print("max_dir_index is ",max_dir_index)
    publish_arrow_marker(max_dir_index)
    last_max_dir_index = max_dir_index
 
    drive_msg = AckermannDriveStamped()
    # drive_msg = AckermannDrive()
    drive_msg.drive.steering_angle = steering_angle
    drive_msg.drive.speed= speed_rate*speed
    
    if Follow:
      print('\033[35m跟随！！22222！！跟随！！2222！！\033[0m')
      drive_msg.drive.speed = min(MIN_OBS_SPEED,speed_rate*speed)
    elif chaoche:
      drive_msg.drive.speed= speed_rate*speed
      print("""
      \033[31m   _____                         _____  
      \033[32m  / ____|                       / ____| 
      \033[33m | (___  _   _ _ __   ___ _ __  | |  __ 
      \033[34m  \___ \| | | | '_ \ / _ \ '__| | | |_ |
      \033[35m  ____) | |_| | |_) |  __/ |    | |__| |
      \033[36m |_____/ \__,_| .__/ \___|_|     \_____|
      \033[37m              | |                      
      \033[35m              |_|   \033[5m超车模式启动！！！\033[0m
      """)
      print("speed:",drive_msg.drive.speed)
    drive_pub.publish(drive_msg)


if __name__ == '__main__': 
  try:
    rospy.init_node("wall_following1")
    scan_sub = rospy.Subscriber('scan', LaserScan, middle_line_callback,queue_size=1)
    #drive_pub = rospy.Publisher('ackermann_cmd', AckermannDrive, queue_size=1)
    drive_pub = rospy.Publisher('ackermann_cmd_stamped', AckermannDriveStamped, queue_size=1)
    scan_pub = rospy.Publisher('front_scan_01', LaserScan, queue_size=10)
    marker_pub = rospy.Publisher('arrow_marker_01', Marker, queue_size=1)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass