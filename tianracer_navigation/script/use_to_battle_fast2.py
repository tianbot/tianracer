#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import copy
import os

class WallFollowing(Node):
    def __init__(self, node_name, scan_pub_topic, marker_pub_topic):
        super().__init__(node_name)
        self.robot_name = os.getenv("TIANRACER_NAME", "tianracer")
        
        self.last_angle = 0.0
        self.last_max_dir_index = 0.0
        self.DIR_DETECT_THRESHOLD = 2.5 
        self.OBS_DETECT_THRESHOLD = 5.0 
        self.MAX_SPEED_RATE = 2.0
        self.THRESHOLD_obs = 0.5
        self.THRESHOLD_TURN = 0.5 
        self.START_ANGLE = -60
        self.END_ANGLE = 60
        self.GO_STARIGHT = 0 
        self.TRANSITION = 0 
        self.last_in_normol = False  
        self.last_in_straight = False  
        self.speed_rate = 1.0         
        self.straight_cnt = 0
        self.MIN_OBS_SPEED = 2.0
        self.Follow = False
        self.turn_rate = 1.0 
        self.P = 1.1
        self.D = 0.2
        self.dynamic_obs = False
        self.chaoche = False

        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd_stamped', 10)
        self.scan_pub = self.create_publisher(LaserScan, scan_pub_topic, 10)
        self.marker_pub = self.create_publisher(Marker, marker_pub_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.middle_line_callback, 1)

    def publish_arrow_marker(self, max_dir_index):
        marker = Marker()
        marker.header.frame_id = self.robot_name + "/laser"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "direction_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        p0 = Point()
        p0.x, p0.y, p0.z = 0.0, 0.0, 0.0
        angle_rad = math.radians(max_dir_index)
        p1 = Point()
        p1.x, p1.y, p1.z = math.sin(angle_rad), math.cos(angle_rad), 0.0
        marker.points.append(p0)
        marker.points.append(p1)

        marker.scale.x = 0.1  
        marker.scale.y = 0.1  
        marker.scale.z = 0.2  
        marker.color.a = 1.0
        marker.color.r = 0.1
        marker.color.g = 1.0  
        marker.color.b = 0.1
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = int(0.1 * 1e9)
        self.marker_pub.publish(marker)

    def get_dis(self, data, angle, deg=True, return_inten=True):
        if deg:
            angle = np.deg2rad(angle)
        
        # Calculate index
        temp = int(round((angle - data.angle_min) / data.angle_increment))
        
        # Clamp temp to safety
        temp = max(2, min(temp, len(data.ranges) - 3))
        
        # Get slice and sort
        data_tmp = np.sort(data.ranges[temp-2:temp+3])
        
        dis = data_tmp[len(data_tmp)//2]
        
        intensities = 0.0
        if return_inten and len(data.intensities) > 0:
            inten_tmp = np.sort(data.intensities[temp-2:temp+3])
            intensities = inten_tmp[len(inten_tmp)//2]
            
        return (dis, intensities) if return_inten else dis

    def get_range(self, data, start_angle, end_engle, return_inten=False):
        all_dis, all_inten = [], []
        for angle in range(start_angle, end_engle):
            tmp = self.get_dis(data, angle, return_inten=True)
            all_dis.append(tmp[0])
            all_inten.append(tmp[1])
        return (all_dis, all_inten) if return_inten else all_dis

    def fill_zeros_with_neighbors(self, data):
        result = list(data)
        n = len(result)
        for i in range(n):
            if result[i] == 0:
                left = next((result[j] for j in range(i-1, -1, -1) if result[j] != 0), None)
                if left is not None:
                    result[i] = left
                    continue
                right = next((result[j] for j in range(i+1, n) if result[j] != 0), None)
                if right is not None:
                    result[i] = right
                    continue
                result[i] = 0
        return result

    def filter_obstacles_by_variance(self, Left_obs_orig, dis_90, variance_threshold=1.0):
        Left_obs = []
        if len(Left_obs_orig) > 0:
            for i in range(0, int(len(Left_obs_orig) / 2)):
                obstacle_range = dis_90[Left_obs_orig[2*i]: Left_obs_orig[2*i+1]]
                dis_obs_var = np.var(obstacle_range)
                print("in filter_obstacles_by_variance,方差：", dis_obs_var)
                Left_obs.extend([Left_obs_orig[2*i], Left_obs_orig[2*i+1]])
        return Left_obs

    def filter_anomalous_values(self, data, max_distance=4, angle_range=2):
        data = np.array(data)
        for i in range(1, len(data) - 1):
            if data[i] != max_distance:
                if all(data[j] == max_distance for j in range(i - angle_range, i + angle_range + 1) if 0 <= j < len(data)):
                    data[i] = (data[i-1] + data[i+1]) / 2
        return data.tolist()

    def filter_small_obstacles(self, Left_obs, min_obstacle_size=2):
        for i in range(int(len(Left_obs) / 2)):
            if abs(Left_obs[2*i] - Left_obs[2*i+1]) <= min_obstacle_size:
                Left_obs[2*i] = Left_obs[2*i+1] = -1
        return [x for x in Left_obs if x != -1]

    def pub_scan(self, dis_90, msgs):
        scan_msg = LaserScan()
        scan_msg.header = msgs.header
        scan_msg.angle_min = np.pi / 2
        scan_msg.angle_max = -np.pi / 2
        scan_msg.angle_increment = -np.pi/180
        scan_msg.ranges = [float(x) for x in dis_90]
        scan_msg.range_max = 100.0
        self.scan_pub.publish(scan_msg)

    def DynamicObastcle(self, dis_list, inten_list, max_dir_num, obs):
        max_range = [int(max_dir_num[0]), int(max_dir_num[-1])]
        obs_range = [int(max_dir_num[1]), int(max_dir_num[2])]
        range_list = inten_list[max_range[0]:max_range[1]]
        obs_intensity = inten_list[obs_range[0]:obs_range[1]]
        average_obs_intensity = np.mean(obs_intensity)
        average_intensity = np.mean(range_list)
        abs_tmp = abs(average_obs_intensity - average_intensity)
        if abs_tmp > 5:
            print('detect dynamic obs abs_tmp!!!!', abs_tmp)
            return True
        return False

    def middle_line_callback(self, data):
        self.dynamic_obs = False
        self.chaoche = False
        self.Follow = False
        self.D = 0.2
        print("###########################################################")
        dis_90, inten_90 = self.get_range(data, -89, 91, True)
        dis_90 = dis_90[::-1]
        inten_90 = inten_90[::-1]
        dis_obs_90 = copy.deepcopy(dis_90)
        lenth_dis = len(dis_90)
        
        left, right = 0, 0
        Left_obs_orig, Left_obs = [], []
        max_dis_num, max_dir_num = [], []
        max_dis, max_dis_index = 0, 0
        self.pub_scan(dis_90, data)
        
        dis_90 = self.fill_zeros_with_neighbors(dis_90)
        inten_90 = self.fill_zeros_with_neighbors(inten_90)
        dis_obs_90 = self.fill_zeros_with_neighbors(dis_obs_90)
        dis_90_copy = tuple(dis_90)
        
        for i in range(0, lenth_dis):
            if dis_90[i] > max_dis and 20 < i < 160:
                max_dis = dis_90[i]
                max_dis_index = i 
            if dis_90[i] > self.DIR_DETECT_THRESHOLD:
                dis_90[i] = self.DIR_DETECT_THRESHOLD 
            if dis_obs_90[i] > self.OBS_DETECT_THRESHOLD:
                dis_obs_90[i] = self.OBS_DETECT_THRESHOLD 
                
        dis_90 = self.filter_anomalous_values(dis_90, max_distance=self.DIR_DETECT_THRESHOLD)
        dis_obs_90 = self.filter_anomalous_values(dis_obs_90, max_distance=self.OBS_DETECT_THRESHOLD)
        
        if max_dis_index < 89: left = 1
        else: right = 1
        
        for i in range(0, lenth_dis-2):
            if dis_obs_90[i]-dis_obs_90[i+1] > self.THRESHOLD_obs and len(Left_obs_orig)%2 == 0:
                Left_obs_orig.append(i+1)
            elif dis_obs_90[i+1]-dis_obs_90[i] > self.THRESHOLD_obs and len(Left_obs_orig)%2 == 1:
                Left_obs_orig.append(i)
        if len(Left_obs_orig)%2 == 1: Left_obs_orig.pop()
        
        Left_obs = self.filter_small_obstacles(Left_obs_orig, min_obstacle_size=2)
        Left_obs = self.filter_obstacles_by_variance(Left_obs, dis_obs_90, variance_threshold=1.0)
        
        if len(Left_obs) > 0:
            for i in range(int(len(Left_obs)/2)):
                obs_middle = dis_obs_90[int((Left_obs[2*i]+Left_obs[2*i+1])/2)]
                expand = min((Left_obs[2*i+1]-Left_obs[2*i])/2*(4-obs_middle), 10)
                start_j = int(max(Left_obs[2*i]-expand, 0))
                end_j = int(min(Left_obs[2*i+1]+expand, lenth_dis-1))
                for j in range(start_j, end_j + 1):
                    dis_obs_90[j] = obs_middle
                Left_obs[2*i] = start_j
                Left_obs[2*i+1] = end_j
            print("有障碍物，障碍物是", Left_obs)
        else:
            print("没有障碍物")

        if len(Left_obs) > 0:
            for i in range(int(len(Left_obs)/2)+1):
                if i == 0:
                    found = False
                    for j in range(int(Left_obs[0]-1), 0, -1):
                        if dis_obs_90[j] <= dis_obs_90[int(Left_obs[0]+1)]:
                            max_dis_num.extend([j, int(Left_obs[0]+1)])
                            found = True
                            break
                    if not found:
                        for j in range(0, int(Left_obs[0]-1)):
                            if dis_obs_90[j] >= dis_obs_90[int(Left_obs[0]+1)]:
                                max_dis_num.extend([j, int(Left_obs[0]+1)])
                                break
                elif i < int(len(Left_obs)/2):
                    max_dis_num.extend([Left_obs[2*i-1], Left_obs[2*i]])
                elif i == int(len(Left_obs)/2):
                    for j in range(int(Left_obs[2*i-1]+1), lenth_dis-1):
                        if dis_obs_90[j] <= dis_obs_90[int(Left_obs[2*i-1]-1)]:
                            max_dis_num.extend([int(Left_obs[2*i-1]-1), j])
                            break
            
            max_dis_val = 0
            max_dis_index_temp = max_dis_index
            for i in range(int(len(max_dis_num)/2)):
                if max_dis_val < max_dis_num[2*i+1] - max_dis_num[2*i]:
                    max_dis_val = max_dis_num[2*i+1] - max_dis_num[2*i]
                    max_dis_index = (max_dis_num[2*i+1] + max_dis_num[2*i])/2
                if left == 1 and max_dis_index < 90 and dis_obs_90[0] < dis_obs_90[lenth_dis-1]-1:
                    max_dis_index += 5*abs(dis_obs_90[lenth_dis-1]-dis_obs_90[0])
                elif right == 1 and max_dis_index > 90 and dis_obs_90[0]-1 > dis_obs_90[lenth_dis-1]:
                    max_dis_index -= 5*abs(dis_obs_90[lenth_dis-1]-dis_obs_90[0])
            
            if len(Left_obs) == 2:
                if max_dis_index_temp >= 89:
                    max_dis_index = int((89+Left_obs[0])/2) if Left_obs[0] >= 89 else max_dis_index_temp
                else:
                    max_dis_index = int((89+Left_obs[1])/2) if Left_obs[1] <= 89 else max_dis_index_temp
            elif len(Left_obs) == 4:
                middle_temp = int((Left_obs[1]+Left_obs[2])/2)
                if max_dis_index_temp >= 89:
                    if Left_obs[0] >= 89: max_dis_index = int((89+Left_obs[0])/2)
                    elif Left_obs[1] <= 89 and middle_temp > 89: max_dis_index = int((Left_obs[1]+max_dis_index_temp)/2)
                    elif middle_temp <= 89 and Left_obs[2] > 89: max_dis_index = middle_temp
                    else: max_dis_index = max_dis_index_temp
                else:
                    if Left_obs[0] > 89: max_dis_index = max_dis_index_temp
                    elif Left_obs[0] <= 89 and middle_temp > 89: max_dis_index = middle_temp
                    elif middle_temp <= 89 and Left_obs[3] > 89: max_dis_index = middle_temp
                    else: max_dis_index = int((Left_obs[3] + 89)/2)

        for i in range(0, lenth_dis-2):
            if dis_90[i] < self.DIR_DETECT_THRESHOLD and dis_90[i+1] == self.DIR_DETECT_THRESHOLD and len(max_dir_num)%2 == 0:
                max_dir_num.append(i+1)
            elif dis_90[i] == self.DIR_DETECT_THRESHOLD and dis_90[i+1] < self.DIR_DETECT_THRESHOLD and len(max_dir_num)%2 == 1:
                max_dir_num.append(i)

        if len(max_dir_num) % 2 == 1 and len(max_dir_num) != 1:
            self.get_logger().error("出现单个不封闭区域，请检查障碍物检测逻辑")
            
        max_dir_index = 0
        if len(max_dir_num) == 1:
            if max_dir_num[0] < 90: max_dir_index = int((max_dir_num[0])/2)
            elif max_dir_num[0] > 90: max_dir_index = int((max_dir_num[0]+lenth_dis-2)/2)
            self.GO_STARIGHT = 0
        elif len(max_dir_num) == 2:
            max_dir_index = int((max_dir_num[0]+max_dir_num[1])/2)
            max_dir_range = max_dir_num[1]-max_dir_num[0]
        elif len(max_dir_num) > 2:
            if len(Left_obs) > 0:
                self.dynamic_obs = self.DynamicObastcle(dis_90, inten_90, max_dir_num, Left_obs)
            cand_space, cand_dirs = [], []
            for i in range(int(len(max_dir_num)/2)):
                cand_space.append(max_dir_num[2*i+1]-max_dir_num[2*i])
                cand_dirs.append((max_dir_num[2*i+1]+max_dir_num[2*i])/2)
            cand_dir_id = np.where(np.array(cand_space) > 18)[0]
            if len(cand_dir_id) != 0:
                selected_dirs = np.array(cand_dirs)[cand_dir_id].tolist()
                max_dir_idx = np.argmin(selected_dirs)
                selected_ranges = np.array(cand_space)[cand_dir_id].tolist()
                max_dir_index = selected_dirs[max_dir_idx]
                max_dir_range = selected_ranges[max_dir_idx]
                cand_dir_chaoche_idx = np.where(np.array(cand_space) > 30)[0]
                if self.dynamic_obs:
                    if cand_dir_chaoche_idx.size: self.chaoche = True
                    else: self.Follow = True
            else:
                cand_dir_id = np.argmax(np.array(cand_space))
                max_dir_index = cand_dirs[cand_dir_id]
                max_dir_range = cand_space[cand_dir_id]
                if self.dynamic_obs:
                    max_dir_index = int((max_dir_num[1] + max_dir_num[2]) / 2)
                    max_dir_range = max_dir_num[-1] - max_dir_num[0]
                    self.Follow = True
            max_dir_index += -2 if max_dir_index < 90 else 2

        if 75 <= max_dir_index <= 105:
            mean_straight = np.mean(dis_90_copy[80:100])
            self.GO_STARIGHT = 1
            self.TRANSITION = 0      
            if self.last_in_straight and max_dir_range > 20:
                self.speed_rate *= 1.05
                limit_rate = 3.0 if mean_straight > 11 else 2.5 if mean_straight > 8 else 2.2 if mean_straight > 7 else 2.0
                if self.speed_rate > limit_rate: self.speed_rate = limit_rate
            else:
                self.speed_rate = 1.1
            self.last_in_straight = True    
        elif 0 < max_dir_index < 75:
            self.P, self.speed_rate, self.turn_rate, self.last_in_straight = 1.5, 1.0, 0.8, False
        elif max_dir_index > 105:
            self.P, self.speed_rate, self.turn_rate, self.last_in_straight = 1.5, 1.0, 0.8, False
        
        normol = 1
        if len(max_dir_num) == 0:
            if self.GO_STARIGHT == 1 or self.TRANSITION == 1:
                for i in range(0, lenth_dis-2):
                    if dis_90[i+1] - dis_90[i] > self.THRESHOLD_TURN:
                        max_dir_index = int((i+1+len(dis_90)/2)/2)
                        self.get_logger().warn("进入过渡路段，前方左转")
                        self.P, normol = 0.8, 0
                    elif dis_90[i] - dis_90[i+1] > self.THRESHOLD_TURN:
                        max_dir_index = int((i+len(dis_90)/2)/2)
                        self.get_logger().warn("进入过渡路段，前方右转")
                        self.P, normol = 0.8, 0
                if normol == 1:
                    max_dir_index = self.last_max_dir_index
                    if self.last_in_normol:
                        self.speed_rate = max(self.speed_rate * 0.9, 0.5)
                        self.turn_rate = min(self.turn_rate * 1.2, 2.5)
                    else:
                        self.speed_rate, self.turn_rate = 0.9, 1.2
                    self.last_in_normol = True
                else:
                    self.speed_rate, self.turn_rate, self.last_in_normol = 1.0, 1.0, False
                self.TRANSITION, self.GO_STARIGHT = 1, 0

        dis_90[0] += 0.00001
        dis_90[lenth_dis-1] += 0.00001
        
        if max_dir_index != 0:
            term1 = -max(math.exp(-max_dis/self.DIR_DETECT_THRESHOLD), 0.7)*(max_dir_index-90)/360 * math.pi
            term2 = (dis_90[0]-dis_90[lenth_dis-1])/(dis_90[0]+dis_90[lenth_dis-1])
            angle = (1.0 * term1 + 0.05 * term2) if (dis_90[0]/dis_90[lenth_dis-1] > 3 or dis_90[lenth_dis-1]/dis_90[0] > 3) else (1.0 * term1 + 0.02 * term2)
            steering_angle = self.P * angle + self.D * (angle - self.last_angle)
            self.last_angle = angle
            speed = 2.4 * (0.3 * math.exp(-np.clip(abs(angle), 0, 0.5)) + 0.7)
            steering_angle = np.clip(self.turn_rate * steering_angle, -math.pi/4, math.pi/4)
            self.publish_arrow_marker(max_dir_index)
            self.last_max_dir_index = max_dir_index
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = self.get_clock().now().to_msg()
            drive_msg.drive.steering_angle = float(steering_angle)
            drive_msg.drive.speed = float(self.speed_rate * speed)
            if self.Follow: drive_msg.drive.speed = min(self.MIN_OBS_SPEED, drive_msg.drive.speed)
            self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowing("wall_following2", "front_scan_02", "arrow_marker_02")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
