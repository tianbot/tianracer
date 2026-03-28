#!/usr/bin/python3
#coding:utf-8						# 中文注释

# ======================================================
# 线性时变模型预测控制
# CloudGQ
# 2021-09-18
# ======================================================


import rospy
import copy
import numpy as np
import math
import tf
import matplotlib
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped , PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
import time
import os
import cvxpy
import sys

#MPC参数设置
T = 7 # horizon length
NX = 3 # x = x, y, yaw
NU = 2 # u = [v,delta]
R = np.diag([0.1, 0.1])  # input cost matrix
Rd = np.diag([0.1, 0.1])  # input difference cost matrix
Q = np.diag([1, 1, 1])  # state cost matrix
Qf = Q  # state final matrix
#车辆参数设置
dt=0.02 # 时间间隔
L=0.26 # 车辆轴距
MAX_STEER = np.deg2rad(20.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(10.0)  # maximum steering speed [rad/s]
MAX_VEL = 0.5  # maximum accel [m/s]
#定义小车类
class Turtlebot:
	def __init__(self):
		rospy.init_node('tianracer_mpc',log_level=rospy.DEBUG)
		##tianracer的真实状态初始化
		# self.ugv = None # 车的实例
		self.robot_state = np.zeros(4) # 车的状态(x,y,yaw,v)
		##车的期望状态初始化
		self.dref = np.zeros((NU, T)) # 车在下面T个时刻的期望输出
		self.xref = np.zeros((NX, T+1)) # 车在下面T+1个时刻的期望状态
		self.refer_yaw = np.zeros((T+1, 1)) # 车在下面T+1个时刻的期望yaw
		self.refer_k = np.zeros((T+1, 1)) # 车在下面T+1个时刻的期望曲率
		self.set_zero = False		
		self.target_ind = 0
		self.vel = 0
		##获取要跑的路径信息
		# self.refer_path = self.ReadTarjectory("/home/tianbot/tianbot_ws/src/tianracer/tianracer_navigation/script/B1.txt")
		# print(self.refer_path)
        ##订阅里程计信息，获取在第一时刻的真实状态
		rospy.Subscriber("/move_base/TebLocalPlannerROS/global_plan", Path, self.path_callback)
		# print(self.refer_path)
		rospy.Subscriber('/tianracer/odom', Odometry, self.vel_cb)
		rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.pose_cb)
		##发布车子的速度和转角
		self.tarj_pub = rospy.Publisher('visualization_tarj', MarkerArray, queue_size=10)
		self.goal_direction_pub = rospy.Publisher('visualization_goal_direction', Marker, queue_size=10)
		self.ackermann_pub = rospy.Publisher("/tianracer/ackermann_cmd", AckermannDrive, queue_size=1)
		time.sleep(2)
		self.loop()

	def path_callback(self,msg):
		# Callback function to process received local plan message
		path_points = msg.poses
		path_length = len(path_points)
		
		# Extract x, y coordinates from each PoseStamped point
		data1 = []
		for pose in path_points:
			x = pose.pose.position.x
			y = pose.pose.position.y
			data1.append((x, y))
		
		# Example of what you can do with the data (printing here)
		rospy.loginfo("Received local plan with {} points".format(path_length))
		rospy.loginfo("Path points (x, y): {}".format(data1))
		data = data1[::1]
		data=np.array(data)
		num_rows = data.shape[0]  # Get the number of rows
		print("row",num_rows)
		refer_path = np.zeros((num_rows,4))
		refer_path[:,0] = data[:, 0]  # x
		refer_path[:,1] = data[:, 1]  # y
		# 使用差分的方式计算路径点的一阶导和二阶导，从而得到切线方向和曲率
		for i in range(len(refer_path)):
			if i == 0:
				dx = refer_path[i+1,0] - refer_path[i,0]
				dy = refer_path[i+1,1] - refer_path[i,1]
				ddx = refer_path[2,0] + refer_path[0,0] - 2*refer_path[1,0]
				ddy = refer_path[2,1] + refer_path[0,1] - 2*refer_path[1,1]
			elif i == (len(refer_path)-1):
				dx = refer_path[i,0] - refer_path[i-1,0]
				dy = refer_path[i,1] - refer_path[i-1,1]
				ddx = refer_path[i,0] + refer_path[i-2,0] - 2*refer_path[i-1,0]
				ddy = refer_path[i,1] + refer_path[i-2,1] - 2*refer_path[i-1,1]
			else:      
				dx = refer_path[i+1,0] - refer_path[i,0]
				dy = refer_path[i+1,1] - refer_path[i,1]
				ddx = refer_path[i+1,0] + refer_path[i-1,0] - 2*refer_path[i,0]
				ddy = refer_path[i+1,1] + refer_path[i-1,1] - 2*refer_path[i,1]
			refer_path[i,2]=math.atan2(dy,dx) # yaw
			# 计算曲率:设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
			# 参考：https://blog.csdn.net/weixin_46627433/article/details/123403726
			refer_path[i,3]=(ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2)) # 曲率k计算
		self.refer_path=refer_path

	def publish_markers(self,xref):
		marker_array = MarkerArray()
		for i in range(len(xref[0])):
			marker = Marker()
			marker.header.frame_id = 'map'
			marker.header.stamp = rospy.Time.now()
			marker.ns = 'points'
			marker.id = i
			marker.type = Marker.SPHERE
			marker.action = Marker.ADD
			marker.pose.position.x = xref[0][i]
			marker.pose.position.y = xref[1][i]
			print("x",xref[0][i],"y",xref[1][i])
			marker.pose.position.z = 0.0
			marker.pose.orientation.x = 0.0
			marker.pose.orientation.y = 0.0
			marker.pose.orientation.z = 0.0
			marker.pose.orientation.w = 1.0
			marker.scale.x = 0.05
			marker.scale.y = 0.05
			marker.scale.z = 0.05
			marker.color.a = 1.0
			marker.color.r = 1.0-i/len(xref[0])
			marker.color.g = i/len(xref[0])
			marker.color.b = i/len(xref[0])
			marker_array.markers.append(marker)

		self.tarj_pub.publish(marker_array)
	
	def publish_arrow(self,x,y,opt_yaw):
		marker = Marker()
		marker.header.frame_id = 'map'
		marker.header.stamp = rospy.Time.now()
		marker.ns = 'arrow'
		marker.id = 1
		marker.type = Marker.ARROW
		marker.action = Marker.ADD
		marker.pose.position.x = x
		marker.pose.position.y = y
		marker.pose.position.z = 0.0
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = math.sin(opt_yaw / 2.0)
		marker.pose.orientation.w = math.cos(opt_yaw / 2.0)
		marker.scale.x = 0.2
		marker.scale.y = 0.02
		marker.scale.z = 0.02
		marker.color.a = 1.0
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0

		self.goal_direction_pub.publish(marker)

	def vel_cb(self,data):
		self.vel = data.twist.twist.linear.x

	def pose_cb(self,data):
		self.robot_state[0] = data.pose.pose.position.x 
		self.robot_state[1] = data.pose.pose.position.y
		quanternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)			# 四元数
		euler = tf.transformations.euler_from_quaternion(quanternion)	# 四元数到欧拉角变换
		self.robot_state[2] = euler[2]
		self.robot_state[3] = self.vel
		# self.robot_state[3] = data.twist.twist.linear.x
		self.x0 = self.robot_state[0:3]
		# print("robot_state",self.robot_state)
	
	def normalize_angle(self,angle):
		"""
		Normalize an angle to [-pi, pi].

		:param angle: (float)
		:return: (float) Angle in radian in [-pi, pi]
		copied from https://atsushisakai.github.io/PythonRobotics/modules/path_tracking/stanley_control/stanley_control.html
		"""
		while angle > np.pi:
			angle -= 2.0 * np.pi

		while angle < -np.pi:
			angle += 2.0 * np.pi

		return angle



	def calc_track_error(self, x, y ,car_yaw):
		"""计算跟踪误差
		Args:
			x (_type_): 当前车辆的位置x
			y (_type_): 当前车辆的位置y

		Returns:
			_type_: _description_
		"""
		# 寻找参考轨迹最近目标点
		d_x = [self.refer_path[i,0]-x for i in range(len(self.refer_path))] 
		d_y = [self.refer_path[i,1]-y for i in range(len(self.refer_path))] 
		d = [np.sqrt(d_x[i]**2+d_y[i]**2) for i in range(len(d_x))]
		d = [np.sqrt(d_x[i] ** 2 + d_y[i] ** 2) if self.is_yaw_condition_satisfied(d_x[i], d_y[i], car_yaw) else np.inf for i in range(len(d_x))]
		s = np.argmin(d) # 最近目标点索引


		yaw = self.refer_path[s, 2]
		k = self.refer_path[s, 3]
		angle = self.normalize_angle(yaw - math.atan2(d_y[s], d_x[s]))
		e = d[s]  # 误差
		if angle < 0:
			e *= -1

		return e, k, yaw, s
	def is_yaw_condition_satisfied(self, d_x, d_y, yaw):
		"""检查目标点与yaw之间的条件是否满足
		Args:
            d_x (_type_): 目标点与当前车辆位置的x方向距离
            d_y (_type_): 目标点与当前车辆位置的y方向距离
            yaw (_type_): 当前车辆的偏航角

        Returns:
            bool: 是否满足条件
        """
		target_yaw = np.arctan2(d_y, d_x)
		angle_diff = np.abs(target_yaw - yaw)
		if angle_diff > np.pi:
			angle_diff = 2 * np.pi - angle_diff
		return angle_diff < np.pi / 2

	def calc_ref_trajectory(self, robot_state, dl=1.0):
		"""计算参考轨迹点，统一化变量数组，便于后面MPC优化使用
			参考自https://github.com/AtsushiSakai/PythonRobotics/blob/eb6d1cbe6fc90c7be9210bf153b3a04f177cc138/PathTracking/model_predictive_speed_and_steer_control/model_predictive_speed_and_steer_control.py
		Args:
			robot_state (_type_): 车辆的状态(x,y,yaw,v)
			dl (float, optional): _description_. Defaults to 1.0.

		Returns:
			_type_: _description_
		"""
		e, k, ref_yaw, ind = self.calc_track_error(robot_state[0], robot_state[1] ,robot_state[2])

		xref = np.zeros((NX, T + 1))
		dref = np.zeros((NU, T)) # 参考控制量
		ncourse = len(self.refer_path)


		xref[0, 0] = self.refer_path[ind,0]
		xref[1, 0] = self.refer_path[ind, 1]
		xref[2, 0] = self.refer_path[ind, 2]

		# 参考控制量[v,delta]
		ref_delta = math.atan2(L*k, 1)
		dref[0, :] = robot_state[3]
		dref[1, :] = ref_delta

		travel = 0.0

		for i in range(T + 1):
			travel += abs(robot_state[3]) * dt
			dind = int(round(travel / dl))

			# if (ind + dind) < ncourse:
			# 	xref[0, i] = self.refer_path[ind + dind,0]
			# 	xref[1, i] = self.refer_path[ind + dind,1]
			# 	xref[2, i] = self.refer_path[ind + dind,2]
			if  (ind+i)<ncourse:
				xref[0, i] = self.refer_path[ind + i,0]
				xref[1, i] = self.refer_path[ind + i,1]
				xref[2, i] = self.refer_path[ind + i,2]
			else:
				new_ind = (ind + i) % ncourse
				# xref[0, i] = self.refer_path[new_ind, 0]
				# xref[1, i] = self.refer_path[new_ind, 1]
				# xref[2, i] = self.refer_path[new_ind, 2]
				xref[0, i] = self.refer_path[ncourse - 1,0]
				xref[1, i] = self.refer_path[ncourse - 1,1]
				xref[2, i] = self.refer_path[ncourse - 1,2]


		return xref, ind, dref
	
	def get_state_space(self,ref_v, ref_delta, ref_yaw):
		"""将模型离散化后的状态空间表达

		Args:
			ref_delta (_type_): 参考的转角控制量
			ref_yaw (_type_): 参考的偏航角

		Returns:
			_type_: _description_
		"""
		A = np.matrix([
			[1.0, 0.0, -ref_v*dt*math.sin(ref_yaw)],
			[0.0, 1.0, ref_v*dt*math.cos(ref_yaw)],
			[0.0, 0.0, 1.0]])

		B = np.matrix([
			[dt*math.cos(ref_yaw), 0],
			[dt*math.sin(ref_yaw), 0],
			[dt*math.tan(ref_delta)/L, ref_v*dt /(L*math.cos(ref_delta)*math.cos(ref_delta))]
		])

		C = np.eye(3)
		return A, B, C
	
	def get_nparray_from_matrix(self,x):
		return np.array(x).flatten()


	def linear_mpc_control(self,xref, x0, delta_ref):
		"""
		linear mpc control

		xref: reference point
		x0: initial state
		delta_ref: reference steer angle
		ugv:车辆对象
		"""

		x = cvxpy.Variable((NX, T + 1))
		u = cvxpy.Variable((NU, T)) 

		cost = 0.0  # 代价函数
		constraints = []  # 约束条件

		for t in range(T):
			cost += cvxpy.quad_form(u[:, t]-delta_ref[:,t], R)

			if t != 0:
				cost += cvxpy.quad_form(x[:, t] - xref[:, t], Q)

			A, B, C = self.get_state_space(delta_ref[0,t],delta_ref[1,t], xref[2,t])
			constraints += [x[:, t + 1]-xref[:, t+1] == A @ (x[:, t]-xref[:, t]) + B @ (u[:, t]-delta_ref[:,t]) ]


		cost += cvxpy.quad_form(x[:, T] - xref[:, T], Qf)

		constraints += [(x[:, 0]) == x0]
		constraints += [cvxpy.abs(u[0, :]) <= MAX_VEL]
		constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

		prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
		prob.solve(solver=cvxpy.CLARABEL, verbose=False)

		if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
			opt_x = self.get_nparray_from_matrix(x.value[0, :])
			opt_y = self.get_nparray_from_matrix(x.value[1, :])
			opt_yaw = self.get_nparray_from_matrix(x.value[2, :])
			opt_v = self.get_nparray_from_matrix(u.value[0, :])
			opt_delta = self.get_nparray_from_matrix(u.value[1, :])

		else:
			print("Error: Cannot solve mpc..")
			opt_v, opt_delta, opt_x, opt_y, opt_yaw = None, None, None, None, None 
		return opt_v, opt_delta, opt_x, opt_y, opt_yaw

	def loop(self):
		rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			if hasattr(self, 'x0') and hasattr(self, 'refer_path'):
				self.xref,self.target_ind,self.dref = self.calc_ref_trajectory(self.robot_state)
				self.publish_markers(self.xref)
				opt_v, opt_delta, opt_x, opt_y, opt_yaw = self.linear_mpc_control(self.xref, self.x0, self.dref)
				self.publish_arrow(opt_x[0],opt_y[0],opt_delta[0]+opt_yaw[0])
				ackermann_cmd = AckermannDrive()
				if (self.set_zero):
					ackermann_cmd.speed = 0
					self.set_zero = False
				else:
					if(opt_v[0]<-0.2):
						ackermann_cmd.speed = -0.2
					elif((opt_v[0]<0.2)):
						ackermann_cmd.speed = 0.2
					else:
						ackermann_cmd.speed = opt_v[0]
				# if(ackermann_cmd.speed<0):
				# 	ackermann_cmd.speed = -ackermann_cmd.speed
				# 	ackermann_cmd.steering_angle = opt_delta[0]
				# else:
				# 	ackermann_cmd.steering_angle = -opt_delta[0]
				ackermann_cmd.steering_angle = opt_delta[0]+0.05
				rospy.loginfo("speed:%.2f",ackermann_cmd.speed)
				rospy.loginfo("steering_angle:%.2f",ackermann_cmd.steering_angle)
				print("")
				self.ackermann_pub.publish(ackermann_cmd)      
			rate.sleep()
if __name__ == "__main__":
	Turtlebot()

		
