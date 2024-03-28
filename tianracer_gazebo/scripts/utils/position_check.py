#! /usr/bin/env python

# @Time: 2023/10/20 17:02:46
# @Author: Jeff Wang(Lr_2002)
# LastEditors: sujit-168 su2054552689@gmail.com
# LastEditTime: 2024-03-26 14:49:24

import rospy, rospkg, os, sys
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates

# sys.path.append("..")
sys.path.append(os.path.abspath(__file__+"/../.."))
import waypoint_race.utils as utils

ana_cnt = 0
ana_check_list = []
ana_history_position= (0,0)

def load_checkpoint(file_name):
    """
        description: load score check point from yaml file_name.yaml
        args:        file_name: is the file name of the yaml file.
        return:  global ana_check_list
    """

    _checkpoint = utils.get_waypoints(file_name)

    for _count in range(len(_checkpoint)):
        _point = _checkpoint[_count]
        _point_x = utils.create_geometry_pose(_point).position.x
        _point_y = utils.create_geometry_pose(_point).position.y
        
        if _count % 2 == 0:
            # Combine the current point and next point into a tuple
            _pair = (_point_x, _point_y),
        else:
            # Combine the previously combined tuple and the current point into a new tuple.
            # The points follow the order of operations, that is, the points in () come first.
            _pair = _pair + ((_point_x, _point_y),)

            # add the tuple combined into the ana_check_list
            ana_check_list.append(_pair)

    # print(ana_check_list)

def is_intersect(line1, line2):  
    """
        description: determine whether two Line Segments Intersect 
        args:        line1, line2:  is the Line two end points Of A Line Segment, Each of which is A binary
                     , such as ((-0.8541378373856054,-0.57080969486389), (-1.915068709596355,-0.5493092660435841))  
        return:   True or False
    """  
    x1, y1 = line1[0]  
    x2, y2 = line1[1]  
    x3, y3 = line2[0]  
    x4, y4 = line2[1]
      
    # determine whether The Two Line Segments Are Parallel 
    if (y4 - y3) * (x2 - x1) == (y2 - y1) * (x4 - x3):  
        return False  
      
    # determine whether the point is in line 
    def is_on_segment(x, y, x1, y1, x2, y2):  
        return min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2)  
      
    # determine whether the line segments intersect 
    ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))  
    ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))  
    if 0 <= ua <= 1 and 0 <= ub <= 1:  
        return True  
    elif is_on_segment(x1, y1, x3, y3, x4, y4) or is_on_segment(x2, y2, x3, y3, x4, y4):  
        return True  
    else:  
        return False

def cal_distance(points):
    """
        calc the L2 norm distance
    """
    point1 ,point2 = points  
    x1, y1 = point1  
    x2, y2 = point2  
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

def analysis(data):
    global ana_history_position,ana_cnt, ana_check_list
    data = data.pose
    now_position = data[2].position
    x = now_position.x
    y = now_position.y
    now_line = (ana_history_position, (x,y))
    distance = cal_distance(now_line)
    # if distance <=0.01:
    #     distance = 0
    ana_history_position = (x, y)
    # print(history_position)
    tmp = ana_cnt % 3
    if is_intersect(now_line, ana_check_list[tmp]):
        print('已经通过' + str(tmp) +'点')
        ana_cnt +=1 
        return ('已经通过' + str(tmp) +'点', ana_cnt-1), distance
    return None, distance

def test():
    package_name = "tianracer_gazebo"

    # Get the package path
    try:
        pkg_path = rospkg.RosPack().get_path(package_name)

        # Construct the path to scripts directory
        filename= os.path.join(pkg_path, "scripts/waypoint_race/check_points.yaml")
        # print(f"check_point.yaml: {filename}")
    except rospkg.ResourceNotFound:
        rospy.logerr("Package '%s' not found" % package_name)
        exit(1)

    filename = rospy.get_param("~filename",filename)

    load_checkpoint(filename)

test()

if __name__ == "__main__":
    rospy.init_node("analysis")
    states = rospy.Subscriber("/gazebo/model_states", ModelStates, callback=analysis)
    rospy.spin()