#! /usr/bin/env python
# source code was from https://www.guyuehome.com/35146
# modified by Lr_2002
import yaml
import rospy
import move_base_msgs.msg as move_base_msgs
import geometry_msgs.msg as geometry_msgs
import visualization_msgs.msg as viz_msgs
import std_msgs.msg as std_msgs

id_count = 1

def get_waypoints(filename):
    """
    read the yaml file 
    """
    # 目标点文件是yaml格式的：
    with open(filename, 'r') as f:
        data = yaml.load(f)

    return data['waypoints']

def create_geo_pose(p):
    """
    generate the geometry pose
    """
    pose = geometry_msgs.Pose()

    pose.position.x = p['pose']['position']['x']
    pose.position.y = p['pose']['position']['y']
    pose.position.z = p['pose']['position']['z']
    pose.orientation.x = p['pose']['orientation']['x']
    pose.orientation.y = p['pose']['orientation']['y']
    pose.orientation.z = p['pose']['orientation']['z']
    pose.orientation.w = p['pose']['orientation']['w']
    return pose

def create_move_base_goal(p):
    """
    generate the goal place
    p: your goal
    """
    target = geometry_msgs.PoseStamped()
    target.header.frame_id = p['frame_id']
    target.header.stamp = rospy.Time.now()
    target.pose = create_geo_pose(p)
    goal = move_base_msgs.MoveBaseGoal(target)
    return goal

def create_viz_markers(waypoints):
    """
    generate rviz_marker
    """
    marray= viz_msgs.MarkerArray()
    for w in waypoints:
        m_arrow = create_arrow(w)
        m_text = create_text(w)
        marray.markers.append(m_arrow)
        marray.markers.append(m_text)
    return marray

def create_marker(w):
    """
    generate marker
    """
    global id_count
    m = viz_msgs.Marker()
    m.header.frame_id = w['frame_id']
    m.ns = w['name']
    m.id = id_count
    m.action = viz_msgs.Marker.ADD
    m.pose = create_geo_pose(w)
    m.scale = geometry_msgs.Vector3(1.0,0.3,0.3)
    m.color = std_msgs.ColorRGBA(0.0,1.0,0.0,1.0)

    id_count = id_count + 1
    return m

def create_arrow(w):
    """
    generate arrow
    """
    m = create_marker(w)
    m.type = viz_msgs.Marker.ARROW
    m.color = std_msgs.ColorRGBA(0.0,1.0,0.0,1.0)
    return m

def create_text(w):
    """
    generate text in rviz
    """
    m = create_marker(w)
    m.type = viz_msgs.Marker.TEXT_VIEW_FACING
    m.pose.position.z = 2.5
    m.text = w['name']
    return m
