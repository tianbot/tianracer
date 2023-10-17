#! /usr/bin/env python
# source code was from https://www.guyuehome.com/35146
# modified by Lr
import yaml
import rospy
import geometry_msgs.msg as geometry_msgs

class WaypointGenerator(object):

    def __init__(self, filename):
        # 通过订阅rviz发出的/clicked_point的rostopic，就可以获取鼠标点击的目标点了：
        # Subscriber订阅/clicked_point的同时，交给作为callback函数的_process_pose去处理路标点：
        self._sub_pose = rospy.Subscriber('/move_base_simple/goal', geometry_msgs.PoseStamped, self._process_pose, queue_size=1)
        self._waypoints = []
        self._filename = filename

    def _process_pose(self, msg):
        print(msg)
        p = msg.pose.position
        ori = msg.pose.orientation
        print(p)
        data = {}
        data['frame_id'] = msg.header.frame_id
        data['pose'] = {}
        # 因为rviz输出的是2D Nav Goal所以只处理2维：
        data['pose']['position'] = {'x': p.x, 'y': p.y, 'z': 0.0}
        data['pose']['orientation'] = {'x': ori.x, 'y': ori.y, 'z': ori.z, 'w':ori.w}
        data['name'] = '%s_%s' % (p.x, p.y)

        self._waypoints.append(data)
        rospy.loginfo("Clicked : (%s, %s, %s)" % (p.x, p.y, p.z))

    def _write_file(self):
        ways = {}
        ways['waypoints'] = self._waypoints
        # 把目标点输出成yaml文件：
        with open(self._filename, 'a') as f:
            print("-------",self._filename)
            f.write(yaml.dump(ways, default_flow_style=False))

    def spin(self):
        rospy.spin()
        self._write_file()


if __name__ == '__main__':

    rospy.init_node('waypoint_generator')
    filename = "/home/tianbot/tianbot_ws/src/tianracer/tianracer_gazebo/scripts/waypoint_touring/points.yaml"

    g = WaypointGenerator(filename)
    rospy.loginfo('Initialized')
    g.spin()
    rospy.loginfo('ByeBye')