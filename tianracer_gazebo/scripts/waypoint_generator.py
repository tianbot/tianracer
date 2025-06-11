#! /usr/bin/env python3
# source code was from https://www.guyuehome.com/35146
# @Time: 2023/10/20  17:03:06
# @Author: Jeff Wang(Lr_2002)

import os
import yaml
import rospy, rospkg
import geometry_msgs.msg as geometry_msgs

robot_name = os.getenv("TIANRACER_NAME", "tianracer")

class WaypointGenerator(object):
    def __init__(self, filename):
        """
        Subscrib "/move_base_simple/goal" which has orientation for the move_base 
        """
        self._sub_pose = rospy.Subscriber(f'{robot_name}/move_base_simple/goal', geometry_msgs.PoseStamped, self._process_pose, queue_size=1)
        self._waypoints = []
        self._filename = filename

    def _process_pose(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation
        data = {}
        data['frame_id'] = msg.header.frame_id
        data['pose'] = {}
        data['pose']['position'] = {'x': pos.x, 'y': pos.y, 'z': 0.0}
        data['pose']['orientation'] = {'x': ori.x, 'y': ori.y, 'z': ori.z, 'w':ori.w}
        data['name'] = '%s_%s' % (pos.x, pos.y)

        self._waypoints.append(data)
        rospy.loginfo("Clicked : (%s, %s, %s)" % (pos.x, pos.y, pos.z))

    def _write_file(self):
        way_pts = {}
        way_pts['waypoints'] = self._waypoints

        # export to yaml file
        try:
            with open(self._filename, 'a') as f:
                rospy.loginfo("your waypoints files save as: \n------------------------------------------------\n %s \n------------------------------------------------", self._filename)
                f.write(yaml.dump(way_pts, default_flow_style=False))
        except FileNotFoundError as e:
            rospy.logerr(e)

    def spin(self):
        rospy.spin()
        self._write_file()

if __name__ == '__main__':
    rospy.init_node('waypoint_generator')
    package_name = "tianracer_gazebo"

    # Get the package path
    try:
        pkg_path = rospkg.RosPack().get_path(package_name)

        # Construct the path to scripts directory
        filename= os.path.join(pkg_path, f"scripts/waypoint_race/points.yaml")
    except rospkg.ResourceNotFound:
        rospy.logerr("Package '%s' not found" % package_name)
        exit(1)

    filename = rospy.get_param("~filename", filename)
    print(f"yaml: {filename}")
    g = WaypointGenerator(filename)
    rospy.loginfo('Initialized, use 2D Nav Goal to generate waypoints in your map')
    rospy.logwarn('press Ctrl+C to save!')
    g.spin()
    rospy.loginfo('File generated')
