#! /usr/bin/env python
import rospy 
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
"""
^Cx: 0.0
y: 0.0
z: 0.0
!!!!!
x: -1.428780526600578
y: -1.8670595822647775
z: 0.005000608851099014
!!!!!
x: -0.8541378373856054
y: -0.57080969486389
z: 0.00999173701919729
!!!!!
----------


t1 


----------
x: 0.0
y: 0.0
z: 0.0
!!!!!
x: -1.422532436312003
y: -1.8682248254146507
z: 0.004994101676492821
!!!!!
x: -1.915068709596355
y: -0.5493092660435841
z: 0.019212890862272014





"""
ana_check1 = ( (-0.8541378373856054,-0.57080969486389) , (-1.915068709596355,-0.5493092660435841))
ana_check2 = ( (-1.0395616340369942, -7.876568553760501), (-1.0041087134758948,-8.944819502545196))
ana_check3 = ( (-0.5749616124606485 ,-0.9118791656634234), (0.42296276126638904 ,-1.0242940945768384))

ana_cnt = 0
ana_check_list = [ana_check1, ana_check2, ana_check3]
ana_history_position= (0,0)
def is_intersect(line1, line2):  
    """  
    判断两个线段是否相交  
    line1、line2：线段的两个端点，每个端点是一个二元组，如((-0.8541378373856054,-0.57080969486389), (-1.915068709596355,-0.5493092660435841))  
    返回值：如果相交返回True，否则返回False  
    """  
    x1, y1 = line1[0]  
    x2, y2 = line1[1]  
    x3, y3 = line2[0]  
    x4, y4 = line2[1]  
      
    # 判断两条线段是否平行  
    if (y4 - y3) * (x2 - x1) == (y2 - y1) * (x4 - x3):  
        return False  
      
    # 判断点是否在线段上  
    def is_on_segment(x, y, x1, y1, x2, y2):  
        return min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2)  
      
    # 判断线段是否相交  
    ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))  
    ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))  
    if 0 <= ua <= 1 and 0 <= ub <= 1:  
        return True  
    elif is_on_segment(x1, y1, x3, y3, x4, y4) or is_on_segment(x2, y2, x3, y3, x4, y4):  
        return True  
    else:  
        return False

def analysis(data):
    global ana_history_position,ana_cnt, ana_check_list
    data = data.pose
    now_position = data[2].position
    x = now_position.x
    y = now_position.y
    now_line = (ana_history_position, (x,y))
    ana_history_position = (x, y)
    # print(history_position)
    tmp = ana_cnt % 3
    if is_intersect(now_line, ana_check_list[tmp]):
        print('已经通过' + str(tmp) +'点')
        ana_cnt +=1 
        return '已经通过' + str(tmp) +'点', ana_cnt-1
    return None
    # print('----------')
if __name__ == "__main__":
    try : 
        print('starting')
        rospy.init_node("analysis")
        states = rospy.Subscriber("/gazebo/model_states", ModelStates, callback=analysis)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
