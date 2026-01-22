import rclpy, cv2, cv_bridge, numpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.get_logger().info("Start line follower.")

        self.bridge = cv_bridge.CvBridge()
        
        # 声明参数，默认颜色为black
        self.declare_parameter('line_color', 'black')

        # 获取参数设置的颜色
        self.color_name = self.get_parameter('line_color').get_parameter_value().string_value
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub = self.create_publisher(Image, 'camera/process_image', 10)

        self.twist = Twist()

        # 常见颜色的HSV范围
        self.color_ranges = {
            'yellow': (numpy.array([ 10,  70, 30]), numpy.array([ 40, 255, 250])),
            'red':    (numpy.array([ 0,  70, 50]), numpy.array([ 10, 255, 255])),
            'green':  (numpy.array([ 40,  70, 50]), numpy.array([ 80, 255, 255])),
            'blue':   (numpy.array([100,  70, 50]), numpy.array([140, 255, 255])),
            'white':  (numpy.array([ 0,   0,200]), numpy.array([180, 30, 255])),
            'black':  (numpy.array([ 0,   0,  0]), numpy.array([180, 255,  50]))
        }

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower, upper = self.color_ranges.get(self.color_name, self.color_ranges['yellow'])
        mask = cv2.inRange(hsv, lower, upper)

        h, w, d = image.shape
        # search_top = int(h/2)
        # search_bot = int(h/2 + 20)

        search_top = int(h/2) + 120
        search_bot = int(h/2 + 150)
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)

        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

            # 基于检测的目标中心点，计算机器人的控制参数
            err = cx - w/2
            self.twist.linear.x = 0.1
            self.twist.angular.z = -float(err) / 400
            self.cmd_vel_pub.publish(self.twist)
            
        self.pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)    
    follower = Follower()
    rclpy.spin(follower)
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()