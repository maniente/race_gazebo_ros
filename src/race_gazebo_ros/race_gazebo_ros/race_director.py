import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from datetime import datetime


class RaceDirector(Node):

    def __init__(self):
        super().__init__('race_director')
       
        for id in range(3):
            self.create_subscription(
                    Bool,
                    f'/lane{id}/touched',
                    lambda msg, id=id: self.add_to_ranking(msg, id),
                    10)
        
        self.create_subscription(
                    Image,
                    '/world/Myworld/model/realsense_d435/link/link/sensor/realsense_d435/image',
                    self.show_image,
                    10)
        
        self.create_subscription(
            Int32,
            '/keyboard/keypress',
            self.start_timer,
            10)
        
        self.time_zero = datetime.now()
        self.flag_start = False
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.position = 0
        self.flag_photofinish = False
        self.flag_arrived = [False,False,False]

    def add_to_ranking(self,msg,id):
        #self.get_logger().info(f'Received: {id}, {msg}')
        if not self.flag_arrived[id] and msg:
            self.flag_arrived[id]= True
            self.position += 1
            lap_time = datetime.now()-self.time_zero
            self.get_logger().info("{}: vehicle{} lap time : {}".format(self.position,id,lap_time))

    def show_image(self, image):
        
        if not self.flag_photofinish and self.position >= 1:
            self.flag_photofinish = True

            # convert ros image to opencv image in mat format
            current_frame = cv2.cvtColor(self.br.imgmsg_to_cv2(image), cv2.COLOR_BGR2RGB)

            # Save the image
            cv2.imwrite("photo_finish.jpg", current_frame)

    def start_timer(self,msg):
        if(msg.data == 83 and not self.flag_start):
            self.flag_start = True
            self.time_zero = datetime.now()


        

def main(args=None):
    rclpy.init(args=args)

    race_director = RaceDirector()

    rclpy.spin(race_director)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    race_director.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()