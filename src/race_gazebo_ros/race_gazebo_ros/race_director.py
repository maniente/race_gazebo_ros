import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library


class RaceDirector(Node):

    def __init__(self):
        super().__init__('race_director')
       
        for id in range(3):
            self.create_subscription(
                    Bool,
                    f'/lane{id}/touched',
                    lambda msg: self.add_to_ranking(msg, id),
                    10)
        
        self.create_subscription(
                    Image,
                    '/world/Myworld/model/realsense_d435/link/link/sensor/realsense_d435/image',
                    self.show_image,
                    10)
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.position = 0
        self.flag_photofinish = False
        self.flag_arrived = [False,False,False]

    def add_to_ranking(self,msg,id):
        self.get_logger().info(f'Received: {id}, {msg}')
        if not self.flag_arrived[id] and msg:
            self.flag_arrived[id]= True
            self.position += 1
            self.get_logger().info("{}: vehicle{}".format(self.position,id))

    def show_image(self, image):
        
        if not self.flag_photofinish and self.position == 1:
            self.flag_photofinish = True

            # Display the message on the console
            self.get_logger().info('Receiving video frame')
            # Convert ROS Image message to OpenCV image
            current_frame = self.br.imgmsg_to_cv2(image,"bgr8")
            
            # Display image
            cv2.imshow("camera", current_frame)
            
            cv2.waitKey(0)

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