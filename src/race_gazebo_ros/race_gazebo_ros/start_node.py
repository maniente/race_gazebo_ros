# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import random
import rclpy 
from rclpy.node import Node

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class StartNode(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.start_publishers = []

        self.create_subscription(
            Int32,
            '/keyboard/keypress',
            self.listener_callback,
            10)
        
        for i in range(3):
            self.start_publishers.append(self.create_publisher(Twist, f'/model/vehicle_{i}/cmd_vel', 10))

    def listener_callback(self, msg):
        # if 's' is pressed
        if(msg.data == 83):
            for start_publisher in self.start_publishers:
                twist = Twist()
                twist.linear.x = 5.0 + (1.0 - 2.0 * random.random())
                start_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = StartNode()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
