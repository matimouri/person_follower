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

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class PersonFollower(Node):

    def __init__(self):
        super().__init__('person_follower')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        ranges = msg.ranges
        detection_zone = ranges[144:215]

        vx = 0.0
        wz = 0.0

        min_distance = 0.6
        max_distance = 1.4
        forward_speed = 0.2
        rotation_speed = 0.3
        slope_index = 26

        for i, distance in enumerate(detection_zone):
            if min_distance < distance < max_distance:
                vx = 0.06 + (distance / max_distance) * forward_speed / 2
                if i < slope_index:
                    wz = rotation_speed
                elif i > len(detection_zone) - slope_index:
                    wz = -rotation_speed
                break

        velocity_msg = Twist()
        velocity_msg.linear.x = vx
        velocity_msg.angular.z = wz
        self.publisher_.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)
    person_follower = PersonFollower()
    rclpy.spin(person_follower)
    person_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
