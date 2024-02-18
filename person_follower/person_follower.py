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

    def listener_callback(self, input_msg):
        angle_min = input_msg.angle_min
        angle_max = input_msg.angle_max
        angle_increment = input_msg.angle_increment
        ranges = input_msg.ranges
        #
        # your code for computing vx, wz
        #
        #vx = 0.
        #wz = 0.

        closest_range = min(ranges)
        closest_index = ranges.index(closest_range)

        #
        min_range = 0.2  
        max_range = 2.0  
        follow_distance = 1.0  
        speed_factor = 0.5  # velocidad lineal vx
        turn_factor = 1.0  # velocidad angular wz

        
        if closest_range > max_range or closest_range < min_range:
            vx = 0.0  # si el objeto esta fuera del rango o demasiado cerca
        else:
            vx = max(0.0, min(speed_factor * (closest_range - follow_distance), speed_factor))

        # posicion angular del punto más cercano
        angle_to_closest = angle_min + closest_index * angle_increment

        # Ajuste de la velocidad angular basado en la posición angular del punto más cercano
        # Convertir ángulo a una dirección relativa (-1 a 1, donde 0 es directamente adelante)
        if angle_to_closest > 0:
            direction = (angle_to_closest - angle_min) / (angle_max - angle_min) * 2 - 1
        else:
            direction = (angle_to_closest - angle_max) / (angle_max - angle_min) * 2 + 1

        wz = direction * turn_factor
        #

        output_msg = Twist()
        output_msg.linear.x = vx
        output_msg.angular.z = wz
        self.publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    person_follower = PersonFollower()
    rclpy.spin(person_follower)
    person_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
