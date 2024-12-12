#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import csv
import numpy as np

set_lookahead = 1.3

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Used to publish point creation
        self.pub_env_viz = self.create_publisher(Marker, '/env_viz', 10)
        self.pub_dynamic_viz = self.create_publisher(Marker, '/dynamic_viz', 10)
        
        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.sub_odom = self.create_subscription(Odometry, '/ego_racecar/odom', self.pose_callback, 10)

        self.x_coordinates = []
        self.y_coordinates = []
        self.orien = []
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.id = 0

        # Visualization for way points
        # Marker set to points
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        # Small size
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1

        # Green points
        self.marker.color.a = 0.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0


        path = '/home/mjdolan03/pure_pursuit_ws/src/pure_pursuit_pkg/pure_pursuit_pkg/data.csv'
        with open(path, 'r') as file:
            reader = csv.reader(file)
            # Iterates through rows of file
            for row in reader:
                # Converts the 3 string values to floating point numbers
                x, y, orien = map(float, row)

                # Adds the x, y, and orien values to their corresponding lists
                self.x_coordinates.append(x)
                self.y_coordinates.append(y)
                self.orien.append(orien)

                # Point object for visualization
                point = Point()
                point.x = x
                point.y = y
                # 2-D
                point.z = 0.0
                
                # Adds the point object to points list for the marker so it can be visualized
                self.marker.points.append(point)

        # Initialize values
        self.flag = False
        self.curr_index = 0
        self.x_current = 0.0
        self.y_current = 0.0
        self.orien_current = 0.0
        self.angle = 0.0

    def pose_callback(self, odometry_info):
        # Current x and y coordinates of car
        self.x_current = odometry_info.pose.pose.position.x
        self.y_current = odometry_info.pose.pose.position.y

        # w * z + x * y
        sin_calc = odometry_info.pose.pose.orientation.w * odometry_info.pose.pose.orientation.z + odometry_info.pose.pose.orientation.x * odometry_info.pose.pose.orientation.y
        
        # y * y + z * z
        cos_calc = odometry_info.pose.pose.orientation.y * odometry_info.pose.pose.orientation.y + odometry_info.pose.pose.orientation.z * odometry_info.pose.pose.orientation.z

        # Converting quaternion to yaw: quaternion form (w, x, y, z)
        yaw_sin = 2.0 * sin_calc
        yaw_cos = 1.0 - 2.0 * cos_calc
        # Computes orientation based on calculated yaw
        self.orien_current = np.arctan2(yaw_sin, yaw_cos)

        # Runs through this portion to set flag to true for finding close waypoint
        if self.flag == False:
            close = 75.0
            for i in range(len(self.x_coordinates)):
                # Calculates current distance: (x2 - x1)^2 + (y2 - y1)^2
                curr_distance = (self.x_coordinates[i] - self.x_current) ** 2 + (self.y_coordinates[i] - self.y_current) ** 2
                # If current distance is closer than shortest distance
                if curr_distance < close:
                    # Sets shortest distance to current distance
                    close = curr_distance
                    # Sets current index as the corresponding shortest distance index
                    self.curr_index = i
            self.flag = True
        # Iterates through current indexes while distance is less than the lookahead distance set above
        # Current index represents target point and this makes sure its far enough ahead of car in terms of distance
        # Will increment target point until while conditional is false: target point distance > lookahead distance
        while True:
            target_distance = np.sqrt((self.x_coordinates[self.curr_index] - self.x_current) ** 2 +(self.y_coordinates[self.curr_index] - self.y_current) ** 2)
            if target_distance > set_lookahead:
                break 
            # Increments and accounts for wrap around
            self.curr_index = (self.curr_index + 1) % len(self.x_coordinates)


        # Calculates lookahead angle based on current position to target point
        lookahead_ang = np.arctan2(self.y_coordinates[self.curr_index] - self.y_current, self.x_coordinates[self.curr_index] - self.x_current)

        # Calculates distance between current position and target point
        # Target point is x/y-coords indexed to target point and current is subtracted for distance calculation
        calc_distance = np.sqrt((self.x_coordinates[self.curr_index] - self.x_current) ** 2 + (self.y_coordinates[self.curr_index] - self.y_current) ** 2)

        # For steering angle calculation: how far car is from ideal y orientation to target (y-axis perpindicular to car)
        y_val = calc_distance * np.sin(lookahead_ang - self.orien_current)


        # Steering angle given formula: 2 * abs_y / L^2
        self.angle = 2.0 * y_val / (calc_distance ** 2)
        
        # Visualization marker for waypoints I'm picking
        point = Point()
        waypoint_mark = Marker()
        # Sets target points
        point.x = self.x_coordinates[self.curr_index]
        point.y = self.y_coordinates[self.curr_index]
        # 2-D
        point.z = 0.0
        # Adds corresponding point to marker points list
        waypoint_mark.points.append(point)
        waypoint_mark.header.frame_id = "map"
        waypoint_mark.id = 0

        waypoint_mark.type = Marker.POINTS
        waypoint_mark.action = Marker.ADD

        # Marker actions correspond to actual coordinate point values
        waypoint_mark.pose.position.x = 0.0
        waypoint_mark.pose.position.y = 0.0
        waypoint_mark.pose.position.z = 0.0

        # Default
        waypoint_mark.pose.orientation.x = 0.0
        waypoint_mark.pose.orientation.y = 0.0
        waypoint_mark.pose.orientation.z = 0.0
        waypoint_mark.pose.orientation.w = 1.0

        # Point sizes
        waypoint_mark.scale.x = 0.4
        waypoint_mark.scale.y = 0.4

        # Purple points
        waypoint_mark.color.a = 1.0
        waypoint_mark.color.r = 1.0
        waypoint_mark.color.g = 0.0
        waypoint_mark.color.b = 1.0

        self.reactive_control()
        # Pass marker values for visualization
        self.pub_env_viz.publish(self.marker)
        self.pub_dynamic_viz.publish(waypoint_mark)

    def reactive_control(self):
        ackermann_drive_result = AckermannDriveStamped()
        ackermann_drive_result.drive.steering_angle = self.angle
        degree_conversion = np.degrees(abs(self.angle))
        # Sharp turn case
        if degree_conversion > 25:
            # Reduce speed
            ackermann_drive_result.drive.speed = 0.7 
        elif degree_conversion > 8:
            ackermann_drive_result.drive.speed = 1.25 
        else:
            # No existing turn or small turn, fast speed
            ackermann_drive_result.drive.speed = 2.0  
        self.pub_drive.publish(ackermann_drive_result)

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

