#!/usr/bin/env python

import rospy
import math
import numpy
 # test lidar data process to detect opponent robot
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Pose2D




def distance(self,x, y, angle):


    """Calculates the distance to the edge of the plateau from a given position and angle.

    Args:
        x: The x-coordinate of the position.
        y: The y-coordinate of the position.
        angle: The angle in degrees, where 0 degrees is north, 90 degrees is east, 180 degrees is south, and 270 degrees is west.

    Returns:
        The distance to the edge of the plateau.
    """
    y_plateau=2
    x_plateau=3

    # Convert the angle to radians.
    angle_radians = angle * math.pi / 180

    # Calculate the distance to the edge of the plateau in each direction.
    distance_north = (y_plateau - y) / math.cos(angle_radians)
    distance_east = (x_plateau - x) / math.cos(math.pi / 2 - angle_radians)
    distance_south = (y - 0) / math.cos(angle_radians - math.pi / 2)
    distance_west = (x - 0) / math.cos(2 * math.pi - angle_radians)

    # Find the shortest distance to the edge of the plateau.
    distance = min(distance_north, distance_east, distance_south, distance_west)

    return distance

def def_CO(xR, yR, theta, D_L):
    """Calculates the coordinates of the laser in the game board's frame of reference.

    Args:
        xR: The x-coordinate of the robot.
        yR: The y-coordinate of the robot.
        theta: The angle of the robot in degrees.
        D_L: The distance to the laser in millimeters.

    Returns:
        The coordinates of the laser in the game board's frame of reference.
    """

    # Convert the angle to radians.
    theta = theta * math.pi / 180

    # Convert the distances from meters to millimeters.
    xR *= 1000
    yR *= 1000
    D_L *= 1000

    # Calculate the coordinates of the laser in the robot's frame of reference.
    xE_R = D_L * math.cos(theta)
    yE_R = D_L * math.sin(theta)

    # Calculate the coordinates of the laser in the game board's frame of reference.
    xE = xR + xE_R
    yE = yR + yE_R

    return [xE, yE]

def position_robot(liste, angle):
    """Calculates the position of the robot.

    Args:
        liste: A list of sensor readings.
        angle: The angle of the robot.

    Returns:
        A tuple containing the distance of the robot, the angle of the robot,
        the middle index of the sensor readings, the start index of the sensor
        readings, and the end index of the sensor readings.
    """

    stop = 0
    test = 0
    robot1 = []

    # Find the start and end indices of the non-None sensor readings.
    start_index = None
    end_index = None
    for i in range(len(liste)):
        if liste[i] is not None:
            if start_index is None:
                start_index = i
            end_index = i
        elif start_index is not None:
            break

    # If there are no non-None sensor readings, return None.
    if start_index is None or end_index is None:
        return None, None, None, None, None

    # Calculate the distance and angle of the robot.
    distance = sum(liste[start_index:end_index + 1]) / (end_index - start_index + 1)
    angle_robot1 = (end_index + start_index) // 2 * 0.7894 + angle

    return distance, angle_robot1, (end_index + start_index) // 2, start_index, end_index

def remplacement(liste):
    """Replaces None values in a list with the previous value.

    Args:
        liste: A list of values.

    Returns:
        A list with the None values replaced with the previous value.
    """

    previous_value = None
    for i in range(len(liste)):
        if liste[i] is None:
            liste[i] = previous_value
        else:
            previous_value = liste[i]
    return liste

def angle_mort(liste):
    """Sets the values of the angle mort to None.

    Args:
        liste: A list of sensor readings.

    Returns:
        A list of sensor readings with the values of the angle mort set to None.
    """

    debut_angle_mort = 210
    fin_angle_mort = 242

    for i in range(debut_angle_mort, fin_angle_mort + 1):
        liste[i] = None

    return liste






class RobotPositionDetector:
    def __init__(self):
        rospy.init_node('robot_position_detector')

        # Initialize a publisher to publish the detected adversarial robot position as PointStamped
        self.pub_pos_rob_opp = rospy.Publisher('/pos_rob_opp', PointStamped, queue_size=10)

        # Initialize variables to store LiDAR data and robot's own position (x, y, theta)
        self.laser_data = None
        self.own_position = Pose2D()

        # Subscribe to the /scan topic to get LiDAR data
        rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback)

        # Subscribe to the /pos_rob topic to get the robot's own position (x, y, theta)
        rospy.Subscriber('/pos_rob', Pose2D, self.robot_position_callback)

    def laser_scan_callback(self, data):
        # Store LiDAR data
        self.laser_data = data

    def robot_position_callback(self, data):
        # Store the robot's own position (x, y, theta)
            self.own_position = data
            
    def process_lidar_data(self):
        if self.laser_data is not None:
            # Process LiDAR data here to determine the position of the adversarial robot (x, y)

            # First, apply the angle mort to the LiDAR data.
            liste_data = angle_mort(list(self.laser_data.ranges))

            # Then, calculate the maximum distance to the edge of the plateau from each angle.
            tab_dist = []
            for i in range(len(liste_data)):
                L_max = distance(self.x_robot, self.y_robot, self.teta_robot + i * 0.7894)
                tab_dist.append(L_max)

            # Then, remove any distances that are greater than the maximum distance to the edge of the plateau.
            for i in range(len(liste_data)):
                if liste_data[i] is not None and liste_data[i] > tab_dist[i]:
                    liste_data[i] = None

            # Then, replace any None values in the list of distances with the previous value.
            liste_data = remplacement(liste_data)

            # Then, find the middle index of the list of distances.
            pos_mid = position_robot(liste_data, self.teta_robot)

            # Finally, calculate the final position of the adversarial robot.
            pos_final = def_CO(self.x_robot, self.y_robot, pos_mid[1], pos_mid[0])

            # Create a PointStamped message to represent the adversarial robot's position
            adversarial_robot_position = PointStamped()
            adversarial_robot_position.header.stamp = rospy.Time.now()
            adversarial_robot_position.header.frame_id = "base_link"  # Adjust the frame ID as needed
            adversarial_robot_position.point.x = pos_final[0]
            adversarial_robot_position.point.y = pos_final[1]

            # Publish the detected adversarial robot position
            self.pub_pos_rob_opp.publish(adversarial_robot_position)




    def run(self):
            rate = rospy.Rate(10)  # 10 Hz
            while not rospy.is_shutdown():
                self.process_lidar_data()
                rate.sleep()

if __name__ == '__main__':
    try:
        robot_detector = RobotPositionDetector()
        robot_detector.run()
    except rospy.ROSInterruptException:
        pass
