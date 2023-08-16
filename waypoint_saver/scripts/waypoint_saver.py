#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import csv
from geometry_msgs.msg import PoseWithCovarianceStamped

class OdomToCSV:
    def __init__(self):
        rospy.init_node('waypoints_saver', anonymous=True)
        self.odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.odom_callback)
        self.csv_filename = '/home/lishuai/catkin_ws/src/caurobot_simulation/waypoint_loader/waypoints/waypoints.csv'
        self.csv_file = open(self.csv_filename, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        #self.csv_writer.writerow(['Timestamp', 'Position_X', 'Position_Y', 'Orientation_Z', 'Orientation_W'])

        # Set the save interval to 1 second (1.0 second)
        self.save_interval = rospy.Duration(0.2)
        self.last_save_time = rospy.Time.now()

    def odom_callback(self, odom_msg):
        current_time = rospy.Time.now()
        time_since_last_save = current_time - self.last_save_time

        if time_since_last_save >= self.save_interval:
            self.save_odom_data(odom_msg)
            self.last_save_time = current_time

    def save_odom_data(self, odom_msg):
        timestamp = odom_msg.header.stamp.to_sec()
        position_x = odom_msg.pose.pose.position.x
        position_y = odom_msg.pose.pose.position.y
        orientation_z = odom_msg.pose.pose.orientation.z
        orientation_w = odom_msg.pose.pose.orientation.w
        print("pose x:",position_x)
        print("pose y:",position_y)
        self.csv_writer.writerow([position_x, position_y,0])
        self.csv_file.flush()

    def run(self):
        rospy.spin()
        self.csv_file.close()

if __name__ == '__main__':
    odom_to_csv = OdomToCSV()
    odom_to_csv.run()
