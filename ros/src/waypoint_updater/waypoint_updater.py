#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
from std_msgs.msg import Int32
from tf.transformations import euler_from_quaternion

import math
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        ### Subscribers
        # all waypoints of the track before and after the car
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)


        # for debugging and testing
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_gt_cb)
        self.traffic_gt = None

        ### Publishers
        # publish a fixed number of waypoints ahead of the car starting with the first point ahead of the car
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        ### Member variables
        self.base_waypoints = None
        self.final_waypoints = None
        self.velocity_cb_state = False 
        self.pose_cb_state = False 

        ## Main Loop    
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            if self.base_waypoints and self.velocity_cb_state and self.pose_cb_state:
                self.closest_wp = self.closest_node()
                self.get_waypoints()
                self.publish_final_waypoints()
            rate.sleep()

    def traffic_gt_cb(self, msg):
        self.traffic_gt = msg
        self.traffic_gt.lights[0].state = 2

    def velocity_cb(self, msg):
        # obtain the current velocity
        self.velocity_cb_state = True 
        self.current_linear_velocity = msg.twist.linear.x
        self.current_angular_velocity = msg.twist.angular.z

    def pose_cb(self, msg):
        # obtain the current pose
        self.pose_cb_state = True 
        self.position = msg.pose.position
        self.orientation = msg.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([
            self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])
        self.yaw = yaw

    def get_waypoints(self):
        self.final_waypoints = []
        for i in range(LOOKAHEAD_WPS):
            wp_index = (self.closest_wp + i) % len(self.base_waypoints.waypoints)
            dist, angle = self.get_next_target(self.base_waypoints.waypoints[wp_index])
            if self.current_linear_velocity < 5.0 :
                l_vel =  self.current_linear_velocity + 0.5
            else:
                l_vel = 5.0

            a_vel = angle * dist / l_vel;
            self.set_waypoint_linear_velocity(self.base_waypoints.waypoints[wp_index], l_vel)
            self.set_waypoint_angular_velocity(self.base_waypoints.waypoints[wp_index], a_vel)


            if self.traffic_gt.lights[0].state == 2:
                self.final_waypoints.append(self.base_waypoints.waypoints[wp_index])

    def publish_final_waypoints(self):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, waypoints):

        self.base_waypoints = waypoints
        # we only need the message once, unsubscribe after first receive
        self.base_waypoints_sub.unregister()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_next_target(self, waypoint):
        # convert to local coordinates
        vx = waypoint.pose.pose.position.x - self.position.x
        vy = waypoint.pose.pose.position.y - self.position.y
        lx = vx * np.cos(self.yaw) + vy * np.sin(self.yaw)
        ly = -vx * np.sin(self.yaw) + vy * np.cos(self.yaw)
        dist = math.sqrt(lx * lx + ly * ly)
        angle = np.arctan2(ly, lx)
        return dist, angle

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_linear_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity

    def set_waypoint_angular_velocity(self, waypoint, angle):
        waypoint.twist.twist.angular.z = angle

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def closest_node(self):

        # current position
        cur_pos_x = self.position.x
        cur_pos_y = self.position.y

        # we can assume the track waypoints are already in a cyclic order
        cur_o = self.orientation
        cur_q = (cur_o.x,cur_o.y,cur_o.z,cur_o.w)
        cur_roll, cur_pitch, cur_yaw = euler_from_quaternion(cur_q)

        closest_dist = 999999.
        closest_wp = None

        wp_len = len(self.base_waypoints.waypoints)  
        for i in range(wp_len):
            base_wp_x = self.base_waypoints.waypoints[i].pose.pose.position.x
            base_wp_y = self.base_waypoints.waypoints[i].pose.pose.position.y
            dist = math.sqrt(math.pow(cur_pos_x - base_wp_x, 2) + math.pow(cur_pos_y - base_wp_y, 2))
            if dist < closest_dist:
                closest_dist = dist
                closest_wp = i

        #Check if waypoint is ahead of vehicle
        closest_wp_x = self.base_waypoints.waypoints[closest_wp].pose.pose.position.x
        closest_wp_y = self.base_waypoints.waypoints[closest_wp].pose.pose.position.y
        dist_ahead = ((closest_wp_x - cur_pos_x)* math.cos(cur_yaw) +
                  (closest_wp_y - cur_pos_y)* math.sin(cur_yaw)) > 0.0

        if not dist_ahead:
            closest_wp = (closest_wp + 1) % wp_len

        return closest_wp

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
