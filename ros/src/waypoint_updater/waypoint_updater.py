#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from tf.transformations import euler_from_quaternion

import math

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

LOOKAHEAD_WPS = 30 # Number of waypoints we will publish. You can change this number

def get_final_wp(self, cl_wp):

	lane = Lane()
	#lane.header = self.base_waypoints.header

	final_wp = []
	i = 0
	while(i<LOOKAHEAD_WPS):
		pos_ = (cl_wp+i)%(len(self.base_waypoints))
		final_wp.append(self.base_waypoints[pos_])
		i += 1

	lane.waypoints = final_wp

	return lane

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
	
		### Publishers
		# publish a fixed number of waypoints ahead of the car starting with the first point ahead of the car
		self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
		
		### Member variables
		self.current_pose = PoseStamped()	# PoseStamped
		self.base_waypoints = Lane()	# Lane
		self.final_waypoints = Lane()	# Lane
		self.current_velocity = TwistStamped()	# TwistStamped

		rospy.spin()

	
	
	def velocity_cb(self, msg):

		self.current_velocity = msg

	def pose_cb(self, msg):

		# TODO: Implement
		# obtain the current pose
		self.current_pose = msg

		# for debugging
		# this should in theory accelerate the car in a straight line from it's current position
		waypoints_final = [Waypoint() for i in range(LOOKAHEAD_WPS)]

		for i in range(LOOKAHEAD_WPS):

			waypoints_final[i].pose = self.current_pose
			waypoints_final[i].pose.pose.position.x += (i+1)*10.
			waypoints_final[i].twist = self.current_velocity
			waypoints_final[i].twist.twist.linear.x += (i+1)*10.

		self.final_waypoints.waypoints = waypoints_final
		self.final_waypoints_pub.publish(self.final_waypoints)

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

	def get_waypoint_velocity(self, waypoint):
		return waypoint.twist.twist.linear.x

	def set_waypoint_velocity(self, waypoints, waypoint, velocity):
		waypoints[waypoint].twist.twist.linear.x = velocity

	def distance(self, waypoints, wp1, wp2):
		dist = 0
		dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
		for i in range(wp1, wp2+1):
			dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
			wp1 = i
		return dist

	def closest_node(self):

		# current position
		cur_pos_x = self.current_pose.position.x
		cur_pos_y = self.current_pose.position.y

		# we can assume the track waypoints are already in a cyclic order
		cur_o = self.current_pose.orientation
		cur_q = (cur_o.x,cur_o.y,cur_o.z,cur_o.w)
		cur_roll, cur_pitch, cur_yaw = euler_from_quaternion(cur_q)

		closest_dist = float("inf")
		closest_wp = 0

		for i in range(len(self.base_waypoints)):

		    base_wp_x = self.base_waypoints[i].pose.pose.position.x
		    base_wp_y = self.base_waypoints[i].pose.pose.position.y
		    dist = math.sqrt(math.pow(cur_pos_x-base_wp_x, 2) + math.pow(cur_pos_y-base_wp_y, 2))
		    if dist < closest_dist:
				closest_dist = dist
				closest_wp = i

		# TODO: modify and complete this function
		#Check if waypoint is ahead of vehicle
		dist_ahead = ((base_wp_x - cur_pos_x)* math.cos(cur_yaw)+
			      (base_wp_y - cur_pos_y)* math.sin(cur_yaw)) > 0.0
		# if not dist_ahead:
		#TODO: use following waypoint if behind vehicle

		return closest_wp


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
