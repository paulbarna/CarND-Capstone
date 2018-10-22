#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import math

'''
This node will publish waypoints from the car's current position
to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version
which does not care about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status
of traffic lights too.

Please note that our simulator also provides the exact location of
traffic lights and their current status in `/vehicle/traffic_lights` message.
You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100     # Number of waypoints we will publish
MAX_DECEL = 1.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        # TODO: Add a subscriber for /obstacle_waypoint

        self.final_waypoints_pub = rospy.Publisher('final_waypoints',
                                                    Lane,
                                                    queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1
        self.vLong = None
        self.gLong_rate = None
        self.loop()

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                # Get closest waypoint
                # closest_waypoint_idx = self.get_closest_waypoint_idx()
                # self.publish_waypoints(closest_waypoint_idx)
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind the vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        now = rospy.Time.now()
        rospy.loginfo("log wp current time %f", now.to_sec())
        base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]
        base_waypoints[0].pose.header.stamp = rospy.Time.now()
        
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane
 
    def decelerate_waypoints(self, waypoints, closest_idx):
        # Don't modify base waypoints.
        temp = []
        for i, wp in enumerate(waypoints):

            p = Waypoint()
            p.pose = wp.pose

	    # -2 so the front of the car stop at the line, 
            # instead of the center of the car
            
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            # prevent car from moving again after stopping at stop_idx
            i = min(i, stop_idx)
	    # if distance is less than 30 waypoints, set the traget velocity proportioanlly to the gLong rate
            dist = self.distance(waypoints, i, stop_idx)
            if dist < 30:
                if self.gLong_rate == None:
                    self.gLong_rate = self.vLong / dist
                vel = math.sqrt(2 * MAX_DECEL* self.gLong_rate * dist)
                if vel < 1.:
                    vel = 0.
                vel=min(vel, wp.twist.twist.linear.x)
            else:
                vel = wp.twist.twist.linear.x
   
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        #rospy.loginfo("old waypoint time: %i %i", waypoints.header.stamp.secs, waypoints.header.stamp.nsecs)
        #rospy.loginfo("new time: %i %i", temp.header.stamp.secs, temp.header.stamp.nsecs)
        now = rospy.Time.now()
        
        return temp
    
    def pose_cb(self, msg):
        self.pose = msg

    def velocity_cb(self, velocity):
        self.vLong = velocity.twist.linear.x

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x,
                                  waypoint.pose.pose.position.y
                                 ]
                                for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message.
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
