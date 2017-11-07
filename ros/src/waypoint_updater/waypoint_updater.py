#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import numpy as np
import matplotlib.pyplot as plt

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        self.base_waypoints = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        # rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        
        # rospy.Subscriber('/vehicle/traffic_lights', )

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_waypoints = []
        self.current_pose  = []

        rospy.spin()

    def pose_cb(self, msg): #msg: Lane

        # current_waypoints 
        self.current_pose = msg # pose message

        if len(self.current_waypoints) > 0:
            wp_next_idx = self.get_next_waypoint(self.current_pose)
            twice_wp = self.current_waypoints + self.current_waypoints

            self.final_waypoints_pub.publish(Lane(None, twice_wp[wp_next_idx:wp_next_idx + 20]))

        # To maintain continuity after a loop finishes

        # self.final_waypoints_pub.publish(waypoint_list)
        # TODO: Implement
        # pass

    def get_next_waypoint(self, pose):
        # find closest next waypoint from a list of base_waypoints w.r.t to car pose
        curr_position = pose.pose.position
        # min_dist = 10000000
        dist = lambda p1, p2: math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)
        # brute force to scan all waypoints
        min_dist_idx = np.argmin(np.abs([dist(curr_position, wp.pose.pose.position) for wp in self.current_waypoints]))

        inc_loop = lambda a: a + 1 if (a + 1) < len(self.current_waypoints) else 0

        # p1 = closest point, p2 = next wp after closest point
        # vector p1 -> p2, & p1 -> car
        wp_vector = lambda wp: np.array([wp.x, wp.y, wp.z])

        p1 = wp_vector(self.current_waypoints[min_dist_idx].pose.pose.position)
        # print("Length c wp:", len(self.current_waypoints))
        # print("min_dist_idx:", min_dist_idx)
        # if min_dist_idx >= len(self.current_waypoints):
        #     p2  = wp_vector(self.current_waypoints[0].pose.pose.position)
        # else:
        #     p2  = wp_vector(self.current_waypoints[min_dist_idx + 1].pose.pose.position)
             

        p2 = wp_vector(self.current_waypoints[inc_loop(min_dist_idx)].pose.pose.position)
        # p2  = wp_vector(self.current_waypoints[0].pose.pose.position))].pose.pose.position)
        car_v = wp_vector(curr_position)



        #
        p1_p2 = p2 - p1 
        p1_car = car_v - p1


        if math.cos(np.dot(p1_p2, p1_car)) >= 0:
            min_dist_idx = inc_loop(min_dist_idx)

        # if min_dist_idx >= len(self.current_waypoints):
        #     min_dist_idx = 0




        return min_dist_idx
        # for wp in current_waypoints:
        #     wp_position = wp.pose.pose.position

        #     curr_dist = dist(curr_position, wp_position)

        #     if min





        # return self.current_waypoints[0:10]


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        return 0





    def waypoints_cb(self, waypoints):
        self.current_waypoints = waypoints.waypoints


        # wp_list = np.array([[wp.pose.pose.position.x, wp.pose.pose.position.y ]
        #                     for wp in self.current_waypoints]) 
        # plt.plot(wp_list[:,0], wp_list[:, 1], "r+")

        # # xy_car = self.current_pose.pose.position
        # # plt.plot(xy_car.x, xy_car.y, "bs")
        # plt.show()

        # unsubscribe from base waypoints
        self.base_waypoints.unregister()

        # TODO: Implement
        # pass

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
