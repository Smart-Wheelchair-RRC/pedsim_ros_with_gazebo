#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from pedsim_msgs.msg import AgentStates
from nav_msgs.msg import Odometry

from message_filters import TimeSynchronizer, Subscriber


def agent_states_callback(data: AgentStates, odom: Odometry):
    marker_array = MarkerArray()
    marker_id = 0

    if odom is None:
        return

    for agent in data.agent_states:
        marker = Marker()
        marker.header = data.header
        marker.header.frame_id = "base_link"
        marker.ns = "agents"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose = agent.pose
        marker.pose.position.x -= odom.pose.pose.position.x
        marker.pose.position.y -= odom.pose.pose.position.y
        marker.pose.position.z -= odom.pose.pose.position.z

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Transform the agent's pose from gazebo to base_link
        # agent_pose_transformed = transform_pose(trans, agent.pose)
        # marker.pose = agent_pose_transformed

        marker_array.markers.append(marker)
        marker_id += 1

    marker_array_pub.publish(marker_array)


if __name__ == "__main__":
    rospy.init_node("agent_states_to_markers")

    rospy.sleep(1)
    odom_sub = Subscriber("/pedsim_simulator/robot_position", Odometry)
    agent_states_sub = Subscriber("/pedsim_simulator/simulated_agents", AgentStates)
    marker_array_pub = rospy.Publisher("/marker", MarkerArray, queue_size=10)

    ts = TimeSynchronizer([agent_states_sub, odom_sub], 10, 0.1)
    ts.registerCallback(agent_states_callback)

    rospy.spin()
