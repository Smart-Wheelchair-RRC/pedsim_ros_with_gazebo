#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Quaternion, Pose, Twist, Vector3, TransformStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from pedsim_msgs.msg import AgentStates, AgentState
import tf2_ros
from nav_msgs.msg import Odometry
import tf2_geometry_msgs

from message_filters import TimeSynchronizer, Subscriber


def transform_pose(input_pose, source_frame="gazebo", target_frame="base_link"):
    try:
        transform: TransformStamped = tfBuffer.lookup_transform(
            target_frame, source_frame, rospy.Time(0))
        output_pose = tf2_geometry_msgs.do_transform_pose(
            input_pose, transform)
        return output_pose
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr("Error transforming pose: {}".format(e))
        return None


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
        # print(agent)
        marker.pose = transform_pose(agent).pose

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


if __name__ == '__main__':
    rospy.init_node('agent_states_to_markers')

    global tfBuffer
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(1)
    odom_sub = Subscriber("/odom", Odometry)
    agent_states_sub = Subscriber(
        "/pedsim_simulator/simulated_agents", AgentStates)
    marker_array_pub = rospy.Publisher(
        "/marker", MarkerArray, queue_size=10)

    ts = TimeSynchronizer([agent_states_sub, odom_sub], 10, 0.1)
    ts.registerCallback(agent_states_callback)

    rospy.spin()
