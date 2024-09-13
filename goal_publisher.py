#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped


class GoalPublisher:
    def __init__(self):
        rospy.init_node("goal_publisher", anonymous=True)

        self.goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.reached_goal_subscriber = rospy.Subscriber("/reached_goal", Empty, self.reached_goal_callback)

        self.coordinates = [
            # (14.78, -0.78),
            # (17.07, -8.86),
            # (11.18, -3.37),
            (12.45, 6.57),
            (9.46, -0.88),
            (11.56, -10.03),
            (16.93, 0.05),
            (3.03, -2.26),
            (11.99, 6.50),
            (13.09, -13.05),
        ]
        self.current_goal_index = 0
        self.is_publishing = False
        self.debounce_duration = rospy.Duration(5.0)
        self.last_publish_time = rospy.Time.now()

    def reached_goal_callback(self, msg):
        if self.is_publishing:
            return

        current_time = rospy.Time.now()
        if current_time - self.last_publish_time < self.debounce_duration:
            return

        self.is_publishing = True
        self.publish_next_goal()
        self.is_publishing = False
        self.last_publish_time = current_time

    def publish_next_goal(self):
        if self.current_goal_index >= len(self.coordinates):
            rospy.loginfo("All goals have been reached!")
            return

        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"

        x, y = self.coordinates[self.current_goal_index]
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        self.goal_publisher.publish(goal_msg)
        rospy.loginfo(f"Published goal {self.current_goal_index + 1}: ({x}, {y})")
        self.current_goal_index += 1

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        goal_publisher = GoalPublisher()
        goal_publisher.run()
    except rospy.ROSInterruptException:
        pass
