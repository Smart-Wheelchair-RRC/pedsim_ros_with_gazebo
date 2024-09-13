#!/usr/bin/env python3
import rospy
import numpy as np
from typing import List, Tuple
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetModelState


class CalculateMetrics:
    def __init__(self, odom_topic="/odom", robot_name="mobile_base") -> None:
        rospy.init_node("calculate_metrics")
        self.odom_topic = odom_topic
        self.robot_name = robot_name
        self.odom: Odometry = None
        self.position_array: List[List[float]] = []
        self.velocity_array: List[List[float]] = []
        self.goal_positions: List[List[float]] = []
        self.robot_positions: List[List[float]] = []
        self.avg_velocities: List[float] = []
        self.distances_travelled: List[float] = []
        self.previous_path_length = 0

        self.tf_buffer = tf2_ros.Buffer()

        self.dynamic_obstacles = np.full((5, 4, 10), 1000, dtype=float)
        self.dynamic_obstacles[:, 2:, :] = 0

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goal_callback)

        rospy.wait_for_service("/gazebo/get_model_state")
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    def update_dynamic_obstacles(self, obs_array: list):
        self.dynamic_obstacles[0:-1] = self.dynamic_obstacles[1:]
        self.dynamic_obstacles[-1, 0:2, :] = self.pipeline.padding_obstacle
        self.dynamic_obstacles[-1, 2:, :] = 0

        obs_array = np.array(obs_array, dtype=float)
        obs_array = obs_array[np.argsort(np.linalg.norm(obs_array[:, :2], axis=-1), axis=-1)][
            : self.pipeline.max_projection_dynamic_obstacles
        ]
        obs_array = obs_array.T

        self.dynamic_obstacles[-1, :, 0 : obs_array.shape[1]] = obs_array

    def odom_callback(self, msg: Odometry) -> None:
        self.odom = msg
        self.position_array.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.velocity_array.append([msg.twist.twist.linear.x, msg.twist.twist.linear.y])

    def goal_callback(self, msg: PoseStamped) -> None:
        self.goal_positions.append([msg.pose.position.x, msg.pose.position.y])

        robot_position = self.get_robot_position()
        self.robot_positions.append(robot_position)

        avg_velocity = self.get_avg_velocity()
        self.avg_velocities.append(avg_velocity)

        total_distance = self.get_path_length() - self.previous_path_length
        self.previous_path_length = total_distance
        self.distances_travelled.append(total_distance)

        rospy.loginfo(f"New goal received: x={msg.pose.position.x}, y={msg.pose.position.y}")
        rospy.loginfo(f"Robot position: x={robot_position[0]}, y={robot_position[1]}")
        rospy.loginfo(f"Average velocity: {avg_velocity} m/s")
        rospy.loginfo(f"Total distance travelled: {total_distance} m")

    def get_robot_position(self) -> Tuple[float, float]:
        try:
            model_state = self.get_model_state(self.robot_name, "world")
            return (model_state.pose.position.x, model_state.pose.position.y)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return (0, 0)

    def get_avg_velocity(self) -> float:
        if self.velocity_array:
            vel_array = np.mean(self.velocity_array, axis=0)
            return np.linalg.norm(vel_array)
        return 0.0

    def get_path_length(self) -> float:
        path_length = 0.0
        for i in range(1, len(self.position_array)):
            path_length += np.linalg.norm(np.array(self.position_array[i]) - np.array(self.position_array[i - 1]))
        return path_length

    def save_metrics(self, filename="metrics.txt"):
        with open(filename, "w") as f:
            f.write("Metrics for the Current Run\n")
            f.write(f"1. Final Average Speed: {self.get_avg_velocity()} m/s\n")
            f.write(f"2. Final Total Distance Travelled: {self.get_path_length()} m\n")
            f.write(f"3. Number of Goals Received: {len(self.goal_positions)}\n")
            f.write("4. Goal Positions, Robot Positions, Average Velocities, and Distances Travelled:\n")
            for i, (goal, robot_pos, avg_vel, distance) in enumerate(
                zip(self.goal_positions, self.robot_positions, self.avg_velocities, self.distances_travelled)
            ):
                f.write(f"   Goal {i+1}: x={goal[0]}, y={goal[1]}\n")
                f.write(f"   Robot Position {i-1}: x={robot_pos[0]}, y={robot_pos[1]}\n")
                f.write(f"   Average Velocity {i-1}: {avg_vel} m/s\n")
                f.write(f"   Total Distance Travelled {i-1}: {distance} m\n")
                f.write("\n")


def main():
    metrics = CalculateMetrics()
    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        print("=====================================")
        print("3. Number of Goals Received: ", len(metrics.goal_positions))
        print("4. Goal Positions, Robot Positions, Average Velocities, and Distances Travelled:")
        for i, (goal, robot_pos, avg_vel, distance) in enumerate(
            zip(metrics.goal_positions, metrics.robot_positions, metrics.avg_velocities, metrics.distances_travelled)
        ):
            print(f"   Goal {i+1}: x={goal[0]}, y={goal[1]}")
            print(f"   Robot Position: x={robot_pos[0]}, y={robot_pos[1]}")
            print(f"   Average Velocity: {avg_vel} m/s")
            print(f"   Total Distance Travelled: {distance} m")
            print()
        print("=====================================")
        metrics.save_metrics()
        print("Total Distance Travelled: ", metrics.get_path_length())
        print("Average_velocity: ", metrics.get_avg_velocity())


if __name__ == "__main__":
    main()
