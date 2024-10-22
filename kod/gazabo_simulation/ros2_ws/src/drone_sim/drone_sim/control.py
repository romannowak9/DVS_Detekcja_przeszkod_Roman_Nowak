import gz.transport13
import geometry_msgs.msg
import rclpy
import rclpy.publisher
import std_msgs.msg

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import gz
from gz.msgs10.clock_pb2 import Clock

from typing import List


class GazeboSimNode(gz.transport13.Node):
    def __init__(self):
        super().__init__()

        #Create subscription
        self.pose_subscriber_ = self.subscribe(Clock, "/clock", self.clock_cb)

        self.sim_time_sec : int = 0

    def clock_cb(self, msg: Clock):
        time_sim = msg.sim
        self.sim_time_sec = time_sim.sec


move_commands = {
    'GoUp': (0, 0, 1, 0), #Z+   # x, y, z, th
    'GoDown': (0, 0, -1, 0),#Z-
    'RotateRight': (0, 0, 0, -1), #Yaw+
    'RotateLeft': (0, 0, 0, 1),#Yaw-
    'GoForward' : (0, 1, 0, 0),  #Up Arrow
    'GoBackward' : (0, -1, 0, 0), #Down Arrow
    'GoRight' : (-1, 0, 0, 0), #Right Arrow
    'GoLeft' : (1, 0, 0, 0),  #Left Arrow
}


def main():
    # print("Starting...")
    rclpy.init()
    control_node = rclpy.create_node("control_node")
    sim_node = GazeboSimNode()

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )

    geo_pub = control_node.create_publisher(geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos_profile)
    arm_pub = control_node.create_publisher(std_msgs.msg.Bool, '/arm_message', qos_profile)

    speed = 0.5
    turn = .2
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0
    yaw_val = 0.0

    curr_drone_pos = [speed, turn, x, y, z, th, status, x_val, y_val, z_val, yaw_val]

    def wait(sec: int):
        start_time = sim_node.sim_time_sec
        while sim_node.sim_time_sec <  start_time + sec:
            continue

    wait(10)

    # Arming drone automatically
    arm_msg = std_msgs.msg.Bool()
    arm_msg.data = True
    arm_pub.publish(arm_msg)
    print(f"Arm toggle is now: {True}")

    curr_drone_pos = procure_move_command("GoUp", curr_drone_pos, geo_pub)

    wait(3)

    curr_drone_pos = procure_move_command("GoDown", curr_drone_pos, geo_pub)
    curr_drone_pos = procure_move_command("GoForward", curr_drone_pos, geo_pub)


def procure_move_command(command: str, initial: List[float], pub: rclpy.publisher) -> List[float]:
    speed = initial[0]
    turn = initial[1]
    x = initial[2]
    y = initial[3]
    z = initial[4]
    th = initial[5]
    status = initial[6]
    x_val = initial[7]
    y_val = initial[8]
    z_val = initial[9]
    yaw_val = initial[10]

    if command in move_commands.keys():
        x = move_commands[command][0]
        y = move_commands[command][1]
        z = move_commands[command][2]
        th = move_commands[command][3]
    else:
        x = 0.0
        y = 0.0
        z = 0.0
        th = 0.0
        raise ValueError("No such command!")

    twist = geometry_msgs.msg.Twist()
    
    x_val = (x * speed) + x_val
    y_val = (y * speed) + y_val
    z_val = (z * speed) + z_val
    yaw_val = (th * turn) + yaw_val
    twist.linear.x = x_val
    twist.linear.y = y_val
    twist.linear.z = z_val
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = yaw_val
    pub.publish(twist)
    print("X:",twist.linear.x, "   Y:",twist.linear.y, "   Z:",twist.linear.z, "   Yaw:",twist.angular.z)

    return [speed, turn, x, y, z, th, status, x_val, y_val, z_val, yaw_val]


if __name__ == '__main__':
    main()
    