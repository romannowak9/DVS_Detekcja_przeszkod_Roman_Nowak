#!/home/roman/anaconda3/envs/v2e/bin/python
import gz.transport13
import geometry_msgs.msg
import rclpy
import rclpy.publisher
import std_msgs.msg
import numpy as np

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import gz
from gz.msgs10.clock_pb2 import Clock

from typing import List

# TODO; Trochę to uprościć, bo jest głupia kalka strzałek i zrobić komendę stop


class GazeboSimNode(gz.transport13.Node):
    def __init__(self):
        super().__init__()

        #Create subscription
        self.clock_subscriber_ = self.subscribe(Clock, "/clock", self.clock_cb)

        self.sim_time_sec : int = 0

    def clock_cb(self, msg: Clock):
        time_sim = msg.sim
        self.sim_time_sec = time_sim.sec


velocity_commands = {
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

    # Publishers
    velocity_pub = control_node.create_publisher(geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos_profile)
    position_pub = control_node.create_publisher(geometry_msgs.msg.Pose, '/offboard_position_cmd', qos_profile)
    acceleration_pub = control_node.create_publisher(geometry_msgs.msg.Accel, '/offboard_acceleration_cmd', qos_profile)
    arm_pub = control_node.create_publisher(std_msgs.msg.Bool, '/arm_message', qos_profile)
    param_pub = control_node.create_publisher(std_msgs.msg.UInt8, '/controlled_param', qos_profile)

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

    curr_drone_vel = [speed, turn, x, y, z, th, status, x_val, y_val, z_val, yaw_val]

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

    curr_drone_vel = procure_velocity_command("GoUp", curr_drone_vel, velocity_pub, param_pub)

    wait(3)
    curr_drone_vel = procure_velocity_command("GoDown", curr_drone_vel, velocity_pub, param_pub)

    while True:
        procure_velocity_command("GoForward", curr_drone_vel, velocity_pub, param_pub, speed = 2)
        wait(3)
    # curr_drone_vel = procure_velocity_command("GoForward", curr_drone_vel, velocity_pub, param_pub, speed = 2)

    # wait(15)
    # curr_drone_vel = procure_velocity_command("GoBackward", curr_drone_vel, velocity_pub, param_pub, speed = 2)
    # curr_drone_vel = procure_velocity_command("GoRight", curr_drone_vel, velocity_pub, param_pub)
    # wait(1)
    
    # while True:
    #     curr_drone_vel = procure_velocity_command("GoLeft", curr_drone_vel, velocity_pub, param_pub)
    #     curr_drone_vel = procure_velocity_command("GoLeft", curr_drone_vel, velocity_pub, param_pub)
    #     wait(2)
    #     curr_drone_vel = procure_velocity_command("GoRight", curr_drone_vel, velocity_pub, param_pub)
    #     curr_drone_vel = procure_velocity_command("GoRight", curr_drone_vel, velocity_pub, param_pub)
    #     wait(2)


    # wait(15)
    # move_to_postion(10.0, 0.0, 2.5, yaw_to_quaternion(np.pi / 2.0), position_pub, param_pub)  # Może tu trzeba dawać aktualną rotację drona? Nawet na pewno



def procure_velocity_command(command: str, initial: List[float], pub: rclpy.publisher, param_pub: rclpy.publisher, speed: float = None) -> List[float]:
    # Setting controlled parameter to velocity
    param_msg = std_msgs.msg.UInt8()
    param_msg.data = 1
    param_pub.publish(param_msg)

    if speed is None:
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

    if command in velocity_commands.keys():
        x = velocity_commands[command][0]
        y = velocity_commands[command][1]
        z = velocity_commands[command][2]
        th = velocity_commands[command][3]
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
    print("Velocities:  X:",twist.linear.x, "   Y:",twist.linear.y, "   Z:",twist.linear.z, "   Yaw:",twist.angular.z)

    return [speed, turn, x, y, z, th, status, x_val, y_val, z_val, yaw_val]


# x, y, z : FLU coordinates; rotation in quaternion form
def move_to_postion(x: float, y: float, z: float, rotation: geometry_msgs.msg.Quaternion, pub: rclpy.publisher, param_pub: rclpy.publisher):
    # Setting controlled parameter to position
    param_msg = std_msgs.msg.UInt8()
    param_msg.data = 2
    param_pub.publish(param_msg)

    pose = geometry_msgs.msg.Pose()

    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    pose.orientation = rotation

    pub.publish(pose)

    print("Position:  X:",pose.position.x, "   Y:",pose.position.y, "   Z:",pose.position.z, "   Yaw:",quaternion_to_euler(rotation)[2])


def quaternion_to_euler(quaternion: geometry_msgs.msg.Quaternion) -> List[float]:
    """
    Converts a quaternion to Euler angles (roll, pitch, yaw).

    Args:
        quaternion (Quaternion): The quaternion to convert.

    Returns:
        List[float]: A list containing the roll, pitch, and yaw angles in radians.
    """
    # Extract the values from the quaternion
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w

    # Calculate roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Calculate pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # Calculate yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return [roll, pitch, yaw]


def yaw_to_quaternion(yaw: float) -> geometry_msgs.msg.Quaternion:
    """
    Converts Euler angles (roll, pitch, yaw) to a quaternion.

    Args:
        roll (float): The roll angle in radians.
        pitch (float): The pitch angle in radians.
        yaw (float): The yaw angle in radians.

    Returns:
        Quaternion: The corresponding quaternion.
    """
    # Compute half angles
    roll = 0.0
    pitch = 0.0

    half_roll = roll / 2.0
    half_pitch = pitch / 2.0
    half_yaw = yaw / 2.0

    # Calculate sin and cos for each half angle
    sin_r = np.sin(half_roll)
    cos_r = np.cos(half_roll)
    sin_p = np.sin(half_pitch)
    cos_p = np.cos(half_pitch)
    sin_y = np.sin(half_yaw)
    cos_y = np.cos(half_yaw)

    # Calculate quaternion components
    qx = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y
    qy = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y
    qz = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y
    qw = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y

    return geometry_msgs.msg.Quaternion(x=qx, y=qy, z=qz, w=qw)



if __name__ == '__main__':
    main()
    