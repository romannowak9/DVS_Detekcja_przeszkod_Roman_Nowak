#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# EDITED AND EXTENDED BY: ROMAN NOWAK
#
############################################################################

__author__ = "Braden Wagstaff"
__contact__ = "braden@arkelectron.com"

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from geometry_msgs.msg import Twist, Vector3, Pose, Accel
from math import pi
from std_msgs.msg import Bool, UInt8
from enum import Enum


class ControlledParam(Enum):
    VELOCITY = 1
    POSITION = 2
    ACCELERATION = 3


class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        #Create subscriptions
        self.offboard_acceleration_sub = self.create_subscription(
            Accel,
            '/offboard_acceleration_cmd',
            self.offboard_acceleration_callback,
            qos_profile)
        
        self.offboard_position_sub = self.create_subscription(
            Pose,
            '/offboard_position_cmd',
            self.offboard_position_callback,
            qos_profile)
        
        self.control_param_sub = self.create_subscription(
            UInt8,
            '/controlled_param',
            self.controlled_param_callback,
            qos_profile)
        
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        self.offboard_velocity_sub = self.create_subscription(
            Twist,
            '/offboard_velocity_cmd',
            self.offboard_velocity_callback,
            qos_profile)
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile)
        
        self.my_bool_sub = self.create_subscription(
            Bool,
            '/arm_message',
            self.arm_message_callback,
            qos_profile)


        #Create publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_velocity = self.create_publisher(Twist, '/fmu/in/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        
        #creates callback function for the arm timer
        # period is arbitrary, just should be more than 2Hz
        arm_timer_period = .1 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # creates callback function for the command loop
        # period is arbitrary, just should be more than 2Hz. Because live controls rely on this, a higher frequency is recommended
        # commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.velocity = Vector3()
        self.position = Vector3()
        self.acceleration = Vector3()
        self.yaw_pos = 0.0
        self.yaw_speed = 0.0  #yaw value we send as command
        self.yaw_accel = 0.0
        self.trueYaw = 0.0  #current yaw value of drone
        self.offboardMode = False
        self.flightCheck = False
        self.myCnt = 0
        self.arm_message = False
        self.failsafe = False
        self.current_state = "IDLE"
        self.last_state = self.current_state
        self.controlled_param = ControlledParam.VELOCITY

    def controlled_param_callback(self, msg: UInt8):
        self.controlled_param = ControlledParam(msg.data)
        self.get_logger().info(f"Controlled parameter: {ControlledParam(self.controlled_param)}")

    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        self.get_logger().info(f"Arm Message: {self.arm_message}")

    #callback function that arms, takes off, and switches to offboard mode
    #implements a finite state machine
    def arm_timer_callback(self):

        match self.current_state:
            case "IDLE":
                if(self.flightCheck and self.arm_message == True):
                    self.current_state = "ARMING"
                    self.get_logger().info(f"Arming")

            case "ARMING":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Arming, Flight Check Failed")
                elif(self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 10):
                    self.current_state = "TAKEOFF"
                    self.get_logger().info(f"Arming, Takeoff")
                self.arm() #send arm command

            case "TAKEOFF":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Takeoff, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.current_state = "LOITER"
                    self.get_logger().info(f"Takeoff, Loiter")
                self.arm() #send arm command
                self.take_off() #send takeoff command

            # waits in this state while taking off, and the 
            # moment VehicleStatus switches to Loiter state it will switch to offboard
            case "LOITER": 
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Loiter, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    self.current_state = "OFFBOARD"
                    self.get_logger().info(f"Loiter, Offboard")
                self.arm()

            case "OFFBOARD":
                if(not(self.flightCheck) or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe == True):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Offboard, Flight Check Failed")
                self.state_offboard()

        if(self.arm_state != VehicleStatus.ARMING_STATE_ARMED):
            self.arm_message = False

        if (self.last_state != self.current_state):
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)

        self.myCnt += 1

    def state_offboard(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboardMode = True   

    # Arms the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    # Takes off the vehicle to a user specified altitude (meters)
    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0) # param7 is altitude in meters
        self.get_logger().info("Takeoff command send")

    #publishes command to /fmu/in/vehicle_command
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7    # altitude value in takeoff command
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    #receives and sets vehicle status values 
    def vehicle_status_callback(self, msg):

        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        
        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        if (msg.pre_flight_checks_pass != self.flightCheck):
            self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass

    #receives Accel commands from Teleop and converts FLU -> NED
    def offboard_acceleration_callback(self, msg: Accel):
        #implements FLU -> NED Transformation
        # X (NED) is -Y (FLU)
        self.acceleration.x = -msg.linear.y
        # Y (NED) is X (FLU)
        self.acceleration.y = msg.linear.x
        # Z (NED) is -Z (FLU)
        self.acceleration.z = -msg.linear.z
        # A conversion for angular z is done in the attitude_callback function(it's the '-' in front of self.trueYaw)
        self.yaw_accel = msg.angular.z

    #receives Pose commands from Teleop and converts FLU -> NED
    def offboard_position_callback(self, msg: Pose):
        #implements FLU -> NED Transformation
        # X (NED) is -Y (FLU)
        self.position.x = -msg.position.y
        # Y (NED) is X (FLU)
        self.position.y = msg.position.x
        # Z (NED) is -Z (FLU)
        self.position.z = -msg.position.z
        # From quternion to euler angle yaw
        # A conversion for angular z is done in the attitude_callback function(it's the '-' in front of self.trueYaw)
        self.yaw_pos = np.arctan2(2.0*(msg.orientation.z*msg.orientation.w + msg.orientation.x*msg.orientation.y), 
                       1.0 - 2.0*(msg.orientation.w*msg.orientation.w + msg.orientation.x*msg.orientation.x))

    #receives Twist commands from Teleop and converts FLU -> NED
    def offboard_velocity_callback(self, msg: Twist):
        #implements FLU -> NED Transformation
        # X (NED) is -Y (FLU)
        self.velocity.x = -msg.linear.y
        # Y (NED) is X (FLU)
        self.velocity.y = msg.linear.x
        # Z (NED) is -Z (FLU)
        self.velocity.z = -msg.linear.z
        # A conversion for angular z is done in the attitude_callback function(it's the '-' in front of self.trueYaw)
        self.yaw_speed = msg.angular.z

    # receives current trajectory values from drone and grabs the yaw value of the orientation
    # reads the drone's orientation in quaternion form and converts it to a yaw angle in radians, 
    # which represents the drone's current heading in the horizontal plane
    def attitude_callback(self, msg: VehicleAttitude):
        orientation_q = msg.q

        #trueYaw is the drones current yaw value
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
        
    #publishes offboard control modes and velocity as trajectory setpoints
    def cmdloop_callback(self):
        if(self.offboardMode == True):
            # self.get_logger().info(f"{ControlledParam(self.controlled_param).name}, {self.controlled_param}")
            # Publish offboard control modes
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = True if self.controlled_param == ControlledParam.POSITION else False
            offboard_msg.velocity = True if self.controlled_param == ControlledParam.VELOCITY else False
            offboard_msg.acceleration = True if self.controlled_param == ControlledParam.ACCELERATION else False
            self.publisher_offboard_mode.publish(offboard_msg)            

            # Compute velocity in the world frame
            cos_yaw = np.cos(self.trueYaw)
            sin_yaw = np.sin(self.trueYaw)
            # Rzutowanie na z osi świata na osie drona
            velocity_world_x = (self.velocity.x * cos_yaw - self.velocity.y * sin_yaw)
            velocity_world_y = (self.velocity.x * sin_yaw + self.velocity.y * cos_yaw)

            position_world_x = (self.position.x * cos_yaw - self.position.y * sin_yaw)
            position_world_y = (self.position.x * sin_yaw + self.position.y * cos_yaw)

            acceleration_world_x = (self.acceleration.x * cos_yaw - self.acceleration.y * sin_yaw)
            acceleration_world_y = (self.acceleration.x * sin_yaw + self.acceleration.y * cos_yaw)

            # Create and publish TrajectorySetpoint message with NaN values for non-controlled parameters
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)

            
            trajectory_msg.velocity[0] = velocity_world_x if self.controlled_param == ControlledParam.VELOCITY else float('nan')
            trajectory_msg.velocity[1] = velocity_world_y if self.controlled_param == ControlledParam.VELOCITY else float('nan')
            trajectory_msg.velocity[2] = self.velocity.z if self.controlled_param == ControlledParam.VELOCITY else float('nan')
            trajectory_msg.yawspeed = self.yaw_speed if self.controlled_param == ControlledParam.VELOCITY else float('nan')
            
            trajectory_msg.position[0] = position_world_x if self.controlled_param == ControlledParam.POSITION else float('nan')
            trajectory_msg.position[1] = position_world_y if self.controlled_param == ControlledParam.POSITION else float('nan')
            trajectory_msg.position[2] = self.position.z if self.controlled_param == ControlledParam.POSITION else float('nan')
            trajectory_msg.yaw = self.yaw_pos if self.controlled_param == ControlledParam.POSITION else float('nan')

            trajectory_msg.acceleration[0] = acceleration_world_x if self.controlled_param == ControlledParam.ACCELERATION else float('nan')
            trajectory_msg.acceleration[1] = acceleration_world_y if self.controlled_param == ControlledParam.ACCELERATION else float('nan')
            trajectory_msg.acceleration[2] = self.acceleration.z if self.controlled_param == ControlledParam.ACCELERATION else float('nan')

            self.publisher_trajectory.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
