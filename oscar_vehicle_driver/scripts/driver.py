#!/usr/bin/env python

###############################################################################
# Copyright 2020 ScPA StarLine Ltd. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

import rospy

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

from oscar_vehicle_api import create_vehicle_by_model

from oscar_vehicle_driver.msg import *
from oscar_vehicle_driver.srv import *

VEHICLE_STATUS_PUB_RATE = 10
VEHICLE_ODOM_PUB_RATE = 25


class OscarVehicleRosDriver:

    def __init__(self):

        rospy.init_node('oscar_vehicle_driver')

        self.oscar_port    = rospy.get_param('oscar/port', '/dev/ttyACM0')
        self.vehicle_model = rospy.get_param('oscar/vehicle_model', 'LEXUS_RX_450H')

        self.vehicle = create_vehicle_by_model(vehicle_model = self.vehicle_model,
                                               interface     = self.oscar_port)

        self.raw_control_mode = True
        rospy.Subscriber("oscar/vehicle_cmd",     VehicleCmd,    self.cmd_cb)
        rospy.Subscriber("oscar/vehicle_cmd_raw", VehicleCmdRaw, self.cmd_raw_cb)

        self.auto_mode_srv      = rospy.Service('oscar/auto_mode',      Trigger, self.auto_mode_cb)
        self.manual_mode_srv    = rospy.Service('oscar/manual_mode',    Trigger, self.manual_mode_cb)
        self.emergency_stop_srv = rospy.Service('oscar/emergency_stop', Trigger, self.emergency_stop_cb)
        self.recover_srv        = rospy.Service('oscar/recover',        Trigger, self.recover_cb)

        # self.test_brake = False
        # self.test_brake_on_srv     = rospy.Service('oscar/test_brake_on',     Trigger, self.test_brake_on_cb)
        # self.test_brake_off_srv    = rospy.Service('oscar/test_brake_off',    Trigger, self.test_brake_off_cb)

        self.forward_move_srv  = rospy.Service('oscar/forward_move',   Trigger, self.forward_move_cb)
        self.cruise_move_srv   = rospy.Service('oscar/cruise_move',   Trigger, self.cruise_move_cb)
        self.backward_move_srv = rospy.Service('oscar/backward_move',  Trigger, self.backward_move_cb)

        self.led_on_srv    = rospy.Service('oscar/led_on',    Trigger, self.led_on_cb)
        self.led_off_srv   = rospy.Service('oscar/led_off',   Trigger, self.led_off_cb)
        self.led_blink_srv = rospy.Service('oscar/led_blink', Trigger, self.led_blink_cb)

        self.left_turn_signal_srv  = rospy.Service('oscar/left_turn_signal',  Trigger, self.left_turn_signal_cb)
        self.right_turn_signal_srv = rospy.Service('oscar/right_turn_signal', Trigger, self.right_turn_signal_cb)
        self.emergency_signals_srv = rospy.Service('oscar/emergency_signals', Trigger, self.emergency_signals_cb)
        self.turn_off_signals_srv  = rospy.Service('oscar/turn_off_signals',  Trigger, self.turn_off_signals_cb)

        self.intercept_steering_wheel_srv = rospy.Service('oscar/intercept_steering_wheel', Trigger, self.intercept_steering_wheel_cb)
        self.rest_steering_wheel_srv      = rospy.Service('oscar/rest_steering_wheel',      Trigger, self.rest_steering_wheel_cb)

        self.start_sending_steering_wheel_srv = rospy.Service('oscar/start_sending_steering_wheel_cmds', Trigger, self.start_sending_steering_wheel_cmd_cb)
        self.stop_sending_steering_wheel_srv  = rospy.Service('oscar/stop_sending_steering_wheel_cmds',  Trigger, self.stop_sending_steering_wheel_cmd_cb)

        self.intercept_vehicle_acceleration_srv = rospy.Service('oscar/intercept_vehicle_move', Trigger, self.intercept_vehicle_move_cb)
        self.rest_vehicle_acceleration_srv      = rospy.Service('oscar/rest_vehicle_move',      Trigger, self.rest_vehicle_move_cb)

        self.start_sending_vehicle_acceleration_srv = rospy.Service('oscar/start_sending_vehicle_move_cmds', Trigger, self.start_sending_vehicle_move_cmd_cb)
        self.stop_sending_vehicle_acceleration_srv  = rospy.Service('oscar/stop_sending_vehicle_move_cmds',  Trigger, self.stop_sending_vehicle_move_cmd_cb)

        self.logger_start_srv = rospy.Service('oscar/logger_start', Trigger, self.logger_start_cb)
        self.logger_stop_srv  = rospy.Service('oscar/logger_stop',  Trigger, self.logger_stop_cb)

        self.vehicle_status_pub   = rospy.Publisher("oscar/vehicle_status", VehicleStatus, queue_size = 1)
        self.vehicle_status_timer = rospy.Timer(rospy.Duration(1./VEHICLE_STATUS_PUB_RATE), self.vehicle_status_timer_cb)

        self.vehicle_odom_pub   = rospy.Publisher("oscar/odom", Odometry, queue_size = 1)
        self.vehicle_odom_timer = rospy.Timer(rospy.Duration(1./VEHICLE_ODOM_PUB_RATE), self.vehicle_odom_timer_cb)
        self.vehicle_odom_broadcaster = tf.TransformBroadcaster()


    def vehicle_status_timer_cb(self, timer):

        vehicle_status_msg = VehicleStatus()
        vehicle_status_msg.mode = self.vehicle.get_mode()
        # print("veh_speed:" + str(self.vehicle.get_vehicle_speed()))
        # print("ws_pos_ve:" + str(self.vehicle.get_steering_wheel_angle_and_velocity()))
        # print("ws_torque:" + str(self.vehicle.get_steering_wheel_and_eps_torques()))
        self.vehicle_status_pub.publish(vehicle_status_msg)


    def vehicle_odom_timer_cb(self, timer):

        x, y, yaw, dx, dy, dyaw, otime = self.vehicle.get_odometry()
        current_time = rospy.Time.now()

        Q = tf.transformations.quaternion_from_euler(0, 0, yaw)

        self.vehicle_odom_broadcaster.sendTransform((x, y, 0.), Q,
                                                    current_time,
                                                    "base_link",
                                                    "odom")

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"

        odom_msg.pose.pose = Pose(Point(x, y, 0), Quaternion(*Q))

        odom_msg.child_frame_id = "base_link"
        odom_msg.twist.twist = Twist(Vector3(dx, dy, 0), Vector3(0, 0, dyaw))

        self.vehicle_odom_pub.publish(odom_msg)


    def cmd_cb(self, msg):

        if self.raw_control_mode:
            self.vehicle.start_controller()
            self.raw_control_mode = False

        self.vehicle.set_speed(msg.speed, msg.acceleration, msg.jerk)
        self.vehicle.set_steering(msg.steering_angle, msg.steering_angle_velocity)


    def cmd_raw_cb(self, msg):

        if not self.raw_control_mode:
            self.vehicle.stop_controller()
            self.raw_control_mode = True

        # if self.test_brake:
        #     if (msg.throttle <= 0):
        #         self.vehicle.set_vehicle_brake(msg.throttle)
        #         self.vehicle.set_vehicle_test_throttle(0)
        #     else:
        #         self.vehicle.set_vehicle_test_throttle(msg.throttle)
        # else:
        self.vehicle.set_vehicle_throttle(msg.throttle)
        self.vehicle.set_steering_wheel_torque(msg.steering_wheel_torque)


    def forward_move_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.set_vehicle_forward_move():
            print("FORWARD")
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def cruise_move_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.set_vehicle_cruise_move():
            print("CRUISE")
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def backward_move_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.set_vehicle_backward_move():
            print("REVERSE")
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    # def test_brake_on_cb(self, request):
    #
    #     response = TriggerResponse()
    #     if self.vehicle.test_brake_on() and self.vehicle.test_throttle_on():
    #         self.test_brake = True
    #         print("TEST BRAKE ON")
    #         response.result = TriggerResponse.DONE
    #     else:
    #         response.result = TriggerResponse.ERROR
    #         response.why = self.vehicle.error_report()
    #     return response
    #
    #
    # def test_brake_off_cb(self, request):
    #
    #     response = TriggerResponse()
    #     if self.vehicle.test_brake_off() and self.vehicle.test_throttle_off():
    #         self.test_brake = False
    #         print("TEST BRAKE OFF")
    #         response.result = TriggerResponse.DONE
    #     else:
    #         response.result = TriggerResponse.ERROR
    #         response.why = self.vehicle.error_report()
    #     return response


    def auto_mode_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.auto_mode():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def manual_mode_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.manual_mode():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def emergency_stop_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.emergency_stop():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def recover_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.recover():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def led_blink_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.led_blink():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def led_on_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.led_on():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def led_off_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.led_off():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def left_turn_signal_cb(self):
        response = TriggerResponse()
        if self.vehicle.left_turn_signal():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def right_turn_signal_cb(self):
        response = TriggerResponse()
        if self.vehicle.right_turn_signal():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def emergency_signals_cb(self):
        response = TriggerResponse()
        if self.vehicle.emergency_signals():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def turn_off_signals_cb(self):
        response = TriggerResponse()
        if self.vehicle.turn_off_signals():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def intercept_steering_wheel_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.steering_wheel_interception_on():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def rest_steering_wheel_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.steering_wheel_interception_off():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def start_sending_steering_wheel_cmd_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.start_sending_steering_wheel_torque_cmd():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def stop_sending_steering_wheel_cmd_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.stop_sending_steering_wheel_torque_cmd():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def intercept_vehicle_move_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.vehicle_move_interception_on():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def rest_vehicle_move_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.vehicle_move_interception_off():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def start_sending_vehicle_move_cmd_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.start_sending_vehicle_move_cmd():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def stop_sending_vehicle_move_cmd_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.stop_sending_vehicle_move_cmd():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def logger_start_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.start_vehicle_logger():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def logger_stop_cb(self, request):

        response = TriggerResponse()
        if self.vehicle.stop_vehicle_logger():
            response.result = TriggerResponse.DONE
        else:
            response.result = TriggerResponse.ERROR
            response.why = self.vehicle.error_report()
        return response


    def spin(self):
        rospy.spin()


if __name__ == '__main__':

    driver = OscarVehicleRosDriver()
    driver.spin()
