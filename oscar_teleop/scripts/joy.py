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

import curses
import rospy

import math as m
from math import sqrt, atan2, copysign

from oscar_vehicle_driver.msg import *
from oscar_vehicle_driver.srv import *

from sensor_msgs.msg import Joy

MAX_STEERING_WHEEL_TORQUE = 90
WAIT_FOR_SERVICE_SERVER_TIMEOUT = 1

class OscarJoyTeleop:

    def __init__(self):

        rospy.init_node('joy_to_oscar_driver')
        rospy.Subscriber("joy", Joy, self.joy_cb)
        self.vehicle_cmd_raw_pub = rospy.Publisher("oscar/vehicle_cmd_raw", VehicleCmdRaw, queue_size = 1)
        self.vehicle_cmd_pub = rospy.Publisher("oscar/vehicle_cmd", VehicleCmd, queue_size = 1)

        self.last_button_A = 0
        self.last_button_B = 0
        self.last_button_logitech = 0

        self.last_button_X = 0
        self.last_button_Y = 0

        self.last_button_start = 0
        self.last_button_back  = 0
        self.direction = "CRUISE"
        self.last_direction = ""

        self.last_stick_left_back  = 0
        self.last_stick_right_back = 0
        self.teleop_allowed = False

        self.steering_wheel_torque = 0
        self.throttle = 0

        self.steering_wheel_angle_control_active = False
        self.last_steering_wheel_angle_control_was_active = False
        self.steering_wheel_angle = 0
        self.real_steering_wheel_angle = 0
        self.max_steering_wheel_angle = 500

        self.last_stick_right_angle  = 0.0
        self.delta_stick_right_angle = 0.0
        self.max_delta_stick_right_angle = 100.0

        self.mode = "MANUAL"
        self.emergency_stop_is_on = False

        self.led = False

        self.last_joy_X_button_state = 0



    def joy_cb(self, msg):

        cur_button_A = msg.buttons[0]
        cur_button_B = msg.buttons[1]
        cur_button_X = msg.buttons[2]
        cur_button_Y = msg.buttons[3]

        cur_button_left_back  = msg.buttons[4]
        cur_button_right_back = msg.buttons[5]

        cur_button_back  = msg.buttons[6]
        cur_button_start = msg.buttons[7]
        cur_button_logitech = msg.buttons[8]

        cur_button_left_stick  = msg.buttons[9]
        cur_button_right_stick = msg.buttons[10]

        cur_stick_left_horizontal  = msg.axes[0] * MAX_STEERING_WHEEL_TORQUE # [-100:100]
        cur_stick_left_vertical    = msg.axes[1]
        cur_stick_left_back        = 50*(msg.axes[2] - 1.0)      # [-100:0]

        cur_stick_right_horizontal = msg.axes[3]
        cur_stick_right_vertical   = msg.axes[4]
        cur_stick_right_back       = -50*(msg.axes[5] - 1.0)    # [0:100]

        cur_arrows_horizontal = msg.axes[6]
        cur_arrows_vertical   = msg.axes[7]

        if self.teleop_allowed:

            # mode
            if ((self.last_button_A == 0) and (cur_button_A == 1)):
                if self.mode == "MANUAL":
                    self.auto_mode()
                else:
                    self.manual_mode()

            self.last_button_A = cur_button_A

            # emergency / recover
            if ((self.last_button_B == 0) and (cur_button_B == 1)):
                if self.emergency_stop_is_on:
                    self.recover()
                else:
                    self.emergency_stop()

            self.last_button_B = cur_button_B

            # led
            if ((self.last_button_logitech == 0) and (cur_button_logitech == 1)):
                if self.led:
                    self.led_off()
                else:
                    self.led_on()

            self.last_button_logitech = cur_button_logitech

            # TEST BRAKE
            # if ((self.last_button_Y == 0) and (cur_button_Y == 1)):
            #     self.test_brake_on()
            # self.last_button_Y = cur_button_Y
            #
            # if ((self.last_button_X == 0) and (cur_button_X == 1)):
            #     self.test_brake_off()
            # self.last_button_X = cur_button_X


            # FORWARD / BACKWARD MOVEMENT
            if ((self.last_button_start == 0) and (cur_button_start == 1)):

                if self.direction == "REVERSE":

                    if self.last_direction == "CRUISE":
                        self.cruise_move()
                        self.direction = "CRUISE"

                    else:
                        self.forward_move()
                        self.direction = "FORWARD"

                    self.last_direction = "REVERSE"

                elif self.direction == "CRUISE":
                    self.last_direction = self.direction
                    self.forward_move()
                    self.direction = "FORWARD"

                else:
                    self.last_direction = self.direction
                    self.cruise_move()
                    self.direction = "CRUISE"

            self.last_button_start = cur_button_start

            if ((self.last_button_back == 0) and (cur_button_back == 1)):
                self.backward_move()
                self.last_direction = self.direction
                self.direction = "REVERSE"
            self.last_button_back = cur_button_back


            # throttle
            if (cur_stick_left_back < -1.0):
                self.throttle = cur_stick_left_back
            else:
                self.throttle = cur_stick_right_back

            # steering
            self.steering_wheel_torque = cur_stick_left_horizontal

            stick_right_module = sqrt(cur_stick_right_horizontal ** 2 + cur_stick_right_vertical ** 2)
            self.steering_wheel_angle_control_active = (stick_right_module > 0.99)

            # steering wheel pos control
            if self.steering_wheel_angle_control_active:

                stick_right_angle = atan2(cur_stick_right_horizontal, cur_stick_right_vertical) / 3.14 * 180

                if self.last_steering_wheel_angle_control_was_active:
                    self.delta_stick_right_angle = stick_right_angle - self.last_stick_right_angle
                    if abs(self.delta_stick_right_angle) < self.max_delta_stick_right_angle:
                        self.steering_wheel_angle += self.delta_stick_right_angle
                        if not (-self.max_steering_wheel_angle < self.steering_wheel_angle < self.max_steering_wheel_angle):
                            self.steering_wheel_angle = copysign(self.max_steering_wheel_angle, self.steering_wheel_angle)

                self.last_stick_right_angle = stick_right_angle
                self.last_steering_wheel_angle_control_was_active = True

            else:
                self.last_steering_wheel_angle_control_was_active = False

            if self.steering_wheel_angle_control_active:
                vehicle_cmd_msg = VehicleCmd()
                vehicle_cmd_msg.steering_angle = self.steering_wheel_angle
                vehicle_cmd_msg.acceleration = self.throttle
                self.vehicle_cmd_pub.publish(vehicle_cmd_msg)
            else:
                vehicle_cmd_raw_msg = VehicleCmdRaw()
                self.steering_wheel_torque = self.steering_wheel_torque / 2
                if self.direction != "CRUISE":
                    self.throttle = self.throttle / 3
                vehicle_cmd_raw_msg.throttle = self.throttle
                vehicle_cmd_raw_msg.steering_wheel_torque = self.steering_wheel_torque
                self.vehicle_cmd_raw_pub.publish(vehicle_cmd_raw_msg)

        # turn on/off joy
        if (((self.last_stick_right_back >= 99.5) and (self.last_stick_left_back <= -99.5)) and
            ((cur_stick_right_back < 99.5) or (cur_stick_left_back > -99.5))):

            self.steering_wheel_torque = 0
            self.throttle = 0
            self.teleop_allowed = not self.teleop_allowed
            rospy.sleep(0.6)

        self.last_stick_right_back = cur_stick_right_back
        self.last_stick_left_back  = cur_stick_left_back


    # def test_brake_on(self):
    #
    #     try:
    #         rospy.wait_for_service('oscar/test_brake_on', WAIT_FOR_SERVICE_SERVER_TIMEOUT)
    #         call_service = rospy.ServiceProxy('oscar/test_brake_on', Trigger)
    #         response = call_service()
    #     except Exception as e:
    #         print(e)
    #
    #
    # def test_brake_off(self):
    #
    #     try:
    #         rospy.wait_for_service('oscar/test_brake_off', WAIT_FOR_SERVICE_SERVER_TIMEOUT)
    #         call_service = rospy.ServiceProxy('oscar/test_brake_off', Trigger)
    #         response = call_service()
    #     except Exception as e:
    #         print(e)


    def forward_move(self):

        try:
            rospy.wait_for_service('oscar/forward_move', WAIT_FOR_SERVICE_SERVER_TIMEOUT)
            call_service = rospy.ServiceProxy('oscar/forward_move', Trigger)
            response = call_service()
        except Exception as e:
            print(e)


    def backward_move(self):

        try:
            rospy.wait_for_service('oscar/backward_move', WAIT_FOR_SERVICE_SERVER_TIMEOUT)
            call_service = rospy.ServiceProxy('oscar/backward_move', Trigger)
            response = call_service()
        except Exception as e:
            print(e)


    def cruise_move(self):

        try:
            rospy.wait_for_service('oscar/cruise_move', WAIT_FOR_SERVICE_SERVER_TIMEOUT)
            call_service = rospy.ServiceProxy('oscar/cruise_move', Trigger)
            response = call_service()
        except Exception as e:
            print(e)


    def auto_mode(self):

        try:
            rospy.wait_for_service('oscar/auto_mode', WAIT_FOR_SERVICE_SERVER_TIMEOUT)
            call_service = rospy.ServiceProxy('oscar/auto_mode', Trigger)
            response = call_service()
            if response.result == TriggerResponse.ERROR:
                print("Error while try to turning on auto mode: " + response.why)
            else:
                self.mode = "AUTO"
        except Exception as e:
            print("Error while try to stop agv:")
            print(e)


    def manual_mode(self):

        try:
            rospy.wait_for_service('oscar/manual_mode', WAIT_FOR_SERVICE_SERVER_TIMEOUT)
            call_service = rospy.ServiceProxy('oscar/manual_mode', Trigger)
            response = call_service()
            if response.result == TriggerResponse.ERROR:
                print("Error while try to turning on auto mode: " + response.why)
            else:
                self.mode = "MANUAL"
        except Exception as e:
            print("Error while try to stop agv:")
            print(e)


    def emergency_stop(self):

        try:
            rospy.wait_for_service('oscar/emergency_stop', WAIT_FOR_SERVICE_SERVER_TIMEOUT)
            call_service = rospy.ServiceProxy('oscar/emergency_stop', Trigger)
            response = call_service()
            if response.result == TriggerResponse.ERROR:
                print("Error while try to turning on auto mode: " + response.why)
            else:
                self.emergency_stop_is_on = True
        except Exception as e:
            print("Error while try to stop agv:")
            print(e)


    def recover(self):

        try:
            rospy.wait_for_service('oscar/recover', WAIT_FOR_SERVICE_SERVER_TIMEOUT)
            call_service = rospy.ServiceProxy('oscar/recover', Trigger)
            response = call_service()
            if response.result == TriggerResponse.ERROR:
                print("Error while try to turning on auto mode: " + response.why)
            else:
                self.emergency_stop_is_on = False
        except Exception as e:
            print("Error while try to stop agv:")
            print(e)


    def led_on(self):

        try:
            rospy.wait_for_service('oscar/led_on', WAIT_FOR_SERVICE_SERVER_TIMEOUT)
            call_service = rospy.ServiceProxy('oscar/led_on', Trigger)
            response = call_service()
            if response.result == TriggerResponse.ERROR:
                print("Error while try to turning on auto mode: " + response.why)
            else:
                self.led = True
        except Exception as e:
            print("Error while try to stop agv:")
            print(e)


    def led_off(self):

        try:
            rospy.wait_for_service('oscar/led_off', WAIT_FOR_SERVICE_SERVER_TIMEOUT)
            call_service = rospy.ServiceProxy('oscar/led_off', Trigger)
            response = call_service()
            if response.result == TriggerResponse.ERROR:
                print("Error while try to turning on auto mode: " + response.why)
            else:
                self.led = False
        except Exception as e:
            print("Error while try to stop agv:")
            print(e)


    def spin(self):

        screen = curses.initscr()

        rate = rospy.Rate(25)
        while not rospy.is_shutdown():

            screen.clear()

            screen.addstr(0, 0, "--- HOW TO ---------------------------------------------------------")
            screen.addstr(2, 2, "Push fully LT and RT triggers to on/off joy")
            screen.addstr(3, 2, "RT trigger for throttle and LT trigger for brake")
            screen.addstr(4, 2, "Left stick for steering wheel torque")
            screen.addstr(5, 2, "B button for turning on/off emergency brake")
            screen.addstr(6, 2, "A button for switch auto/manual mode")

            screen.addstr(8, 0, "--- STATE ----------------------------------------------------------")

            screen.addstr(10, 2, "TELEOP")
            if self.teleop_allowed:
                screen.addstr(11, 2, "ACTIVE", curses.A_BOLD)
            else:
                screen.addstr(11, 2, "INACTIVE")

            screen.addstr(10, 18, "MODE")
            if self.mode == "AUTO":
                screen.addstr(11, 18, self.mode, curses.A_BOLD)
            else:
                screen.addstr(11, 18, self.mode)

            screen.addstr(10, 34, "EBRAKE")
            if self.emergency_stop_is_on:
                screen.addstr(11, 34, "ON", curses.A_BOLD)
            else:
                screen.addstr(11, 34, "OFF")

            screen.addstr(10, 48, "DIRECTION")
            screen.addstr(11, 48, self.direction, curses.A_BOLD)


            screen.addstr(13, 0, "--- COMMANDS ------------------------------------------------------")

            screen.addstr(15, 2, "THROTTLE")
            screen.addstr(16, 2, str(format(self.throttle, '.1f')), curses.A_BOLD)

            screen.addstr(15, 18, "SW_TORQUE")
            screen.addstr(15, 34, "SW Angle")

            if self.steering_wheel_angle_control_active:
                screen.addstr(16, 34, str(format(self.steering_wheel_angle, '.0f')), curses.A_BOLD)
                screen.addstr(16, 18, "-")
            else:
                screen.addstr(16, 34, str(format(self.real_steering_wheel_angle, '.0f')))
                screen.addstr(16, 18, str(format(self.steering_wheel_torque, '.1f')), curses.A_BOLD)

            screen.addstr(18, 1, "")

            screen.refresh()
            rate.sleep()

        curses.endwin()

if __name__ == '__main__':

    teleop = OscarJoyTeleop()
    teleop.spin()
