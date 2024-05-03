#!/usr/bin/env python

# BSD 3-Clause License
# Copyright (c) 2024 SMART Research Group - Saxion University of Applied Sciences
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Vector3Stamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_srvs.srv import Trigger
from tkinter import Tk, Label, Entry, Button
from m4e_mav_trajectory_planner.srv import PlanTakeoff

class M4EGui:
    def __init__(self):
        self.loadParameters()
        self.waypoint_pub = rospy.Publisher(self.waypoint_topic, Vector3Stamped, queue_size=1)
        self.state_sub = rospy.Subscriber(self.state_topic, State, self.state_callback)

        self.setup_gui()

    def loadParameters(self):
        # Load topic names from parameters
        self.waypoint_topic = self.load_parameter("topics_names/waypoint_topic","waypoint")
        self.state_topic = self.load_parameter("topics_names/state_topic", "mavros/state")

        # Load service names from parameters
        self.arming_service = self.load_parameter('services_names/arming_client_name', 'mavros/cmd/arming')
        self.offboard_service = self.load_parameter('services_names/set_mode_client_name', 'mavros/set_mode')
        self.landing_service = self.load_parameter('services_names/landing_client_name', 'mavros/cmd/land')
        self.mani_arm_motors_service = self.load_parameter('manipulator_controller/services_names/arming_client_name', 'm4e_mani/m4e_ros_controller/enableAllMotors')
        self.mani_disarm_motors_service = self.load_parameter('manipulator_controller/services_names/disarming_client_name', 'm4e_mani/m4e_ros_controller/disableAllMotors')
        self.mani_zero_motors_service = self.load_parameter('manipulator_controller/services_names/zeroing_client_name', 'm4e_mani/m4e_ros_controller/zeroAllMotors')
        self.mani_fold_service = self.load_parameter('manipulator_controller/services_names/fold_arm_client_name', 'm4e_mani_user_control/foldArm')
        self.mani_extend_service = self.load_parameter('manipulator_controller/services_names/extend_arm_client_name', 'm4e_mani_user_control/extendArm')
        self.takeoff_service = self.load_parameter('services_names/plan_takeoff_client_name', 'plan_takeoff')
    
    def setup_gui(self):
        window = Tk()

        # Configure button size
        button_width = 15
        button_height = 2
        button_font = ('Arial', 12, 'bold')
        labels_font = ('Helvetica', 12, 'bold')
        lights_font = ('Helvetica', 11, 'bold')

        # Title
        title_font = ('Helvetica', 16, 'bold')
        title_label = Label(window, text='Sarax GUI', font=title_font)
        title_label.grid(row=0, column=2, padx=5, pady=5)

        # Takeoff
        takeoff_label = Label(window, text='Takeoff altitude:', font=labels_font)
        takeoff_label.grid(row=1, column=2, padx=5, pady=5)
        self.altitude_input = Entry(window, width=10)
        self.altitude_input.grid(row=1, column=3, padx=5, pady=5)
        takeoff_button = Button(window, text='Takeoff', command=self.takeoff_callback,
                                width=button_width, height=button_height, font=button_font)
        takeoff_button.grid(row=1, column=4, padx=5, pady=5)

        # Waypoint
        wp_label = Label(window, text='New waypoint:', font=labels_font)
        wp_label.grid(row=2, column=0, padx=5, pady=5)
        self.wp_x_input = Entry(window, width=10)
        self.wp_x_input.grid(row=2, column=1, padx=5, pady=5)
        self.wp_y_input = Entry(window, width=10)
        self.wp_y_input.grid(row=2, column=2, padx=5, pady=5)
        self.wp_z_input = Entry(window, width=10)
        self.wp_z_input.grid(row=2, column=3, padx=5, pady=5)
        waypoint_button = Button(window, text='Go to waypoint', command=self.waypoint_callback,
                                 width=button_width, height=button_height, font=button_font)
        waypoint_button.grid(row=2, column=4, padx=5, pady=5)

        # Status lights
        status_text = Label(window, text='Status: ', font=labels_font)
        status_text.grid(row=3, column=0, padx=5, pady=5)
        self.armed_light = Label(window, width=12, height=2, bg='green', text='Not Armed', font=lights_font)
        self.armed_light.grid(row=3, column=1, padx=5, pady=5)
        self.offboard_light = Label(window, width=12, height=2, bg='green', text='Not offboard', font=lights_font)
        self.offboard_light.grid(row=3, column=2, padx=5, pady=5)

        # Services
        services_text = Label(window, text='Services: ', font=labels_font)
        services_text.grid(row=4, column=0, padx=5, pady=5)
        arming_service_button = Button(window, text='Arm Service', command=self.arming_service_callback,
                                       width=button_width, height=button_height, font=button_font)
        arming_service_button.grid(row=4, column=1, padx=5, pady=5)
        offboard_service_button = Button(window, text='Offboard Service', command=self.offboard_service_callback,
                                         width=button_width, height=button_height, font=button_font)
        offboard_service_button.grid(row=4, column=2, padx=5, pady=5)
        disarming_service_button = Button(window, text='Disarm Service', command=self.disarming_service_callback,
                                          width=button_width, height=button_height, font=button_font)
        disarming_service_button.grid(row=4, column=3, padx=5, pady=5)
        landing_service_button = Button(window, text='Land Service', command=self.landing_service_callback,
                                        width=button_width, height=button_height, font=button_font)
        landing_service_button.grid(row=4, column=4, padx=5, pady=5)

        # Manipulator Services
        mani_services_text = Label(window, text='Manipulator Services: ', font=labels_font)
        mani_services_text.grid(row=5, column=0, padx=5, pady=5)
        mani_zero_motors_service_button = Button(window, text='Zero Motors', command=self.mani_zero_motors_service_callback,
                                       width=button_width, height=button_height, font=button_font)
        mani_zero_motors_service_button.grid(row=5, column=1, padx=5, pady=5)
        mani_arm_motors_service_button = Button(window, text='Arm Motors', command=self.mani_arm_motors_service_callback,
                                       width=button_width, height=button_height, font=button_font)
        mani_arm_motors_service_button.grid(row=5, column=2, padx=5, pady=5)
        mani_disarm_motors_service_button = Button(window, text='Disarm Motors', command=self.mani_disarm_motors_service_callback,
                                       width=button_width, height=button_height, font=button_font)
        mani_disarm_motors_service_button.grid(row=5, column=3, padx=5, pady=5)
        mani_fold_service_button = Button(window, text='Fold Arm', command=self.mani_fold_service_callback,
                                       width=button_width, height=button_height, font=button_font)
        mani_fold_service_button.grid(row=6, column=1, padx=5, pady=5)
        mani_extend_service_button = Button(window, text='Extend Arm', command=self.mani_extend_service_callback,
                                       width=button_width, height=button_height, font=button_font)
        mani_extend_service_button.grid(row=6, column=2, padx=5, pady=5)

        window.mainloop()

    def takeoff_callback(self):
        value = float(self.altitude_input.get())
        try:
            rospy.wait_for_service(self.takeoff_service, timeout=2)
        except:
            rospy.logerr('[gui_node] {} is not available yet!'.format(self.takeoff_service))
        try:
            service = rospy.ServiceProxy(self.takeoff_service, PlanTakeoff)
            service(value)
        except:
            rospy.logerr('[gui_node] PlanTakeoff Service call failed!')
        self.altitude_input.delete(0, 'end')  # to clear the text after clicking the button

    def waypoint_callback(self):
        valuex = float(self.wp_x_input.get())
        valuey = float(self.wp_y_input.get())
        valuez = float(self.wp_z_input.get())
        msg = self.create_vector3_stamped(valuex, valuey, valuez)
        self.waypoint_pub.publish(msg)
        self.wp_x_input.delete(0, 'end')  # to clear the text after clicking the button
        self.wp_y_input.delete(0, 'end')  # to clear the text after clicking the button
        self.wp_z_input.delete(0, 'end')  # to clear the text after clicking the button

    def state_callback(self, msg):
        if msg.armed:
            try:
                self.armed_light.config(bg="red", text="Armed")
            except:
                rospy.logerr("[m4e_gui] Arm indicator light failed.")
        else:
            try:
                self.armed_light.config(bg="green", text="Not Armed")
            except:
                rospy.logerr("[m4e_gui] Arm indicator light failed.")

        if msg.mode == "OFFBOARD":
            try:
                self.offboard_light.config(bg="red", text="Offboard")
            except:
                rospy.logerr("[m4e_gui] Offboard indicator light failed.")
        else:
            try:
                self.offboard_light.config(bg="green", text="Not Offboard")
            except:
                rospy.logerr("[m4e_gui] Offboard indicator light failed.")

    def arming_service_callback(self):
        try:
            rospy.wait_for_service(self.arming_service, timeout=2)
        except:
            rospy.logerr('[gui_node] {} is not available yet!').format(self.arming_service)
        try:
            service = rospy.ServiceProxy(self.arming_service, CommandBool)
            service(True)
        except rospy.ServiceException:
            rospy.logerr('[gui_node] Arming Service call failed!')

    def disarming_service_callback(self):
        try:
            rospy.wait_for_service(self.arming_service, timeout=2)
        except:
            rospy.logerr('[gui_node] {} is not available yet!'.format(self.arming_service))
        try:
            service = rospy.ServiceProxy(self.arming_service, CommandBool)
            service(False)
        except rospy.ServiceException:
            rospy.logerr('[gui_node] Disarming Service call failed!')

    def landing_service_callback(self):
        try:
            rospy.wait_for_service(self.landing_service, timeout=2)
        except:
            rospy.logerr('[gui_node] {} is not available yet!'.format(self.landing_service))
        try:
            service = rospy.ServiceProxy(self.landing_service, CommandTOL)
            service()
        except:
            rospy.logerr('[gui_node] Landing Service call failed!')
        msg = self.create_vector3_stamped(0, 0, -2)
        self.waypoint_pub.publish(msg)

    def offboard_service_callback(self):
        try:
            rospy.wait_for_service(self.offboard_service, timeout=2)
        except:
            rospy.logerr('[gui_node] {} is not available yet!'.format(self.offboard_service))
        try:
            service = rospy.ServiceProxy(self.offboard_service, SetMode)
            service(0, 'OFFBOARD')
        except:
            rospy.logerr('[gui_node] Offboard Service call failed!')
    
    def mani_zero_motors_service_callback(self):
        try:
            rospy.wait_for_service(self.mani_zero_motors_service, timeout=2)
        except:
            rospy.logerr('[gui_node] {} is not available yet!'.format(self.mani_zero_motors_service))
        try:
            service = rospy.ServiceProxy(self.mani_zero_motors_service, Trigger)
            service()
        except:
            rospy.logerr('[gui_node] Zero Manipulator Service call failed!')
    
    def mani_arm_motors_service_callback(self):
        try:
            rospy.wait_for_service(self.mani_arm_motors_service, timeout=2)
        except:
            rospy.logerr('[gui_node] {} is not available yet!'.format(self.mani_arm_motors_service))
        try:
            service = rospy.ServiceProxy(self.mani_arm_motors_service, Trigger)
            service()
        except:
            rospy.logerr('[gui_node] Disarm Manipulator Motors Service call failed!')

    def mani_disarm_motors_service_callback(self):
        try:
            rospy.wait_for_service(self.mani_disarm_motors_service, timeout=2)
        except:
            rospy.logerr('[gui_node] {} is not available yet!'.format(self.mani_disarm_motors_service))
        try:
            service = rospy.ServiceProxy(self.mani_disarm_motors_service, Trigger)
            service()
        except:
            rospy.logerr('[gui_node] Disarm Manipulator Motors Service call failed!')

    def mani_fold_service_callback(self):
        try:
            rospy.wait_for_service(self.mani_fold_service, timeout=2)
        except:
            rospy.logerr('[gui_node] {} is not available yet!'.format(self.mani_fold_service))
        try:
            service = rospy.ServiceProxy(self.mani_fold_service, Trigger)
            service()
        except:
            rospy.logerr('[gui_node] Fold Manipulator Service call failed!')

    def mani_extend_service_callback(self):
        try:
            rospy.wait_for_service(self.mani_extend_service, timeout=2)
        except:
            rospy.logerr('[gui_node] {} is not available yet!'.format(self.mani_extend_service))
        try:
            service = rospy.ServiceProxy(self.mani_extend_service, Trigger)
            service()
        except:
            rospy.logerr('[gui_node] Extend Manipulator Service call failed!')

    @staticmethod
    def create_vector3_stamped(x, y, z):
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.vector.x = x
        msg.vector.y = y
        msg.vector.z = z
        return msg
    
    def load_parameter(self, parameter_name, default_value):
        # Get the current namespace
        current_namespace = rospy.get_namespace()
        
        # Construct the full parameter name
        full_parameter_name = current_namespace + parameter_name
        
        # Check if the parameter exists
        if rospy.has_param(full_parameter_name):
            parameter_value = rospy.get_param(full_parameter_name)
            return parameter_value
        else:
            rospy.logwarn("[gui_node] Parameter {} does not exist".format(full_parameter_name))
            return (current_namespace + default_value)

if __name__ == '__main__':
    rospy.init_node('m4e_gui_node')
    gui = M4EGui()