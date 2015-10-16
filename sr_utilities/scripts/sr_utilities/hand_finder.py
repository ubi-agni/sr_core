#!/usr/bin/env python
#
# Copyright 2014 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#
import rospy
import rospkg
from local_urdf_parser_py import URDF


class HandControllerTuning(object):
    def __init__(self, config_dir):
        """

        """
        ros_pack = rospkg.RosPack()
        ethercat_path = ros_pack.get_path('sr_ethercat_hand_config')
        self.friction_compensation = {}
        self.host_control = {}
        self.motor_control = {}
        for hand_serial in config_dir:
            self.friction_compensation[hand_serial] = \
                ethercat_path + '/controls/' + 'friction_compensation.yaml'
            host_path = ethercat_path + '/controls/host/' + config_dir[hand_serial] + '/'
            self.host_control[hand_serial] = \
                [host_path + 'sr_edc_calibration_controllers.yaml',
                 host_path + 'sr_edc_joint_velocity_controllers_PWM.yaml',
                 host_path + 'sr_edc_effort_controllers_PWM.yaml',
                 host_path + 'sr_edc_joint_velocity_controllers.yaml',
                 host_path + 'sr_edc_effort_controllers.yaml',
                 host_path + 'sr_edc_mixed_position_velocity_'
                             'joint_controllers_PWM.yaml',
                 host_path + 'sr_edc_joint_position_controllers_PWM.yaml',
                 host_path + 'sr_edc_mixed_position_velocity_'
                             'joint_controllers.yaml',
                 host_path + 'sr_edc_joint_position_controllers.yaml']

            self.motor_control[hand_serial] = \
                ethercat_path + '/controls/motors/' +\
                config_dir[hand_serial] + '/motor_board_effort_controllers.yaml'


class HandCalibration(object):
    def __init__(self, config_dir):
        """

        """
        ros_pack = rospkg.RosPack()
        ethercat_path = ros_pack.get_path('sr_ethercat_hand_config')
        self.calibration_path = {}
        for hand_serial in config_dir:
            self.calibration_path[hand_serial] = \
                ethercat_path + '/calibrations/' + config_dir[hand_serial] + '/' \
                + "calibration.yaml"


class HandConfig(object):

    def __init__(self, config_dir, mapping, joint_prefix):
        """

        """
        self.config_dir = config_dir
        self.mapping = mapping
        self.joint_prefix = joint_prefix


class HandJoints(object):
    def __init__(self, joint_prefix):
        """

        """
        self.joints = {}
        hand_joints = []

        joints = ['FFJ1', 'FFJ2', 'FFJ3', 'FFJ4', 'MFJ1', 'MFJ2', 'MFJ3',
                  'MFJ4', 'RFJ1', 'RFJ2', 'RFJ3', 'RFJ4', 'LFJ1', 'LFJ2',
                  'LFJ3', 'LFJ4', 'LFJ5', 'THJ1', 'THJ2', 'THJ3', 'THJ4',
                  'THJ5', 'WRJ1', 'WRJ2']

       
        
        if rospy.has_param('robot_description'):
            robot_description = rospy.get_param('robot_description')
            
            # concatenate all the joints with prefixes
            for hand_serial in joint_prefix:
                for joint in joints:
                    hand_joints.append(joint_prefix[hand_serial] + joint)
                    
            # add the prefixed joints to each hand but remove fixed joints
            hand_urdf = URDF.from_xml_string(robot_description)
            for hand_serial in joint_prefix:
                joints_tmp = []
                self.joints[hand_serial] = []
                for joint in hand_urdf.joints:
                    if joint.type != 'fixed':
                        #TODO:use a split at _, why whould a prefix be of size two
                        prefix = joint.name[:2] + "_"
                        if prefix not in joint_prefix.values():
                            rospy.logdebug("joint " + joint.name + "has invalid "
                                           "prefix")
                        elif prefix == joint_prefix[hand_serial]:
                            joints_tmp.append(joint.name)
                for joint_unordered in hand_joints:
                    if joint_unordered in joints_tmp:
                        self.joints[hand_serial].append(joint_unordered)

        else:
            rospy.logwarn("No robot_description found on parameter server."
                          "Joint names are loaded for 5 finger hand")
            if len(joint_prefix) > 0:
                # concatenate all the joints with prefixes
                for hand_serial in joint_prefix:
                    hand_joints = []
                    for joint in joints:
                        hand_joints.append(joint_prefix[hand_serial] + joint)
                    self.joints[hand_serial] = hand_joints


class HandFinder(object):
    """
    The HandFinder is a utility library for detecting Shadow Hands running on
    the system. The idea is to make it easier to write generic code,
     using this library to handle prefixes, joint prefixes etc...
    """

    def __init__(self):
        """
        Parses the parameter server to extract the necessary information.
        """
        if not rospy.has_param("/hand"):
            rospy.logerr("No hand is detected")
            hand_parameters = {'joint_prefix': {}, 'mapping': {}, 'config_dir': {}}
        else:
            # hand param is always at root (shared between hands)
            hand_parameters = rospy.get_param("/hand")
        self.hand_config = HandConfig(hand_parameters["config_dir"],
                                      hand_parameters["mapping"],
                                      hand_parameters["joint_prefix"])
        self.hand_joints = HandJoints(self.hand_config.joint_prefix).joints
        self.calibration_path = \
            HandCalibration(self.hand_config.config_dir).calibration_path
        self.hand_control_tuning = \
            HandControllerTuning(self.hand_config.config_dir)

    def get_calibration_path(self):
        return self.calibration_path

    def get_hand_joints(self):
        return self.hand_joints

    def get_hand_parameters(self):
        return self.hand_config

    def get_hand_control_tuning(self):
        return self.hand_control_tuning
