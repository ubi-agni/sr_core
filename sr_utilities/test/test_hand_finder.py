#!/usr/bin/env python

import rospy, rospkg
import rostest
import unittest
from sr_utilities.hand_finder import HandFinder
joint_names = ["FFJ1", "FFJ2", "FFJ3", "FFJ4", "MFJ1", "MFJ2", "MFJ3", "MFJ4",
                                "RFJ1", "RFJ2", "RFJ3", "RFJ4", "LFJ1", "LFJ2", "LFJ3", "LFJ4", "LFJ5",
                                "THJ1", "THJ2", "THJ3", "THJ4", "THJ5", "WRJ1", "WRJ2"]
controller_params = ["sr_edc_calibration_controllers.yaml",
                           "sr_edc_joint_velocity_controllers_PWM.yaml",
                           "sr_edc_effort_controllers_PWM.yaml",
                           "sr_edc_joint_velocity_controllers.yaml",
                           "sr_edc_effort_controllers.yaml",
                           "sr_edc_mixed_position_velocity_joint_controllers_PWM.yaml",
                           "sr_edc_joint_position_controllers_PWM.yaml",
                           "sr_edc_mixed_position_velocity_joint_controllers.yaml",
                           "sr_edc_joint_position_controllers.yaml"]

class TestHandFinder(unittest.TestCase):
    rospack = rospkg.RosPack()
    ethercat_path = rospack.get_path('sr_ethercat_hand_config')
    
    def test_no_hand_finder(self):
        if rospy.has_param("/hand"):
            rospy.delete_param("/hand")

        hand_finder = HandFinder()

        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix),
                         0, "correct parameters without a hand")
        self.assertEqual(len(hand_finder.get_hand_parameters().mapping),
                         0, "correct parameters without a hand")
        self.assertEqual(len(hand_finder.get_hand_joints()), 0,
                         "correct joints without a hand")
        self.assertEqual(len(hand_finder.get_calibration_path()), 0,
                         "correct calibration path without a hand")
        self.assertEqual(len(hand_finder.get_hand_control_tuning().
                             friction_compensation), 0,
                         "correct tuning without a hands")
        self.assertEqual(len(hand_finder.get_hand_control_tuning().
                             host_control), 0,
                         "correct tuning without a hands")
        self.assertEqual(len(hand_finder.get_hand_control_tuning().
                             motor_control), 0,
                         "correct tuning without a hands")

    def test_one_hand_finder(self):
        rospy.set_param("hand/config_dir/1", "rh_motor")
        rospy.set_param("hand/joint_prefix/1", "rh_")
        rospy.set_param("hand/mapping/1", "right")

        hand_finder = HandFinder()
        self.assertIsNotNone(hand_finder.get_hand_parameters(),
                             "Parameters extracted.")
        self.assertIsNotNone(hand_finder.get_hand_joints(),
                             "Joints extracted.")
        joints = hand_finder.get_hand_joints()['1']
        self.assertEqual(len(joints), 24, "incorrect number of joints")
        self.assertEqual(joints[0], ("rh_" + joint_names[0]),"incorrect joint_name")
        self.assertIsNotNone(hand_finder.get_calibration_path(),
                             "Calibration extracted.")
        calibration_path = hand_finder.get_calibration_path()['1']
        self.assertEqual(calibration_path, self.ethercat_path + "/calibrations/rh_motor/" + "calibration.yaml","incorrect calibration file")
        self.assertIsNotNone(hand_finder.get_hand_control_tuning(),
                             "Control tuning parameters extracted.")
        ctrl_tun_friction_comp_path = hand_finder.get_hand_control_tuning().friction_compensation['1']
        self.assertEqual(ctrl_tun_friction_comp_path, self.ethercat_path + "/controls/friction_compensation.yaml","incorrect friction compensation file")
        
        ctrl_tun_motors_path = hand_finder.get_hand_control_tuning().motor_control['1']
        self.assertEqual(ctrl_tun_motors_path, self.ethercat_path + "/controls/motors/rh_motor/motor_board_effort_controllers.yaml","incorrect motor config file")
        
        ctrl_tun_host_control_paths = hand_finder.get_hand_control_tuning().host_control['1']
        
        self.assertEqual(len(ctrl_tun_host_control_paths), len(controller_params), "incorrect number of host controllers param")
        for controller_path, controller_param in zip(ctrl_tun_host_control_paths, controller_params):
            self.assertEqual(controller_path, self.ethercat_path +"/controls/host/rh_motor/"+ controller_param,"incorrect controller config file")
        
    
    def test_one_hand_finder_no_ns(self):
        rospy.set_param("hand/config_dir/1", "rh_motor")
        rospy.set_param("hand/joint_prefix/1", "rh_")
        rospy.set_param("hand/mapping/1", "")

        hand_finder = HandFinder()
        self.assertIsNotNone(hand_finder.get_hand_parameters(),
                             "Parameters extracted.")
        self.assertIsNotNone(hand_finder.get_hand_joints(),
                             "Joints extracted.")
        joints = hand_finder.get_hand_joints()['1']
        self.assertEqual(len(joints), 24, "incorrect number of joints")
        self.assertEqual(joints[0], ("rh_" + joint_names[0]),"incorrect joint_name")
        self.assertIsNotNone(hand_finder.get_calibration_path(),
                             "Calibration extracted.")
        calibration_path = hand_finder.get_calibration_path()['1']
        self.assertEqual(calibration_path, self.ethercat_path + "/calibrations/rh_motor/" + "calibration.yaml","incorrect calibration file")
        self.assertIsNotNone(hand_finder.get_hand_control_tuning(),
                             "Control tuning parameters extracted.")
        ctrl_tun_friction_comp_path = hand_finder.get_hand_control_tuning().friction_compensation['1']
        self.assertEqual(ctrl_tun_friction_comp_path, self.ethercat_path + "/controls/friction_compensation.yaml","incorrect friction compensation file")
        
        ctrl_tun_motors_path = hand_finder.get_hand_control_tuning().motor_control['1']
        self.assertEqual(ctrl_tun_motors_path, self.ethercat_path + "/controls/motors/rh_motor/motor_board_effort_controllers.yaml","incorrect motor config file")
        
        ctrl_tun_host_control_paths = hand_finder.get_hand_control_tuning().host_control['1']
        
        self.assertEqual(len(ctrl_tun_host_control_paths), len(controller_params), "incorrect number of host controllers param")
        for controller_path, controller_param in zip(ctrl_tun_host_control_paths, controller_params):
            self.assertEqual(controller_path, self.ethercat_path +"/controls/host/rh_motor/"+ controller_param,"incorrect controller config file")

    def test_one_hand_finder_no_prefix(self):
        rospy.set_param("hand/config_dir/1", "rh_motor")
        rospy.set_param("hand/joint_prefix/1", "")
        rospy.set_param("hand/mapping/1", "rh")

        hand_finder = HandFinder()
        self.assertIsNotNone(hand_finder.get_hand_parameters(),
                             "Parameters extracted.")
        self.assertIsNotNone(hand_finder.get_hand_joints(),
                             "Joints extracted.")
        joints = hand_finder.get_hand_joints()['1']
        self.assertEqual(len(joints), 24, "incorrect number of joints")
        self.assertEqual(joints[0], joint_names[0],"incorrect joint_name")
        self.assertIsNotNone(hand_finder.get_calibration_path(),
                             "Calibration extracted.")
        calibration_path = hand_finder.get_calibration_path()['1']
        self.assertEqual(calibration_path, self.ethercat_path + "/calibrations/rh_motor/" + "calibration.yaml","incorrect calibration file")
        self.assertIsNotNone(hand_finder.get_hand_control_tuning(),
                             "Control tuning parameters extracted.")
        ctrl_tun_friction_comp_path = hand_finder.get_hand_control_tuning().friction_compensation['1']
        self.assertEqual(ctrl_tun_friction_comp_path, self.ethercat_path + "/controls/friction_compensation.yaml","incorrect friction compensation file")
        
        ctrl_tun_motors_path = hand_finder.get_hand_control_tuning().motor_control['1']
        self.assertEqual(ctrl_tun_motors_path, self.ethercat_path + "/controls/motors/rh_motor/motor_board_effort_controllers.yaml","incorrect motor config file")
        
        ctrl_tun_host_control_paths = hand_finder.get_hand_control_tuning().host_control['1']
        
        self.assertEqual(len(ctrl_tun_host_control_paths), len(controller_params), "incorrect number of host controllers param")
        for controller_path, controller_param in zip(ctrl_tun_host_control_paths, controller_params):
            self.assertEqual(controller_path, self.ethercat_path +"/controls/host/rh_motor/"+ controller_param,"incorrect controller config file")
 
    def test_one_hand_finder_no_prefix_no_ns(self):
        rospy.set_param("hand/config_dir/1", "rh_motor")
        rospy.set_param("hand/joint_prefix/1", "")
        rospy.set_param("hand/mapping/1", "")

        hand_finder = HandFinder()
        self.assertIsNotNone(hand_finder.get_hand_parameters(),
                             "Parameters extracted.")
        self.assertIsNotNone(hand_finder.get_hand_joints(),
                             "Joints extracted.")
        joints = hand_finder.get_hand_joints()['1']
        self.assertEqual(len(joints), 24, "incorrect number of joints")
        self.assertEqual(joints[0], joint_names[0],"incorrect joint_name")
        self.assertIsNotNone(hand_finder.get_calibration_path(),
                             "Calibration extracted.")
        calibration_path = hand_finder.get_calibration_path()['1']
        self.assertEqual(calibration_path, self.ethercat_path + "/calibrations/rh_motor/" + "calibration.yaml","incorrect calibration file")
        self.assertIsNotNone(hand_finder.get_hand_control_tuning(),
                             "Control tuning parameters extracted.")
        ctrl_tun_friction_comp_path = hand_finder.get_hand_control_tuning().friction_compensation['1']
        self.assertEqual(ctrl_tun_friction_comp_path, self.ethercat_path + "/controls/friction_compensation.yaml","incorrect friction compensation file")
        
        ctrl_tun_motors_path = hand_finder.get_hand_control_tuning().motor_control['1']
        self.assertEqual(ctrl_tun_motors_path, self.ethercat_path + "/controls/motors/rh_motor/motor_board_effort_controllers.yaml","incorrect motor config file")
        
        ctrl_tun_host_control_paths = hand_finder.get_hand_control_tuning().host_control['1']
        
        self.assertEqual(len(ctrl_tun_host_control_paths), len(controller_params), "incorrect number of host controllers param")
        for controller_path, controller_param in zip(ctrl_tun_host_control_paths, controller_params):
            self.assertEqual(controller_path, self.ethercat_path +"/controls/host/rh_motor/"+ controller_param,"incorrect controller config file")

    def test_two_hand_finder(self):
        rospy.set_param("hand/config_dir/1", "rh_motor")
        rospy.set_param("hand/joint_prefix/1", "rh_")
        rospy.set_param("hand/mapping/1", "right")
        rospy.set_param("hand/config_dir/2", "lh_motor")
        rospy.set_param("hand/joint_prefix/2", "lh_")
        rospy.set_param("hand/mapping/2", "left")

        hand_finder = HandFinder()
        self.assertIsNotNone(hand_finder.get_hand_parameters(),
                             "Parameters extracted.")
        self.assertIsNotNone(hand_finder.get_hand_joints(),
                             "Joints extracted.")
        joints = hand_finder.get_hand_joints()['1']
        self.assertEqual(len(joints), 24, "incorrect number of joints")
        self.assertEqual(joints[0], ("rh_" + joint_names[0]),"incorrect joint_name")
        joints = hand_finder.get_hand_joints()['2']
        self.assertEqual(len(joints), 24, "incorrect number of joints")
        self.assertEqual(joints[0], ("lh_" + joint_names[0]),"incorrect joint_name")
        self.assertIsNotNone(hand_finder.get_calibration_path(),
                             "Calibration extracted.")
        calibration_path = hand_finder.get_calibration_path()['1']
        self.assertEqual(calibration_path, self.ethercat_path + "/calibrations/rh_motor/" + "calibration.yaml","incorrect calibration file")
        calibration_path = hand_finder.get_calibration_path()['2']
        self.assertEqual(calibration_path, self.ethercat_path + "/calibrations/lh_motor/" + "calibration.yaml","incorrect calibration file")
        
        ctrl_tun_friction_comp_path = hand_finder.get_hand_control_tuning().friction_compensation['1']
        self.assertEqual(ctrl_tun_friction_comp_path, self.ethercat_path + "/controls/friction_compensation.yaml","incorrect friction compensation file")
        
        ctrl_tun_motors_path = hand_finder.get_hand_control_tuning().motor_control['1']
        self.assertEqual(ctrl_tun_motors_path, self.ethercat_path + "/controls/motors/rh_motor/motor_board_effort_controllers.yaml","incorrect motor config file")
        
        ctrl_tun_host_control_paths = hand_finder.get_hand_control_tuning().host_control['1']
        
        self.assertEqual(len(ctrl_tun_host_control_paths), len(controller_params), "incorrect number of host controllers param")
        for controller_path, controller_param in zip(ctrl_tun_host_control_paths, controller_params):
            self.assertEqual(controller_path, self.ethercat_path +"/controls/host/rh_motor/"+ controller_param,"incorrect controller config file")
        
        ctrl_tun_friction_comp_path = hand_finder.get_hand_control_tuning().friction_compensation['2']
        self.assertEqual(ctrl_tun_friction_comp_path, self.ethercat_path + "/controls/friction_compensation.yaml","incorrect friction compensation file")
        
        ctrl_tun_motors_path = hand_finder.get_hand_control_tuning().motor_control['2']
        self.assertEqual(ctrl_tun_motors_path, self.ethercat_path + "/controls/motors/lh_motor/motor_board_effort_controllers.yaml","incorrect motor config file")
        
        ctrl_tun_host_control_paths = hand_finder.get_hand_control_tuning().host_control['2']
        
        self.assertEqual(len(ctrl_tun_host_control_paths), len(controller_params), "incorrect number of host controllers param")
        for controller_path, controller_param in zip(ctrl_tun_host_control_paths, controller_params):
            self.assertEqual(controller_path, self.ethercat_path +"/controls/host/lh_motor/"+ controller_param,"incorrect controller config file")


if __name__ == "__main__":
    rospy.init_node("test_hand_finder")
    rostest.rosrun("sr_utilities", "test_hand_finder", TestHandFinder)
