#!/usr/bin/env python
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *
from time import sleep
from threading import Event

import platform
import sys
import socket
import struct
import json

import rospy
import rosparam
import rospkg
from std_msgs.msg import Bool, UInt8, Int8, Float32, Duration
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, TransformStamped



class MetaWearRosSimple:
    def __init__(self, mac_adr):


        # Create publisher 
        self.frame_id = rospy.get_name().split('/')[-1]

        self.pub_rotation = rospy.Publisher(self.frame_id + '/rotation', QuaternionStamped, queue_size = 10)
        self.pub_accel = rospy.Publisher(self.frame_id + '/accel', Vector3Stamped, queue_size = 10)
        self.pub_gyro = rospy.Publisher(self.frame_id + '/gyro', Vector3Stamped, queue_size = 10)
        self.pub_g = rospy.Publisher(self.frame_id + '/gravity', Vector3Stamped, queue_size = 10)
        self.pub_acc_lin = rospy.Publisher(self.frame_id + '/lin_acc', Vector3Stamped, queue_size = 10)

        d = MetaWear(mac_adr)
        d.connect()
        print("Connected to " + d.address)

        self.device = d
        self.samples_acc = 0
        self.samples_rot = 0
        self.samples_gyro = 0

        self.cb_gyro_cpp =  FnVoid_VoidP_DataP(self.gyro_cb)
        self.cb_acc_cpp =   FnVoid_VoidP_DataP(self.acc_cb)
        self.cb_rot_cpp =   FnVoid_VoidP_DataP(self.rot_cb)
        self.cb_g_cpp =   FnVoid_VoidP_DataP(self.g_cb)
        self.cb_acc_lin_cpp =   FnVoid_VoidP_DataP(self.acc_lin_cb)



        print("Configuring device")
        libmetawear.mbl_mw_settings_set_connection_parameters(self.device.board, 10, 20, 0, 6000)

        print("Sensor Fusion")
        libmetawear.mbl_mw_sensor_fusion_set_mode(self.device.board, SensorFusionMode.IMU_PLUS)
        libmetawear.mbl_mw_sensor_fusion_set_acc_range(self.device.board, SensorFusionAccRange._8G)
        libmetawear.mbl_mw_sensor_fusion_write_config(self.device.board)
        rospy.sleep(1)

        #self.gyro_signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(self.device.board, SensorFusionData.CORRECTED_GYRO)
        #self.acc_signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(self.device.board, SensorFusionData.CORRECTED_ACC)
        self.rot_signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(self.device.board, SensorFusionData.QUATERNION)
        #self.g_signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(self.device.board, SensorFusionData.GRAVITY_VECTOR)
        #self.acc_lin_signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(self.device.board, SensorFusionData.LINEAR_ACC)

        rospy.sleep(1)

        #libmetawear.mbl_mw_datasignal_subscribe(self.gyro_signal, None, self.cb_gyro_cpp)
        #libmetawear.mbl_mw_datasignal_subscribe(self.acc_signal, None, self.cb_acc_cpp)
        libmetawear.mbl_mw_datasignal_subscribe(self.rot_signal, None, self.cb_rot_cpp)
        #libmetawear.mbl_mw_datasignal_subscribe(self.g_signal, None, self.cb_g_cpp)
        #libmetawear.mbl_mw_datasignal_subscribe(self.acc_lin_signal, None, self.cb_acc_lin_cpp)
        
        rospy.sleep(1)

        #libmetawear.mbl_mw_sensor_fusion_enable_data(self.device.board, SensorFusionData.CORRECTED_GYRO)
        #libmetawear.mbl_mw_sensor_fusion_enable_data(self.device.board, SensorFusionData.CORRECTED_ACC)
        libmetawear.mbl_mw_sensor_fusion_enable_data(self.device.board, SensorFusionData.QUATERNION)
        #libmetawear.mbl_mw_sensor_fusion_enable_data(self.device.board, SensorFusionData.GRAVITY_VECTOR)
        #libmetawear.mbl_mw_sensor_fusion_enable_data(self.device.board, SensorFusionData.LINEAR_ACC)

        rospy.sleep(1)

        libmetawear.mbl_mw_sensor_fusion_start(self.device.board)

    def gyro_cb(self, ctx, data):
        # print("%s -> %s" % (self.device.address, parse_value(data)))
        self.samples_gyro += 1
        temp_data = parse_value(data)
        gyro = Vector3Stamped()
        gyro.header.stamp = rospy.Time.now()
        gyro.header.frame_id = self.frame_id
        gyro.vector.x = temp_data.x
        gyro.vector.y = temp_data.y
        gyro.vector.z = temp_data.z

        # print("gyro; (%.4f,%.4f,%.4f) " % (temp_data.x, temp_data.y, temp_data.z) )
        self.pub_gyro.publish(gyro)

    def acc_cb(self, ctx, data):
        self.samples_acc += 1
        temp_data = parse_value(data)
        # print("acc; (%.4f,%.4f,%.4f) " % (temp_data.x, temp_data.y, temp_data.z) )
        acc = Vector3Stamped()
        acc.header.stamp = rospy.Time.now()
        acc.header.frame_id = self.frame_id
        acc.vector.x = temp_data.x
        acc.vector.y = temp_data.y
        acc.vector.z = temp_data.z
        self.pub_accel.publish(acc)

    def g_cb(self, ctx, data):
        self.samples_acc += 1
        temp_data = parse_value(data)
        # print("acc; (%.4f,%.4f,%.4f) " % (temp_data.x, temp_data.y, temp_data.z) )
        # print(temp_data)
        acc = Vector3Stamped()
        acc.header.stamp = rospy.Time.now()
        acc.header.frame_id = self.frame_id
        acc.vector.x = temp_data.x
        acc.vector.y = temp_data.y
        acc.vector.z = temp_data.z
        self.pub_g.publish(acc) 

    def acc_lin_cb(self, ctx, data):

        self.samples_acc += 1
        temp_data = parse_value(data)
        # print("acc; (%.4f,%.4f,%.4f) " % (temp_data.x, temp_data.y, temp_data.z) )
        # print(temp_data)
        acc = Vector3Stamped()
        acc.header.stamp = rospy.Time.now()
        acc.header.frame_id = self.frame_id
        acc.vector.x = temp_data.x
        acc.vector.y = temp_data.y
        acc.vector.z = temp_data.z
        self.pub_acc_lin.publish(acc)

    
    def rot_cb(self, ctx, data):
        temp_data = parse_value(data)

        quat = QuaternionStamped()
        quat.header.stamp = rospy.Time.now()
        quat.header.frame_id = self.frame_id
        quat.quaternion.x = temp_data.x
        quat.quaternion.y = temp_data.y
        quat.quaternion.z = temp_data.z
        quat.quaternion.w = temp_data.w

        self.samples_rot += 1
        # print("rot; (%.4f,%.4f,%.4f, %.4f) " % (temp_data.w, temp_data.x, temp_data.y, temp_data.z) )
        self.pub_rotation.publish(quat)

    def run(self):
        loop_rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            loop_rate.sleep()
        print("Disconnect")
        libmetawear.mbl_mw_sensor_fusion_stop(self.device.board)
        #libmetawear.mbl_mw_datasignal_unsubscribe(self.gyro_signal)
        #libmetawear.mbl_mw_datasignal_unsubscribe(self.acc_signal)
        libmetawear.mbl_mw_datasignal_unsubscribe(self.rot_signal)
        #libmetawear.mbl_mw_datasignal_unsubscribe(self.acc_lin_signal)
        #libmetawear.mbl_mw_datasignal_unsubscribe(self.g_signal)

        libmetawear.mbl_mw_debug_disconnect(self.device.board)
            

if __name__ == '__main__':
    #rosrun metawear_ros metawear_node.py _address:=E5:4D:16:18:CD:90
    mac_adr = sys.argv[1].split('_address:=')[1]
    name = mac_adr.replace(':','_')
    print(name)
    rospy.init_node('metawear_ros_'+ name, anonymous = False)

    mw = MetaWearRosSimple(mac_adr)
    mw.run()
   