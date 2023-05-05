# -*-coding:utf-8-*-

from robomaster import robot
from robomaster import config
import numpy as np

import time

# def uwb_callback(sub_info):
#     print(sub_info)
#     id, pox_x, pox_y, pox_z, vel_x, vel_y, vel_z, eop_x, eop_y, eop_z = sub_info
#     print("uwb info, id:{0} pox_x:{1}, pox_y:{2}, pox_z:{3}, vel_x:{4}, vel_y:{5}, vel_z:{6}, eop_x:{7}, eop_y:{8}, eop_z:{9}".\
#           format(id, pox_x, pox_y, pox_z, vel_x, vel_y, vel_z, eop_x, eop_y, eop_z))


# def imu_callback(sub_info):
#     print(sub_info)


# def velocity_callback(sub_info):
#     print(sub_info)


# def position_callback(sub_info):
#     print(sub_info)


class EKFLocalization():
    def __init__(self, ep_chassis=None):
        self.p_x = 0.0
        self.p_y = 0.0
        self.p_theta = 0.0
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_theta = 0.0

        self.Q = np.diag([1.0e-6, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-6])
        self.R = np.diag([1.0e-8, 1.0e-8, 10.0, 10.0, 1.0e-4, 1.0e-4])
        self.P = np.zeros([7,7])

        self.current_vector = np.array([0.0, 0.0, 0.0, 0.0])
        
        self.measure_vector = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # vx, vy, uwb_x, uwb_y, odom_x, odom_y
        self.state_vector = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # vx, vy, px, py, delta_px, delta_py, delta_yaw

        self.last_vector = np.array([0.0, 0.0, 0.0, 0.0])
        self.prediction_vector = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.current_yaw = None
        self.init_ekf_pos = False

        self.ready_prediction = False
        self.ready_pos = False
        self.ready_velocity = False

        self.ep_chassis = ep_chassis
        self.uwb_position_list = []
        self.odom_position_list = []
        self.init_coord = False
        self.uwb_pos_x = None
        self.uwb_pos_y = None

        self.init_yaw = None
        self.init_x = None
        self.init_y = None

        self.world_x = None
        self.world_y = None
        self.world_yaw = None

        # time
        self.last_uwb_time = None
        self.delta_uwb_time = None
        self.last_vel_time = None
        self.delta_vel_time = None

        # file for debug
        self.ekf_pos_f = open("ekf_pos.txt", "w")
        self.odom_pos_f = open("odom_pos.txt", "w")
        self.odom_vel_f = open("odom_vel.txt", "w")
        self.uwb_pos_f = open("uwb_pos.txt", "w")
        self.P_f = open("P_matrix.txt", "w")
        self.K_f = open("P_matrix.txt", "w")
    
    def close(self):
        self.ekf_pos_f.close()
        self.odom_pos_f.close()
        self.uwb_pos_f.close()
        self.P_f.close()
        self.K_f.close()

    def data_collector(self, d=1.0):
        if self.ep_chassis is not None:
            self.ep_chassis.move(x=d, y=0, z=0, xy_speed=1.0).wait_for_completed()
            self.ep_chassis.move(x=0, y=-d, z=0, xy_speed=1.0).wait_for_completed()
            self.ep_chassis.move(x=-d, y=0, z=0, xy_speed=1.0).wait_for_completed()
            self.ep_chassis.move(x=0, y=d, z=0, xy_speed=1.0).wait_for_completed()
        else:
            print("cannot handle ep_chassis!")
    
    def init_coordinate(self):
        self.data_collector()
        if len(self.odom_position_list)>0:
            assert len(self.odom_position_list) == len(self.uwb_position_list)
            odom_vec = np.array(self.odom_position_list)
            uwb_vec = np.array(self.uwb_position_list)
            odom_mat = np.dot(odom_vec.T, odom_vec)
            uwb_mat = np.dot(uwb_vec.T, odom_vec)
            param_mat = np.dot(uwb_mat, np.linalg.inv(odom_mat))
            self.init_yaw = np.arctan2(param_mat[1, 0], param_mat[0, 0])
            self.init_x = param_mat[0, 2]
            self.init_y = param_mat[1, 2]
            self.init_coord = True
            print("Init coordinate complete!")
            print("Init matrix: [{}, {}, {}]".format(param_mat[0,0], param_mat[0,1], param_mat[0,2]))
            print("             [{}, {}, {}]".format(param_mat[1,0], param_mat[1,1], param_mat[1,2]))
            print("             [{}, {}, {}]".format(param_mat[2,0]      , param_mat[2,1], param_mat[2,2]))
            print("init yaw: {}, init x: {}, init y: {}".format(self.init_yaw, self.init_x, self.init_y))
        else:
            print("No data used to init coordinate!")
    
    def prediction(self, dt=0.02):
        # A = np.array([[1, 0, 0, 0],
        #               [0, 1, 0, 0],
        #               [np.cos(self.current_yaw)*dt, -np.sin(self.current_yaw)*dt, 1, 0],
        #               [np.sin(self.current_yaw)*dt,  np.cos(self.current_yaw)*dt, 0, 1]])

        cos_yaw = np.cos(self.current_yaw+self.state_vector[-1])
        sin_yaw = np.sin(self.current_yaw+self.state_vector[-1])
        jacobA = np.array([[1, 0, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0, 0],
                           [cos_yaw*dt, -sin_yaw*dt, 1, 0, 0, 0, -self.state_vector[0]*sin_yaw*dt-self.state_vector[1]*cos_yaw*dt],
                           [sin_yaw*dt,  cos_yaw*dt, 0, 1, 0, 0,  self.state_vector[0]*cos_yaw*dt-self.state_vector[1]*sin_yaw*dt],
                           [0, 0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 0, 1]])

        # self.prediction_vector = np.dot(A, self.last_vector)
        self.prediction_vector[0] = self.state_vector[0]
        self.prediction_vector[1] = self.state_vector[1]
        self.prediction_vector[2] = self.state_vector[2] + self.state_vector[0]*cos_yaw*dt - self.state_vector[1]*sin_yaw*dt
        self.prediction_vector[3] = self.state_vector[3] + self.state_vector[0]*sin_yaw*dt + self.state_vector[1]*cos_yaw*dt
        # print("-----------------9-------------------")
        self.prediction_vector[4] = self.state_vector[4]
        self.prediction_vector[5] = self.state_vector[5]
        self.prediction_vector[6] = self.state_vector[6]
        # print("-----------------6-------------------")
        self.P = np.dot(jacobA, np.dot(self.P, jacobA.T)) + self.Q
        # print("-----------------7-------------------")
    
    def updateEKF(self):
        # self.P_f.write("{}, {}, {}, {}, {}, {}, {}, {}, {} \n".format(self.P[0,0], self.P[0,1], self.P[0,2],
        #                                                               self.P[1,0], self.P[1,1], self.P[1,2],
        #                                                               self.P[2,0], self.P[2,1], self.P[2,2]))
        # H = np.array([[1, 0, 0, 0],
        #               [0, 1, 0, 0],
        #               [0, 0, 1, 0],
        #               [0, 0, 0, 1]])
        cos_yaw = np.cos(self.prediction_vector[-1])
        sin_yaw = np.sin(self.prediction_vector[-1])
        # print("-----------1-----------")
        # print(self.state_vector)
        jacobH = np.array([[1., 0., 0., 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0, 0],
                           [0, 0, cos_yaw, sin_yaw, -cos_yaw, -sin_yaw, -(self.prediction_vector[2]-self.prediction_vector[4])*sin_yaw+(self.prediction_vector[3]-self.prediction_vector[5])*cos_yaw],
                           [0, 0, -sin_yaw, cos_yaw, sin_yaw, -cos_yaw, -(self.prediction_vector[2]-self.prediction_vector[4])*cos_yaw-(self.prediction_vector[3]-self.prediction_vector[5])*sin_yaw]])
        h = np.array([0., 0., 0., 0., 0., 0.])
        h[0] = self.prediction_vector[0]
        h[1] = self.prediction_vector[1]
        h[2] = self.prediction_vector[2]
        h[3] = self.prediction_vector[3]
        h[4] = (self.prediction_vector[2]-self.prediction_vector[4])*cos_yaw + (self.prediction_vector[3]-self.prediction_vector[5])*sin_yaw
        h[5] = -(self.prediction_vector[2]-self.prediction_vector[4])*sin_yaw + (self.prediction_vector[3]-self.prediction_vector[5])*cos_yaw
        
        # print(h)
        # print(jacobH.shape, self.P.shape, self.R.shape)
        part_1 = np.dot(jacobH, np.dot(self.P, jacobH.T)) + self.R
        # print(part_1)
        K = np.dot(self.P, np.dot(jacobH.T, np.linalg.inv(part_1)))

        # print("-------------2-------------")
        self.state_vector = self.prediction_vector + np.dot(K, (self.measure_vector-h))
        
        # self.K_f.write("{}, {}, {}, {}, {}, {}, {}, {}, {} \n".format(K[0,0], K[0,1], K[0,2],
        #                                                               K[1,0], K[1,1], K[1,2],
        #                                                               K[2,0], K[2,1], K[2,2]))
        print(self.state_vector)
        self.ekf_pos_f.write("{}, {}, {}, {} \n".format(self.state_vector[2], self.state_vector[3], self.state_vector[0], self.state_vector[1]))
        part_2 = np.eye(7) - np.dot(K, jacobH)
        self.P = np.dot(part_2, self.P)
        self.world_x = self.state_vector[2]
        self.world_y = self.state_vector[3]
        self.world_yaw = self.current_yaw + self.state_vector[-1]

    
    def uwb_callback(self, sub_info):
        current_time = time.time()
        if self.last_uwb_time is None:
            self.last_uwb_time = current_time
        else:
            self.delta_uwb_time = current_time - self.last_uwb_time
            self.last_uwb_time = current_time
            # print("delta uwb time: ", self.delta_uwb_time)

        id, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, eop_x, eop_y, eop_z = sub_info
        # print("UWB callback: {}, {}".format(pos_x, pos_y))
        self.uwb_pos_x = pos_x
        self.uwb_pos_y = pos_y
        if self.init_coordinate and self.init_ekf_pos:
            self.current_vector[-2] = pos_x
            self.current_vector[-1] = pos_y

            self.measure_vector[2] = pos_x
            self.measure_vector[3] = pos_y

            self.ready_pos = True
            self.uwb_pos_f.write("{}, {} \n".format(pos_x, pos_y))

            if (self.ready_velocity) and (self.delta_uwb_time is not None):
                # print("EKF update")
                self.prediction(dt=self.delta_uwb_time)
                self.updateEKF()
                # self.ready_velocity = False
                # self.ready_pos = False
        
    
    def imu_callback(self, sub_info):
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = sub_info
        # self.current_vector[2] = gyro_z
        # print("IMU callback: {}, {}, {}".format(acc_x, acc_y, gyro_z))
    
    def velocity_callback(self, sub_info):
        current_time = time.time()
        if self.last_vel_time is None:
            self.last_vel_time = current_time
        else:
            self.delta_vel_time = current_time - self.last_vel_time
            self.last_vel_time = current_time
            # print("delta velocity time: ", self.delta_vel_time)
        vgx, vgy, vgz, vbx, vby, vbz = sub_info
        if self.init_coordinate and self.init_ekf_pos:
            self.current_vector[0] = vbx
            self.current_vector[1] = vby
            self.measure_vector[0] = vbx
            self.measure_vector[1] = vby
            self.ready_velocity = True
            self.odom_vel_f.write("{}, {} \n".format(vbx, vby))
        # print("velocity callback: {}, {}".format(vbx, vby))

    def attitude_calllback(self, sub_info):
        yaw, pitch, roll = sub_info
        # print("attitude callback: {}, {}, {}".format(yaw, pitch, roll))
        if self.init_coord:
            # self.current_yaw = np.pi * yaw / 180 + self.init_yaw
            self.current_yaw = np.pi * yaw / 180
    
    def position_callback(self, sub_info):
        x, y, theta = sub_info
        self.measure_vector[4] = x
        self.measure_vector[5] = y
        # print("position callback: {}, {}, {}".format(x, y, theta))
        if not self.init_coord:
            if len(self.odom_position_list) > 0:
                last_position = self.odom_position_list[-1]
                dist = np.sqrt((x-last_position[0])**2 + (y-last_position[1])**2)
                if dist>0.1 and (self.uwb_pos_x is not None):
                    self.odom_position_list.append([x, y, 1])
                    self.uwb_position_list.append([self.uwb_pos_x, self.uwb_pos_y, 1])
            else:
                if self.uwb_pos_x is not None:
                    self.odom_position_list.append([x, y, 1])
                    self.uwb_position_list.append([self.uwb_pos_x, self.uwb_pos_y, 1])
        else:
            if not self.init_ekf_pos:
                world_x = np.cos(self.init_yaw) * x - np.sin(self.init_yaw) * y + self.init_x
                world_y = np.sin(self.init_yaw) * x + np.cos(self.init_yaw) * y + self.init_y
                self.last_vector[-2] = world_x
                self.last_vector[-1] = world_y
                self.state_vector[2] = world_x
                self.state_vector[3] = world_y
                self.state_vector[4] = self.init_x
                self.state_vector[5] = self.init_y
                self.state_vector[6] = self.init_yaw
                self.init_ekf_pos = True
            world_x = np.cos(self.init_yaw) * x - np.sin(self.init_yaw) * y + self.init_x
            world_y = np.sin(self.init_yaw) * x + np.cos(self.init_yaw) * y + self.init_y
            self.odom_pos_f.write("{}, {} \n".format(world_x, world_y))


if __name__ == '__main__':
    ep_robot = robot.Robot()
    # 指定机器人的 SN 号
    ep_robot.initialize(conn_type="sta", sn="3JKDH3B001W80E") # 3JKDH3B00118M5

    ep_version = ep_robot.get_version()
    print("Robot Version: {0}".format(ep_version))

    

    ep_ai_module = ep_robot.ai_module
    ep_chassis = ep_robot.chassis

    ekf_localization = EKFLocalization(ep_chassis=ep_chassis)

    ep_ai_module.sub_uwb_event(callback=ekf_localization.uwb_callback)
    ep_chassis.sub_imu(freq=50, callback=ekf_localization.imu_callback)
    ep_chassis.sub_velocity(freq=50, callback=ekf_localization.velocity_callback)
    ep_chassis.sub_position(cs=1, freq=50, callback=ekf_localization.position_callback)
    ep_chassis.sub_attitude(freq=50, callback=ekf_localization.attitude_calllback)

    ekf_localization.init_coordinate()
    

    # time.sleep(150)

    x_val = 0.5
    y_val = 0.3
    z_val = 30

    # 前进 3秒
    ep_chassis.drive_speed(x=x_val, y=0, z=0, timeout=5)
    time.sleep(2)

    # 左移 3秒
    ep_chassis.drive_speed(x=0, y=-y_val, z=0, timeout=5)
    time.sleep(2)

    # 后退 3秒
    ep_chassis.drive_speed(x=-x_val, y=0, z=0, timeout=5)
    time.sleep(2)

    # 右移 3秒
    ep_chassis.drive_speed(x=0, y=y_val, z=0, timeout=5)
    time.sleep(2)

    # 左转 3秒
    ep_chassis.drive_speed(x=0, y=0, z=-z_val, timeout=5)
    time.sleep(2)

    # 右转 3秒
    ep_chassis.drive_speed(x=0, y=0, z=z_val, timeout=5)
    time.sleep(2)

    # 停止麦轮运动
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)

    ekf_localization.close()

    ep_ai_module.unsub_uwb_event()
    ep_chassis.unsub_imu()
    ep_chassis.unsub_velocity()
    ep_chassis.unsub_position()

    ep_robot.close()