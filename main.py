"""
@author: 15201622364
"""

from controller import Camera, RangeFinder, Supervisor
from algorithm.rover_controller import rover_controller
from algorithm import slam
import numpy as np
import cv2

if __name__ == "__main__":
    sup = Supervisor()
    robot = sup.getFromDef('Rover')  # 连接巡视器

    motor_lf = sup.getDevice('lf_motor')  # 连接左前轮驱动
    motor_lb = sup.getDevice('lb_motor')  # 连接左后轮驱动
    motor_rf = sup.getDevice('rf_motor')  # 连接右前轮驱动
    motor_rb = sup.getDevice('rb_motor')  # 连接右后轮驱动

    motor_lf.setPosition(float('inf'))  # 左前轮初始化
    motor_rf.setPosition(float('inf'))
    motor_lb.setPosition(float('inf'))
    motor_rb.setPosition(float('inf'))
    motor_lf.setVelocity(0)  # 给定左前轮转速，不超过1
    motor_rf.setVelocity(0)
    motor_lb.setVelocity(0)
    motor_rb.setVelocity(0)

    cam_f = Camera('cam_f')  # 连接前相机-RGB
    cam_b = Camera('cam_b')
    cam_f.enable(30)  # 初始化
    cam_b.enable(30)

    dep_f = RangeFinder('depth_b')  # 连接前相机D
    dep_b = RangeFinder('depth_f')
    dep_f.enable(30)  # 初始化
    dep_b.enable(30)

    """先验信息"""
    moon_map = np.load('map.npy')
    pos_A = [10, -15]  # 初始点的x\y坐标
    area_B = [-15, 15, 10, 10]  # 目标区域的中心点x\y坐标、x\y宽度

    con = rover_controller(moon_map, pos_A, area_B)  # 决策程序实例化
    world_time = 0
    done = False

    while not done:

        """获取RGBD数据"""
        rgb_image_f = cam_f.getImageArray()  # 前相机获取RGB图像
        rgb_image_b = cam_b.getImageArray()
        d_image_f = dep_f.getRangeImageArray()  # 前相机获取D图像
        d_image_b = dep_b.getRangeImageArray()

        """jinz"""
        slam.get_image_from_camera(cam_f)

        cv2.waitKey(100)

        """决策主程序"""
        motor_velocity, done, target_pos = con.step(rgb_image_f, rgb_image_b, d_image_f, d_image_b, world_time)

        """巡视器轮速控制"""
        if motor_velocity:
            motor_lf.setVelocity(motor_velocity[0])  # 给定左前轮转速，-1~1
            motor_rf.setVelocity(motor_velocity[1])
            motor_lb.setVelocity(motor_velocity[2])
            motor_rb.setVelocity(motor_velocity[3])

        """仿真环境运行"""
        for _ in range(30):
            sup.step(32)
            world_time += 32  # 32ms
