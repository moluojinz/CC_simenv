"""
@author: 15201622364
"""
import keyboard

from controller import Camera, RangeFinder, Supervisor
from algorithm.rover_controller import rover_controller
from algorithm import slam
import numpy as np
import cv2
import open3d as o3d

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

    dep_f = RangeFinder('depth_f')  # 连接前相机D
    dep_b = RangeFinder('depth_b')
    dep_f.enable(30)  # 初始化
    dep_b.enable(30)

    """先验信息"""
    moon_map = np.load('map.npy')
    pos_A = [10, -15]  # 初始点的x\y坐标
    area_B = [-15, 15, 10, 10]  # 目标区域的中心点x\y坐标、x\y宽度

    con = rover_controller(moon_map, pos_A, area_B)  # 决策程序实例化
    world_time = 0
    done = False

    """jinz"""
    point_cloud = o3d.geometry.PointCloud()
    camera_position = np.array([0, 0, 0])  # 替换为实际的初始化坐标
    camera_position = np.reshape(camera_position, (3, 1))
    while not done:

        """获取RGBD数据"""
        rgb_image_f = cam_f.getImageArray()  # 前相机获取RGB图像
        rgb_image_b = cam_b.getImageArray()
        d_image_f = dep_f.getRangeImageArray()  # 前相机获取D图像
        d_image_b = dep_b.getRangeImageArray()

        """jinz"""
        relative_position = slam.slam_pose(cam_f)
        if relative_position is not None:
            # 将相对位置转换为NumPy数组
            relative_position = (np.asarray(relative_position))
            print("relative_position:")
            print(relative_position)  # 计算相机位置相对于初始化坐标的坐标
            camera_position = camera_position + relative_position
            print("Camera position relative to initial coordinate:")
            print(camera_position)
            # 将相机位置作为点添加到点云中
            point_cloud.points.append(camera_position)
            # o3d.visualization.draw_geometries([point_cloud])
        else:
            print("Invalid relative position")

        key=cv2.waitKey(10)

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
        if keyboard.is_pressed(17):
            break

        if key & 0xFF == ord('q'):
            break
    o3d.visualization.draw_geometries([point_cloud])
