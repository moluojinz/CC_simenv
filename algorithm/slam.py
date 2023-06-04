import numpy as np
import cv2
import open3d as o3d


def o3d_test():
    # 生成随机点云
    num_points = 1000
    points = np.random.rand(num_points, 3)  # 生成在[0, 1)范围内的随机坐标

    # 创建Open3D中的点云对象
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    # 可视化点云
    o3d.visualization.draw_geometries([point_cloud])

    pass


def slam_o3d(dep_data, dep_width, dep_height):
    # print(dep_data)
    print(dep_height)
    print(dep_width)
    dep_data = np.array(dep_data, dtype=np.float32)
    dep_data = dep_data.T
    # rows, cols = dep_data.shape
    # print(rows)
    # print(cols)
    points = []
    for y in range(dep_data.shape[0]):
        for x in range(dep_data.shape[1]):
            if not np.isnan(dep_data[y, x]) and not np.isinf(dep_data[y, x]) and dep_data[y, x] > 0:  # 非零距离值表示有效点
                point = [dep_data[y, x] * x, dep_data[y, x] * y, dep_data[y, x]]
                points.append(point)
    # # 将点云数据添加到Open3D点云对象中
    points_cloud = o3d.geometry.PointCloud()
    points_cloud.points = o3d.utility.Vector3dVector(points)
    # # 可视化点云
    o3d.visualization.draw_geometries([points_cloud])
    pass


def get_depth_data(rangefinder_data, width, height):
    # image = np.zeros((height, width, 3), dtype=np.uint16)

    webots_depth_array = np.asarray(rangefinder_data, dtype=np.float)
    webots_depth_array = cv2.rotate(webots_depth_array, cv2.ROTATE_90_CLOCKWISE)  # 用于旋转，看你webot的视角了
    webots_depth_array = cv2.flip(webots_depth_array, 1)  # 最后又进行了翻转
    distance = webots_depth_array[100, 300]
    print(distance)

    color = (0, 255, 0)  # 标记颜色为绿色 (BGR)
    marker_type = cv2.MARKER_CROSS
    marker_size = 10
    thickness = 2
    cv2.drawMarker(webots_depth_array, (300, 100), color, marker_type, marker_size, thickness)

    # for x in range(webots_depth_array.shape[0]):
    #     for y in range(webots_depth_array.shape[1]):
    #         distance = webots_depth_array[x, y]
    #         if  distance > 0:  # 非零距离值表示有效点
    #             image[x, y] = (255, 255, 255)  # 将有效点置为白色

    # 显示图像

    cv2.imshow('dep', webots_depth_array)
    # cv2.imshow("Rangefinder Data", image)
    pass


def get_cammatrix(cam):
    image_width = cam.getWidth()
    image_height = cam.getHeight()
    fov = cam.getFov()
    focal_length = (image_width / 2) / np.tan(fov / 2)
    principal_point_x = image_width / 2
    principal_point_y = image_height / 2
    cam_matrix = np.array([[focal_length, 0, principal_point_x],
                           [0, focal_length, principal_point_y],
                           [0, 0, 1]])
    # print(cam_matrix.shape[0], cam_matrix.shape[1], cam_matrix.ndim)

    return cam_matrix


# 创建ORB特征提取器和特征匹配器
orb = cv2.ORB_create()
matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# 初始化上一帧的图像、特征点和描述符
previous_image = None
previous_keypoints = None
previous_descriptors = None
T = None  # 初始化移动向量为 None


def slam_pose(cam):
    global previous_image, previous_keypoints, previous_descriptors, T

    image = cam.getImage()
    image = np.frombuffer(image, np.uint8).reshape((cam.getHeight(), cam.getWidth(), 4))

    # 转换为灰度图像
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 相机内参
    K = get_cammatrix(cam)

    # 提取特征点和描述符
    keypoints, descriptors = orb.detectAndCompute(gray, None)

    if previous_image is not None and previous_keypoints is not None and previous_descriptors is not None:
        # 匹配特征点
        matches = matcher.match(previous_descriptors, descriptors)

        # 提取匹配点的坐标
        src_pts = np.float32([previous_keypoints[m.queryIdx].pt for m in matches])
        """.reshape((-1, 1, 2))"""
        dst_pts = np.float32([keypoints[m.trainIdx].pt for m in matches])
        """.reshape((-1, 1, 2))"""

        # 估计基础矩阵（或本质矩阵）
        F, _ = cv2.findFundamentalMat(src_pts, dst_pts, cv2.FM_8POINT, ransacReprojThreshold=None, confidence=None,
                                      maxIters=None)
        print("fundamental_matrix is:\n", F)
        # 从基础矩阵计算本质矩阵
        principal_point = (K[0, 2], K[0, 2])  # 光心, TUM dataset 标定值
        focal_length = K[1, 1]  # 焦距, TUM dataset 标定值
        E, _ = cv2.findEssentialMat(src_pts, dst_pts, focal_length, principal_point, cv2.RANSAC)
        print("essential_matrix is:\n", E)

        # 从本质矩阵恢复相机位姿
        _, R, t, _ = cv2.recoverPose(E, src_pts, dst_pts, K, mask=None)
        print("R is:\n", R)
        print("t is:\n", t)
        # 计算相机的移动向量
        T = -np.dot(R.T, t)
    # 更新上一帧的图像、特征点和描述符
    previous_image = image
    previous_keypoints = keypoints
    previous_descriptors = descriptors

    return T


def cloud_world_test():
    # 创建自定义的几何图形表示坐标轴
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

    # 创建可视化窗口并添加坐标轴
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(axis)

    # 设置相机参数和视角
    view_control = vis.get_view_control()
    view_control.set_lookat([0, 0, 0])  # 设置视点
    view_control.set_up([0, -1, 0])  # 设置相机的上方向
    view_control.set_front([-1, 0, 0])  # 设置相机的前方向

    # 渲染可视化窗口
    vis.run()
