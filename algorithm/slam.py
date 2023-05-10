import numpy as np
import cv2

def get_image_from_camera(webots_camera):
    img = webots_camera.getImageArray()
    img_array = np.asarray(img, dtype=np.uint8)
    img_array = cv2.cvtColor(img_array, cv2.COLOR_BGRA2RGB)
    img = cv2.rotate(img_array, cv2.ROTATE_90_CLOCKWISE)  # 用于旋转，看你webot的视角了
    img = cv2.flip(img, 1)  # 最后又进行了翻转
    sift = cv2.SIFT_create()
    (kp1, des1) = sift.detectAndCompute(img, None)
    # (kp2, des2) = sift.detectAndCompute(last_sift, None)  # 上一次sift图像
    sift_img = cv2.drawKeypoints(img, kp1, img, color=(0, 0, 255))

    # bf = cv2.BFMatcher()  # knn匹配
    # matches = bf.knnMatch(des1, des2, k=2)
    # print('用于 原图和旋转图 图像匹配的所有特征点数目：', len(matches))
    # """
    #     调整ratio
    # ratio=0.4：对于准确度要求高的匹配；
    # ratio=0.6：对于匹配点数目要求比较多的匹配；
    # ratio=0.5：一般情况下。
    # """
    # ratio = 0.4
    # good = []
    # for m, n in matches:
    #     # 如果最接近和次接近的比值大于一个既定的值，那么我们保留这个最接近的值，认为它和其匹配的点为good_match
    #     if m.distance < ratio * n.distance:
    #         good.append([m])
    # match_result = cv2.drawMatchesKnn(sift_img, kp1, last_sift, kp2, good, None, flags=2)
    # cv2.imshow("original_lena and lena_rot45 feature matching result", match_result)

    cv2.imshow("f", sift_img)
    return sift_img



