"""
@author: 15201622364
"""

class rover_controller(object):
    def __init__(self, moon_map, pos_A, area_B):
        self.map = moon_map
        self.pos_A = pos_A
        self.area_B = area_B
        
    def step(self, rgb_image_f, rgb_image_b, d_image_f, d_image_b, world_time):
        """
        功能：进行一步决策推理，请根据输入自行判断是否、如何进行决策推理
        输入：
            rgb_image_f: 前相机RGB数据，分辨率1280*720
            rgb_image_b: 后相机RGB数据，分辨率1280*720
            d_image_f: 前相机D数据，与RGB对齐，分辨率1280*720
            d_image_b: 后相机D数据，与RGB对齐，分辨率1280*720
            world_time: 仿真器的时间，单位ms
        输出：
            motor_velocity: 四轮转速[左前，右前，左后，右后]，范围[-1,1]，如不进行推理则输出[]
            done: 任务是否结束，结束为True，不结束False，自行判断何时结束
            target_pos: 发现的目标的二维坐标，维度n*2的list，n为发现的目标个数
                        当done为True时输出该list，当done为False时输出[]
        """
        motor_velocity = [1, 1, 1, 1]
        done = False
        target_pos = [[1, 2],
                      [3, 4],
                      [5, 6]]
        return motor_velocity, done, target_pos


