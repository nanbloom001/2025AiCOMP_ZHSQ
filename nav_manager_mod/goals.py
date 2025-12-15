"""默认导航目标点列表。"""

from typing import List
from .config import Goal, Task


# 仅保留活跃的目标点；需要时在此处添加新点。
# 格式: Goal(x, y, yaw, [Task("task_type", seq_id), ...])
# 可用任务: traffic_light, doll, illegal_parking, fire, garbage, fallen_vehicle, plate
GOALS: List[Goal] = [
    # Goal(1.069, -0.061, -0.038, [Task("traffic_light", 1)]),  # 0
    # Goal(3.279, -0.082, 0.262, [Task("none", 0)]),   # 1
    # Goal(3.386, -0.151, 1.4979, [Task("none", 0)]),  # 2
    # Goal(3.442, 0.989, 1.4617, [Task("none", 0)]),   # 3
    # Goal(3.451, 0.967, 3.109, [Task("none", 0)]),    # 4
    # Goal(2.695, 0.968, 3.081, [Task("none", 0)]),    # 5
    # Goal(2.760, 1.028, 1.6967, [Task("none", 0)]),   # 6
    # Goal(2.789, 1.021, -1.705, [Task("none", 0)]),   # 7
    # Goal(2.783, 0.999, 3.086, [Task("none", 0)]),    # 8
    # Goal(2.007, 1.034, 3.094, [Task("fire", 0)]),    # 9
    # Goal(1.852, 1.059, 1.428, [Task("none", 0)]),    # 10
    # Goal(1.958, 1.834, 1.415, [Task("none", 0)]),    # 11
    # Goal(2.142, 3.254, 1.432, [Task("none", 0)]),    # 12
    # Goal(2.081, 3.292, -3.074, [Task("none", 0)]),   # 13
    # Goal(1.035, 3.318, 3.117, [Task("plate", 0)]),    # 14
    # Goal(0.828, 2.734, 3.111, [Task("plate", 0)]),    # 15
    # Goal(0.740, 2.122, 3.107, [Task("plate", 0)]),    # 16
    # Goal(0.673, 2.088, -1.645, [Task("none", 0)]),   # 17
    # Goal(0.520, -0.050, 0.135, [Task("none", 0)]),   # 18
    # Goal(0.066, -0.061, 0.041, [Task("none", 0)]),   # 19

    ###wlynew脚本里的点###
Goal(1.007, -0.0313, -0.0269, [Task("traffic_light", 1)]),           # 1
Goal(2.4271, -0.0878, -0.0327, [Task("none", 0)]),          # 2
Goal(2.5021, -0.0609, 1.5036, [Task("doll", 1)]),           # 3 - peoA1
Goal(2.5052, -0.0683, -0.023, [Task("none", 0)]),           # 4
Goal(3.1925, -0.1134, -0.0705, [Task("none", 0)]),          # 5
Goal(3.3601, -0.0975, 1.5568, [Task("none", 0)]),           # 6
Goal(3.3723, 0.3622, 1.4943, [Task("none", 0)]),            # 7
Goal(3.368, 0.4439, -3.1309, [Task("doll", 2)]),            # 8 - peoA2
Goal(3.3441, 0.4354, 1.4854, [Task("none", 0)]),            # 9
Goal(3.3543, 1.0162, 1.5449, [Task("none", 0)]),            # 10 - sign
Goal(3.3497, 1.0412, 3.0944, [Task("none", 0)]),            # 11
Goal(2.5216, 1.0237, -3.1118, [Task("none", 0)]),           # 12
Goal(2.5265, 1.0277, -1.766, [Task("doll", 3)]),            # 13 - peoA3
Goal(2.5296, 1.0299, 1.4427, [Task("doll", 4)]),            # 14 - peoB1
Goal(2.5499, 1.0173, 3.0361, [Task("none", 0)]),            # 15
Goal(1.9494, 1.0248, 3.1086, [Task("fire", 1)]),            # 16 - building1
Goal(1.8582, 1.0409, 1.4775, [Task("none", 0)]),            # 17
Goal(1.891, 1.6092, 1.5274, [Task("none", 0)]),             # 18
Goal(1.9052, 1.8039, -0.2063, [Task("doll", 5)]),           # 19 - peoB2


Goal(1.979, 1.869, 3.042, [Task("fire", 2)]),               # 20 - building2


Goal(1.8859, 1.7875, 1.484, [Task("traffic_light", 2)]),    # 20 - lights
Goal(1.9451, 3.2437, 1.5285, [Task("none", 0)]),            # 21
Goal(1.9425, 3.2753, 3.094, [Task("none", 0)]),             # 22
Goal(0.7866, 3.2978, 3.1343, [Task("plate", 1)]),            # 23 - car1
Goal(0.7836, 3.2986, -1.586, [Task("none", 0)]),            # 24
Goal(0.7706, 2.8457, -1.613, [Task("none", 0)]),            # 25
Goal(0.7684, 2.7052, -3.1104, [Task("plate", 2)]),           # 26 - car2
Goal(0.7772, 2.7176, -1.6326, [Task("none", 0)]),           # 27
Goal(0.78, 2.6127, -1.5742, [Task("none", 0)]),             # 28
Goal(0.7963, 2.5053, 0.0159, [Task("garbage", 0)]),            # 29 - bin
Goal(0.7877, 2.5066, -1.6758, [Task("none", 0)]),           # 30
Goal(0.7645, 2.1732, -1.6683, [Task("none", 0)]),           # 31
Goal(0.7594, 2.0563, -3.115, [Task("plate", 3)]),            # 32 - car3
Goal(0.7868, 2.0749, -1.6737, [Task("none", 0)]),           # 33
Goal(0.7986, 1.455, 0.0373, [Task("fire", 3)]),             # 34 - building3
Goal(0.8201, 1.4647, -1.5834, [Task("none", 0)]),           # 35
Goal(0.7978, 1.2881, -1.6691, [Task("none", 0)]),           # 36
Goal(0.805, 1.3, -3.0572, [Task("none", 0)]),               # 37 - motor1
Goal(0.8148, 1.3056, -1.6782, [Task("none", 0)]),           # 38
Goal(0.8085, 0.7411, -1.591, [Task("none", 0)]),            # 39
Goal(0.7932, 0.7422, 3.1065, [Task("none", 0)]),            # 40 - motor2
Goal(0.8055, 0.7527, -1.6335, [Task("none", 0)]),           # 41
Goal(0.8228, -0.0583, -1.5503, [Task("none", 0)]),          # 42
Goal(0.821, -0.0609, 0.0368, [Task("none", 0)]),            # 43
Goal(0.1694, -0.1258, 0.1259, [Task("none", 0)]),           # 44
]


def load_goals() -> List[Goal]:
    """加载并返回目标点列表。"""
    return list(GOALS)
