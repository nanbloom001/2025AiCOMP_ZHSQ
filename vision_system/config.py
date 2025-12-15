# config.py
import os

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
# Models are located in the 'models' subdirectory of the current folder
YOLO_MODELS_DIR = os.path.join(CURRENT_DIR, "models")

DEFAULT_CONFIG = {
    "image_topic": "/camera/color/image_raw",
    "headless": False,
}

# --- Save Settings ---
SAVE_RESULT_IMAGES = True
SAVE_RESULT_DIR = os.path.join(CURRENT_DIR, "task_save")

# --- 1. 垃圾桶类别 (Model A) ---
BIN_CLASS_NAMES = [
    'blue_close', 'blue_open', 'green_close', 'green_open',
    'grey_close', 'grey_open', 'red_close', 'red_open'
]

# --- 2. 具体垃圾类别 (Model C) ---
SPECIFIC_TRASH_NAMES = [
    "00_clamp", "01_cpu", "02_trashcan", "03_bottle", "04_can", "05_plastic",
    "06_spoon", "07_sauce", "08_orange", "09_tea", "10_sugarcane", "11_cookie",
    "12_mushroom", "13_garlic", "14_bread", "15_bones", "16_banana", "17_tomato",
    "18_nailpolish", "19_pesticide", "20_paint", "21_battery", "22_bandaid",
    "23_needle", "24_cottonswab", "25_gloves", "26_glue", "27_plasicbag",
    "28_soil", "29_toothpaste", "30_towel", "31_brush", "32_broom", "33_mirror",
    "34_comb", "35_coat"
]

# --- 3. 辅助名称映射 (用于播报) ---
TRASH_COMMON_NAMES = {
    "00_clamp": "旧夹子", "01_cpu": "废电脑", "02_trashcan": "坏垃圾桶", "03_bottle": "玻璃瓶",
    "04_can": "易拉罐", "05_plastic": "塑料瓶", "06_spoon": "勺子", "07_sauce": "番茄酱",
    "08_orange": "橙皮", "09_tea": "废弃茶叶", "10_sugarcane": "甘蔗", "11_cookie": "饼干",
    "12_mushroom": "蘑菇", "13_garlic": "大蒜", "14_bread": "面包", "15_bones": "骨头",
    "16_banana": "香蕉皮", "17_tomato": "西红柿", "18_nailpolish": "过期指甲油", "19_pesticide": "农药瓶",
    "20_paint": "油漆", "21_battery": "废旧电池", "22_bandaid": "创可贴", "23_needle": "注射器",
    "24_cottonswab": "医用棉签", "25_gloves": "医用手套", "26_glue": "胶水", "27_plasicbag": "塑料袋",
    "28_soil": "渣土", "29_toothpaste": "牙膏皮", "30_towel": "毛巾", "31_brush": "牙刷",
    "32_broom": "扫把", "33_mirror": "旧镜子", "34_comb": "木制梳子", "35_coat": "脏污衣服"
}

# --- 4. 核心分类规则映射 (36类 -> 4大类) ---
TRASH_MAPPING = {
    "00_clamp": "可回收物",        "01_cpu": "可回收物", "02_trashcan": "可回收物", "03_bottle": "可回收物",
    "04_can": "可回收物",          "05_plastic": "可回收物", "06_spoon": "可回收物", "07_sauce": "厨余垃圾",
    "08_orange": "厨余垃圾",       "09_tea": "厨余垃圾", "10_sugarcane": "厨余垃圾", "11_cookie": "厨余垃圾",
    "12_mushroom": "厨余垃圾",     "13_garlic": "厨余垃圾", "14_bread": "厨余垃圾", "15_bones": "厨余垃圾",
    "16_banana": "厨余垃圾",       "17_tomato": "厨余垃圾", "18_nailpolish": "有害垃圾", "19_pesticide": "有害垃圾",
    "20_paint": "有害垃圾",        "21_battery": "有害垃圾", "22_bandaid": "有害垃圾", "23_needle": "有害垃圾",
    "24_cottonswab": "有害垃圾",   "25_gloves": "有害垃圾", "26_glue": "其他垃圾", "27_plasicbag": "其他垃圾",
    "28_soil": "其他垃圾",         "29_toothpaste": "其他垃圾", "30_towel": "其他垃圾", "31_brush": "其他垃圾",
    "32_broom": "其他垃圾",        "33_mirror": "其他垃圾", "34_comb": "其他垃圾", "35_coat": "其他垃圾"
}

MODEL_LIST = [
    {
        "name": "best_bin",
        "path": os.path.join(YOLO_MODELS_DIR, "best_bin.onnx"),
        "version": "yolov10n",
        "task_type": "detect",
        "input_size": 640,
        "conf_thres": 0.25,
        "iou_thres": 0.45,
        "class_names": BIN_CLASS_NAMES
    },
    {
        "name": "best_trash",
        "path": os.path.join(YOLO_MODELS_DIR, "best_trash.onnx"),
        "version": "yolov10n",
        "task_type": "detect",
        "input_size": 640,
        "conf_thres": 0.25,
        "iou_thres": 0.45,
        "class_names": ["trash"]
    },
    {
        "name": "best_classify",
        "path": os.path.join(YOLO_MODELS_DIR, "best_classify.pth"),
        "task_type": "classify",
        "num_classes": len(SPECIFIC_TRASH_NAMES),
        "class_names": SPECIFIC_TRASH_NAMES
    },
    {   
        "name": "eb",
        "path": os.path.join(YOLO_MODELS_DIR, "eb.onnx"),
        "version": "yolov10n",
        "task_type": "detect",
        "input_size": 416,
        "conf_thres": 0.25,
        "iou_thres": 0.45,
        "class_names": ["0","1"]
    },
    {
        "name": "fire",
        "path": os.path.join(YOLO_MODELS_DIR, "fire.onnx"),
        "version": "yolov10n",
        "task_type": "detect",
        "input_size": 416,
        "conf_thres": 0.25,
        "iou_thres": 0.45,
        "class_names": ["fire"]
    },
    {
        "name": "lights",
        "path": os.path.join(YOLO_MODELS_DIR, "lights.onnx"),
        "version": "yolov10n",
        "task_type": "detect",
        "input_size": 416,
        "conf_thres": 0.25,
        "iou_thres": 0.45,
        "class_names": ["red", "green", "yellow"]
    },
    # {
    #     "name": "people_v2",
    #     "path": os.path.join(YOLO_MODELS_DIR, "people_v2.onnx"),
    #     "version": "yolov10s",
    #     "task_type": "detect",
    #     "input_size": 640,
    #     "conf_thres": 0.25,
    #     "iou_thres": 0.45,
    #     "class_names": ['bad0', 'bad1', 'bad2', 'bad3', 'coffee', 'cook', 'dective', 'doctor', 'engineer', 'fireman', 'gardener', 'guitar', 'it', 'office', 'painter', 'photo', 'postman', 'professor', 'rapper', 'security', 'teacher']
    # },
     {
        "name": "people_v3_s",
        "path": os.path.join(YOLO_MODELS_DIR, "people_v3_s.onnx"),
        "version": "yolov10s",
        "task_type": "detect",
        "input_size": 640,
        "conf_thres": 0.5,
        "iou_thres": 0.45,
        "class_names": ['bad0', 'bad1', 'bad2', 'bad3', 'coffee', 'cook', 'dective', 'doctor', 'engineer', 'fireman', 'gardener', 'guitar', 'it', 'office', 'painter', 'photo', 'postman', 'professor', 'rapper', 'security', 'teacher']
    },
]
