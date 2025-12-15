#!/usr/bin/env python
# coding:utf-8
import cv2
import numpy as np


class TrafficLightYCrCbDetector:
    def __init__(self):
        """初始化检测器"""
        # 跟踪状态（移除黄色）
        self.is_first_detected = {'red': True, 'green': True}
        self.last_track_boxes = {'red': [], 'green': []}
        self.last_track_num = {'red': 0, 'green': 0}

        # 亮度调整参数
        self.brightness_gain = 0.3
        self.brightness_bias = (1 - self.brightness_gain) * 120

        # YCrCb阈值（扩大检测范围）
        self.ycrcb_ranges = {
            'red': {'cr_min': 140, 'cr_max': 255},  # 红色Cr范围（扩大）
            'green': {'cr_min': 90, 'cr_max': 107}  # 绿色Cr范围（扩大）
        }

        # 形态学核大小
        self.dilate_kernel_size = (15, 15)
        self.erode_kernel_size = (1, 1)

        # 最小面积阈值（过滤噪点）
        self.min_area = 300  # 降低最小面积，检测更多候选区域

        # 可选：是否启用额外的亮度/饱和度过滤
        self.use_additional_filters = True

    def is_intersected(self, rect1, rect2):
        """判断两个矩形区域是否相交"""
        min_x = max(rect1[0], rect2[0])
        min_y = max(rect1[1], rect2[1])
        max_x = min(rect1[0] + rect1[2], rect2[0] + rect2[2])
        max_y = min(rect1[1] + rect1[3], rect2[1] + rect2[3])
        return min_x < max_x and min_y < max_y

    def crop_upper_half(self, image):
        """裁剪图像上半部分"""
        height = image.shape[0]
        return image[0:height // 2, :]

    def adjust_brightness(self, image):
        """调整图像亮度"""
        adjusted = image * self.brightness_gain + self.brightness_bias
        adjusted = np.clip(adjusted, 0, 255).astype(np.uint8)
        return adjusted

    def create_color_mask(self, ycrcb_image, original_image, color):
        """基于YCrCb颜色空间创建颜色掩码（扩大检测范围）"""
        y, cr, cb = cv2.split(ycrcb_image)

        mask = np.zeros((ycrcb_image.shape[0], ycrcb_image.shape[1]), dtype=np.uint8)

        # 根据颜色类型设置不同的阈值
        cr_min = self.ycrcb_ranges[color]['cr_min']
        cr_max = self.ycrcb_ranges[color]['cr_max']

        # 基础Cr通道过滤
        mask[(cr >= cr_min) & (cr <= cr_max)] = 255

        # 可选：添加额外的过滤条件，减少噪声
        if self.use_additional_filters:
            # 转换到HSV以获取饱和度信息
            hsv = cv2.cvtColor(original_image, cv2.COLOR_BGR2HSV)
            _, s, v = cv2.split(hsv)

            # 添加亮度约束（V > 50，避免太暗的区域）
            brightness_mask = v > 50

            if color == 'red':
                # 红色：高饱和度（S > 80）
                saturation_mask = s > 80
            else:  # green
                # 绿色：中等饱和度（S > 60）
                saturation_mask = s > 60

            # 组合所有约束
            mask = mask & brightness_mask.astype(np.uint8) * 255
            mask = mask & saturation_mask.astype(np.uint8) * 255

        # 形态学操作
        kernel_dilate = np.ones(self.dilate_kernel_size, np.uint8)
        kernel_erode = np.ones(self.erode_kernel_size, np.uint8)
        mask = cv2.dilate(mask, kernel_dilate)
        mask = cv2.erode(mask, kernel_erode)

        return mask

    def process_mask(self, mask, color):
        """处理掩码，检测轮廓并计算面积"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        total_area = 0
        valid_boxes = []

        if contours:
            track_boxes = []

            # 计算每个轮廓的边界框
            for contour in contours:
                area = cv2.contourArea(contour)

                # 过滤掉面积过小的轮廓
                if area < self.min_area:
                    continue

                # 获取凸包和边界矩形
                hull = cv2.convexHull(contour)
                x, y, w, h = cv2.boundingRect(hull)

                # 过滤掉长宽比不合理的区域（红绿灯通常接近圆形）
                aspect_ratio = float(w) / h if h > 0 else 0
                if 0.5 < aspect_ratio < 2.0:  # 允许一定范围的长宽比
                    track_boxes.append((x, y, w, h, area))

            if self.is_first_detected[color]:
                # 首次检测，存储所有有效的跟踪框
                valid_boxes = track_boxes
                self.last_track_boxes[color] = track_boxes
                self.last_track_num[color] = len(track_boxes)
                self.is_first_detected[color] = False
            else:
                # 与前一帧进行匹配
                for box in track_boxes:
                    for last_box in self.last_track_boxes[color][:self.last_track_num[color]]:
                        if self.is_intersected(box[:4], last_box[:4]):
                            valid_boxes.append(box)
                            break

                # 更新跟踪框
                self.last_track_boxes[color] = track_boxes
                self.last_track_num[color] = len(track_boxes)

            # 计算总面积
            for box in valid_boxes:
                total_area += box[4]  # 使用实际轮廓面积

        else:
            # 没有检测到轮廓，重置状态
            self.is_first_detected[color] = True

        return total_area, valid_boxes

    def detect(self, image):
        """主检测函数（扩大检测范围）"""
        # 裁剪上半部分
        cropped = self.crop_upper_half(image)

        # 调整亮度
        adjusted = self.adjust_brightness(cropped)

        # 转换到YCrCb色彩空间
        ycrcb = cv2.cvtColor(adjusted, cv2.COLOR_BGR2YCrCb)

        # 为每种颜色创建掩码并处理（只检测红色和绿色）
        results = {}
        masks = {}

        for color in ['red', 'green']:
            mask = self.create_color_mask(ycrcb, adjusted, color)
            area, boxes = self.process_mask(mask, color)

            results[color] = {
                'area': area,
                'boxes': boxes,
                'detected': area > 0
            }
            masks[color] = mask

        # 确定当前亮灯状态（面积最大的颜色）
        max_area_color = max(results.items(), key=lambda x: x[1]['area'])
        active_light = max_area_color[0] if max_area_color[1]['area'] > 0 else None

        return results, active_light, cropped, adjusted, masks

    def get_traffic_light_state(self, active_light):
        """判断交通灯状态（移除黄灯）"""
        if not active_light:
            return "未检测到信号灯"

        state_map = {
            'red': "红灯 - 停止",
            'green': "绿灯 - 通行"
        }

        return state_map.get(active_light, "未知状态")

    def draw_results(self, image, results, active_light):
        """在图像上绘制检测结果（移除黄色）"""
        result_image = image.copy()
        h, w = image.shape[:2]

        # 绘制裁剪区域分界线
        cv2.line(result_image, (0, h // 2), (w, h // 2), (255, 0, 0), 2)
        cv2.putText(result_image, "Detection Area (Upper Half)",
                    (10, h // 2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # 颜色映射（移除黄色）
        color_map = {
            'red': (0, 0, 255),
            'green': (0, 255, 0)
        }

        # 在图像上绘制检测框
        for color, info in results.items():
            if info['detected']:
                bgr_color = color_map[color]
                for box in info['boxes']:
                    x, y, w, h = box[:4]
                    # 绘制矩形框
                    cv2.rectangle(result_image, (x, y), (x + w, y + h), bgr_color, 2)
                    # 标注颜色
                    cv2.putText(result_image, color.upper(), (x, y - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr_color, 2)

        # 显示检测信息
        y_offset = 30
        for color in ['red', 'green']:
            info = results[color]
            bgr_color = color_map[color]

            status = "ON" if info['detected'] else "OFF"
            text = f"{color.upper()}: {status} (面积: {info['area']})"
            text_color = bgr_color if info['detected'] else (128, 128, 128)

            cv2.putText(result_image, text, (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)
            y_offset += 35

        # 显示交通灯状态
        state = self.get_traffic_light_state(active_light)
        state_color = color_map.get(active_light, (255, 255, 255))

        cv2.rectangle(result_image, (5, h - 60), (w - 5, h - 10), (0, 0, 0), -1)
        cv2.putText(result_image, f"Status: {state}", (15, h - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, state_color, 2)

        return result_image


def main():
    """主函数"""
    # 初始化检测器
    detector = TrafficLightYCrCbDetector()

    # 从图片文件检测
    image_path = 'community_data/light/raw_image/41.jpg'
    image = cv2.imread(image_path)

    if image is None:
        print("无法加载图片！")
        return

    # 执行检测
    results, active_light, cropped, adjusted, masks = detector.detect(image)

    # 获取交通灯状态
    state = detector.get_traffic_light_state(active_light)

    # 绘制结果
    result_image = detector.draw_results(image, results, active_light)

    # 显示结果
    cv2.imshow('Original', image)
    cv2.imshow('Cropped Upper Half', cropped)
    cv2.imshow('Brightness Adjusted', adjusted)
    cv2.imshow('Detection Result', result_image)

    # 显示各颜色掩码（移除黄色）
    cv2.imshow('Red Mask', masks['red'])
    cv2.imshow('Green Mask', masks['green'])

    # 打印检测详情
    print("=" * 60)
    print(f"检测区域: 图像上半部分")
    print(f"YCrCb阈值配置:")
    print(f"  红色 Cr范围: {detector.ycrcb_ranges['red']['cr_min']} - {detector.ycrcb_ranges['red']['cr_max']}")
    print(f"  绿色 Cr范围: {detector.ycrcb_ranges['green']['cr_min']} - {detector.ycrcb_ranges['green']['cr_max']}")
    print(f"  额外过滤: {'启用' if detector.use_additional_filters else '禁用'}")
    print("-" * 60)
    print(f"交通灯状态: {state}")
    print(f"当前亮灯: {active_light.upper() if active_light else '无'}")
    print("=" * 60)
    print("详细信息:")
    for color, info in results.items():
        print(f"  {color.upper()}:")
        print(f"    检测到: {info['detected']}")
        print(f"    总面积: {info['area']}")
        print(f"    检测框数量: {len(info['boxes'])}")

    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main_video():
    """摄像头实时检测（移除黄色）"""
    detector = TrafficLightYCrCbDetector()
    cap = cv2.VideoCapture(0)

    print("按 'q' 退出, 'm' 显示/隐藏掩码")
    show_masks = False

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 执行检测
        results, active_light, cropped, adjusted, masks = detector.detect(frame)

        # 绘制结果
        result_image = detector.draw_results(frame, results, active_light)

        cv2.imshow('Traffic Light Detection', result_image)

        # 可选：显示掩码
        if show_masks:
            cv2.imshow('Red Mask', masks['red'])
            cv2.imshow('Green Mask', masks['green'])

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('m'):
            show_masks = not show_masks
            if not show_masks:
                cv2.destroyWindow('Red Mask')
                cv2.destroyWindow('Green Mask')

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # 图片检测
    # main()

    # 视频检测（取消注释使用）
    main_video()