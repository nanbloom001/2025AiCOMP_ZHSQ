from .base_task import BaseTask
import rospy
import cv2
import threading
import time
import math
import tf
import numpy as np
import os
from geometry_msgs.msg import Twist
from stitching import Stitcher
import config

class FallenVehicleTask(BaseTask):
    """
    倒车检测与全景拼接任务 (Slave Side)。
    
    功能：
    1. 控制机器人旋转一周，采集多张图像。
    2. 使用 Stitcher 算法将采集的图像拼接成全景图。
    3. 在全景图上运行 YOLO 模型检测倒地车辆 (fallen_vehicle)。
    4. 返回检测结果。
    """
    def __init__(self, model_loader, data_manager):
        super().__init__(model_loader, data_manager)
        # 加载 YOLO 模型 (模型名称 'eb' 对应倒车检测)
        self.model = self.model_loader.load_yolo_model("eb")
        
        # 机器人运动控制
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.tf_listener = tf.TransformListener()
        
        # 状态变量
        self.latest_image = None
        self.lock = threading.Lock()
        self.is_processing = False
        self.result_data = None
        self.worker_thread = None
        self.stop_flag = False
        self.sequence_finished = False

    def start(self, seq_id):
        """启动任务：重置状态并开始后台处理线程。"""
        super().start(seq_id)
        self.sequence_finished = False
        self.is_processing = False
        self.result_data = None
        self.stop_flag = False

    def stop(self):
        super().stop()
        self.stop_flag = True
        # Do not join thread here to avoid blocking the main thread if stop is called from it
        # The thread will exit when it sees stop_flag
        self.is_processing = False
        self.result_data = None

    def process(self, image):
        if not self.active: 
            return image, None
        
        if self.sequence_finished:
            return image, None
        
        # Update latest image
        with self.lock:
            self.latest_image = image.copy()
            
        # Return result if ready
        if self.result_data:
            res_img, res_msg = self.result_data
            self.result_data = None 
            self.is_processing = False 
            self.sequence_finished = True
            return res_img, res_msg

        # Start sequence if not running
        if not self.is_processing:
            self.is_processing = True
            self.stop_flag = False
            self.worker_thread = threading.Thread(target=self.run_sequence)
            self.worker_thread.start()
            
        return image, None

    def run_sequence(self):
        rospy.loginfo("FallenVehicleTask: Starting sequence...")
        captured_images = []
        
        # 1. Rotate Right 105 deg (-105)
        if not self.rotate_robot(-105, 1.0): 
            self.is_processing = False
            return
        time.sleep(0.5) # Stabilize
        img1 = self.get_image()
        if img1 is None: 
            self.is_processing = False
            return
        captured_images.append(img1)
        rospy.loginfo("Captured Image 1")
        
        # 2. Rotate Left 15 deg (+15)
        if not self.rotate_robot(18, 1.0): 
            self.is_processing = False
            return
        time.sleep(0.5)
        img2 = self.get_image()
        if img2 is None: 
            self.is_processing = False
            return
        captured_images.append(img2)
        rospy.loginfo("Captured Image 2")

        # 3. Rotate Left 15 deg (+15)
        if not self.rotate_robot(18, 1.0): 
            self.is_processing = False
            return
        time.sleep(0.5)
        img3 = self.get_image()
        if img3 is None: 
            self.is_processing = False
            return
        captured_images.append(img3)
        rospy.loginfo("Captured Image 3")
        
        # Stitching
        rospy.loginfo("Stitching images...")
        stitcher = Stitcher(
            detector="orb",
            confidence_threshold=0.1,
            range_width=1,
            warper_type="cylindrical",
            blender_type="feather",
            compensator="gain_blocks",
            crop=True,
            try_use_gpu=False,
            medium_megapix=0.4,
            final_megapix=-1
        )
        
        try:
            panorama = stitcher.stitch(captured_images)
        except Exception as e:
            rospy.logerr(f"Stitching failed: {e}")
            self.is_processing = False
            return

        if panorama is None:
            rospy.logerr("Stitching returned None")
            self.is_processing = False
            return
            
        rospy.loginfo("Stitching done. Running inference...")
        
        # Save Stitched Image
        if config.SAVE_RESULT_IMAGES:
            try:
                if not os.path.exists(config.SAVE_RESULT_DIR):
                    os.makedirs(config.SAVE_RESULT_DIR)
                timestamp = int(time.time() * 1000)
                filename = f"task_fallen_vehicle_stitch_{timestamp}.jpg"
                filepath = os.path.join(config.SAVE_RESULT_DIR, filename)
                cv2.imwrite(filepath, panorama)
                rospy.loginfo(f"Saved stitched image to {filepath}")
            except Exception as e:
                rospy.logwarn(f"Failed to save stitched image: {e}")

        # Inference
        res_img, _, (boxes, scores, class_ids) = self.model.infer(panorama)
        
        msg = None
        if len(boxes) > 0:
            msg = "done_fallen_vehicle detected"
            rospy.loginfo("Fallen vehicle detected!")
        else:
            rospy.loginfo("No fallen vehicle detected.")
            
        self.result_data = (res_img, msg)

    def get_image(self):
        with self.lock:
            if self.latest_image is None:
                return None
            return self.latest_image.copy()

    def get_yaw(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            _, _, yaw = tf.transformations.euler_from_quaternion(rot)
            return yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                _, _, yaw = tf.transformations.euler_from_quaternion(rot)
                return yaw
            except Exception as e:
                rospy.logwarn(f"TF Error: {e}")
                return None

    def rotate_robot(self, angle_deg, speed):
        if self.stop_flag: return False
        
        target_rad = math.radians(angle_deg)
        start_yaw = self.get_yaw()
        
        if start_yaw is None:
            rospy.logerr("Cannot get initial yaw")
            return False

        twist = Twist()
        twist.angular.z = speed if angle_deg > 0 else -speed
        
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.stop_flag:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
            
            current_yaw = self.get_yaw()
            if current_yaw is None: continue
            
            delta_yaw = current_yaw - start_yaw
            while delta_yaw > math.pi: delta_yaw -= 2*math.pi
            while delta_yaw < -math.pi: delta_yaw += 2*math.pi
            
            if abs(delta_yaw) >= abs(target_rad):
                break
        
        # Stop
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(0.5)
        
        # Print actual rotation
        end_yaw = self.get_yaw()
        if end_yaw is not None:
            real_delta = end_yaw - start_yaw
            while real_delta > math.pi: real_delta -= 2*math.pi
            while real_delta < -math.pi: real_delta += 2*math.pi
            rospy.loginfo(f"Rotation finished. Target: {angle_deg} deg, Actual: {math.degrees(real_delta):.2f} deg")
            
        return True
