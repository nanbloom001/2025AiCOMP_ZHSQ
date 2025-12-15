import rospy
import cv2

class BaseTask:
    def __init__(self, model_loader, data_manager):
        self.model_loader = model_loader
        self.data_manager = data_manager
        self.active = False
        self.seq_id = 0

    def start(self, seq_id):
        self.active = True
        self.seq_id = seq_id
        rospy.loginfo(f"Task {self.__class__.__name__} started with seq_id {seq_id}")

    def stop(self):
        self.active = False
        rospy.loginfo(f"Task {self.__class__.__name__} stopped")

    def process(self, image):
        """
        Process the image and return (result_image, result_data)
        result_data: String to be published to /visual_task_done, or None if not done/no result
        """
        return image, None
