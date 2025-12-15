import rospy
import cv2

class BaseTask:
    """
    所有视觉任务的基类。
    
    定义了任务的基本生命周期接口：启动 (start)、停止 (stop) 和处理 (process)。
    子类应当重写这些方法以实现具体的业务逻辑。
    """
    def __init__(self, model_loader, data_manager):
        """
        初始化任务。
        
        Args:
            model_loader: 模型加载器实例，用于获取共享的推理模型。
            data_manager: 数据管理器实例，用于保存结果或加载配置。
        """
        self.model_loader = model_loader
        self.data_manager = data_manager
        self.active = False
        self.seq_id = 0

    def start(self, seq_id):
        """
        启动任务。
        
        Args:
            seq_id (int): 任务序列号，通常用于区分同一类任务的不同执行阶段或地点。
        """
        self.active = True
        self.seq_id = seq_id
        rospy.loginfo(f"Task {self.__class__.__name__} started with seq_id {seq_id}")

    def stop(self):
        """
        停止任务。
        
        应当在此处释放资源或重置状态。
        """
        self.active = False
        rospy.loginfo(f"Task {self.__class__.__name__} stopped")

    def process(self, image):
        """
        处理单帧图像。
        
        Args:
            image (np.ndarray): 输入的图像数据 (OpenCV 格式)。
            
        Returns:
            tuple: (result_image, result_data)
                - result_image (np.ndarray): 处理后的图像 (用于可视化/调试)。
                - result_data (str or None): 任务结果字符串。
                  如果任务完成，返回用于发布到 /visual_task_done 的字符串；
                  如果未完成或无结果，返回 None。
        """
        return image, None
