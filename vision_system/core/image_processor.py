import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

class ImageProcessor:
    @staticmethod
    def imgmsg_to_cv2(img_msg):
        """ 纯 Python 实现的图像转换，无视环境冲突 """
        dtype = np.uint8
        n_channels = 3
        
        if img_msg.encoding == "bgr8":
            pass
        elif img_msg.encoding == "rgb8":
            pass
        elif img_msg.encoding == "mono8":
            n_channels = 1
        else:
            rospy.logwarn_throttle(5.0, f"非标准编码: {img_msg.encoding}，尝试作为3通道处理")

        dtype = np.dtype(dtype)
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')

        height = int(img_msg.height)
        width = int(img_msg.width)
        expected_bytes = height * width * n_channels * dtype.itemsize
        data_len = len(img_msg.data)
        if data_len < expected_bytes:
            rospy.logerr_throttle(
                1.0,
                f"图像数据长度不足: got={data_len} expected>={expected_bytes} enc={img_msg.encoding} ({width}x{height}x{n_channels})",
            )
            return None

        try:
            if n_channels == 3:
                img = np.ndarray(shape=(img_msg.height, img_msg.width, 3),
                                 dtype=dtype, buffer=img_msg.data)
            else:
                img = np.ndarray(shape=(img_msg.height, img_msg.width),
                                 dtype=dtype, buffer=img_msg.data)
        except Exception as e:
            rospy.logerr_throttle(1.0, f"图像数据解析失败: {e}")
            return None

        if img_msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            
        return img

    @staticmethod
    def resize_image(image, width=None, height=None):
        if image is None:
            return None
        h, w = image.shape[:2]
        if width is None and height is None:
            return image
        
        if width is None:
            ratio = height / float(h)
            dim = (int(w * ratio), height)
        else:
            ratio = width / float(w)
            dim = (width, int(h * ratio))
            
        return cv2.resize(image, dim, interpolation=cv2.INTER_AREA)
