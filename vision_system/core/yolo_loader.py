import os
import rospy
import numpy as np
import cv2
from PIL import Image as PilImage

# Import torch and onnxruntime for YOLO/Classifier
try:
    import torch
    import torchvision.models as models
    import torchvision.transforms as transforms
    import torch.nn as nn
except ImportError:
    rospy.logwarn("Torch not installed. ClassifierModel will not work.")
    torch = None

try:
    import onnxruntime
except ImportError:
    rospy.logwarn("ONNX Runtime not installed. YOLOv10Model will not work.")
    onnxruntime = None

class ClassifierModel:
    def __init__(self, model_path, num_classes, device=None):
        if torch is None:
            rospy.logerr("Torch not installed, cannot load ClassifierModel")
            raise ImportError("Torch not installed")
            
        self.device = device if device else torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        rospy.loginfo(f"Loading Classifier: {model_path} on {self.device}")
        
        self.model = models.mobilenet_v3_small(weights=None)
        num_ftrs = self.model.classifier[3].in_features
        self.model.classifier[3] = nn.Linear(num_ftrs, num_classes)
        
        # Load state dict
        try:
            state_dict = torch.load(model_path, map_location=self.device)
            self.model.load_state_dict(state_dict)
        except Exception as e:
            rospy.logerr(f"Failed to load classifier weights: {e}")
            raise
            
        self.model = self.model.to(self.device)
        self.model.eval()
        
        self.transform = transforms.Compose([
            transforms.Resize(256), transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])

    def infer(self, image_crop):
        # image_crop is a numpy array (BGR) from cv2
        pil_img = PilImage.fromarray(cv2.cvtColor(image_crop, cv2.COLOR_BGR2RGB))
        inp = self.transform(pil_img).unsqueeze(0).to(self.device)
        with torch.no_grad():
            out = self.model(inp)
            _, pred = torch.max(out, 1)
        return pred.item()

class YOLOv10Model:
    def __init__(self, onnx_path, input_size, confidence_thres=0.5, iou_thres=0.45, class_names=None, version="yolov10n"):
        if onnxruntime is None:
            rospy.logerr("Failed to import onnxruntime. Please ensure it is installed.")
            raise ImportError("onnxruntime not installed")

        self.version = version
        self.confidence_thres = confidence_thres
        self.iou_thres = iou_thres
        self.input_height = input_size
        self.input_width = input_size
        self.class_names = class_names if class_names is not None else ['class_0']
        
        sess_options = onnxruntime.SessionOptions()
        sess_options.intra_op_num_threads = os.cpu_count() or 4
        sess_options.execution_mode = onnxruntime.ExecutionMode.ORT_SEQUENTIAL
        sess_options.graph_optimization_level = onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL

        providers = []
        available = onnxruntime.get_available_providers()
        rospy.loginfo(f"ONNX Runtime Path: {onnxruntime.__file__}")
        rospy.loginfo(f"Available Providers: {available}")
        
        if 'OpenVINOExecutionProvider' in available:
            providers.append(('OpenVINOExecutionProvider', {'device_type': 'CPU'}))
        providers.append('CPUExecutionProvider')

        try:
            self.session = onnxruntime.InferenceSession(onnx_path, sess_options, providers=providers)
        except Exception as e:
            rospy.logwarn(f"OpenVINO init failed, fallback to CPU: {e}")
            self.session = onnxruntime.InferenceSession(onnx_path, sess_options, providers=['CPUExecutionProvider'])

        rospy.loginfo(f"Current Inference Providers: {self.session.get_providers()} | Version: {self.version}")

        model_inputs = self.session.get_inputs()
        self.input_name = model_inputs[0].name
        self.output_name = self.session.get_outputs()[0].name

    def preprocess(self, image):
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.img_height, self.img_width = image_rgb.shape[:2]
        
        ratio = min(self.input_width / self.img_width, self.input_height / self.img_height)
        new_width = int(self.img_width * ratio)
        new_height = int(self.img_height * ratio)
        
        resized_img = cv2.resize(image_rgb, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
        
        canvas = np.full((self.input_height, self.input_width, 3), 114, dtype=np.uint8)
        start_y = (self.input_height - new_height) // 2
        start_x = (self.input_width - new_width) // 2
        canvas[start_y:start_y + new_height, start_x:start_x + new_width, :] = resized_img
        
        input_image = canvas.transpose((2, 0, 1))
        input_image = np.ascontiguousarray(input_image)
        input_tensor = input_image.astype(np.float32) / 255.0
        input_tensor = np.expand_dims(input_tensor, axis=0)
        return input_tensor

    def postprocess(self, output):
        predictions = np.squeeze(output[0])
        
        # YOLOv10 NMS-Free
        if predictions.ndim == 2 and predictions.shape[-1] == 6:
            if not hasattr(self, '_logged_mode'):
                rospy.loginfo(">>> Mode Confirmed: YOLOv10 NMS-Free Mode (No Post-process Latency)")
                self._logged_mode = True

            boxes = predictions[:, :4]
            scores = predictions[:, 4]
            class_ids = predictions[:, 5]
            
            mask = scores > self.confidence_thres
            boxes = boxes[mask]
            scores = scores[mask]
            class_ids = class_ids[mask].astype(int)
            return self.rescale_boxes(boxes, scores, class_ids)

        # Traditional NMS
        if not hasattr(self, '_logged_mode'):
            rospy.loginfo(">>> Mode Confirmed: Traditional NMS Mode (OpenCV Accelerated)")
            self._logged_mode = True

        boxes = predictions[:, :4]
        if predictions.shape[1] > 5:
            class_scores = predictions[:, 5:]
            class_ids = np.argmax(class_scores, axis=1)
            confidences = predictions[:, 4] * np.max(class_scores, axis=1)
        else:
            confidences = predictions[:, 4]
            class_ids = predictions[:, 5]

        mask = confidences > self.confidence_thres
        boxes = boxes[mask]
        scores = confidences[mask]
        class_ids = class_ids[mask]
        
        if len(boxes) == 0: return [], [], []

        nms_boxes = boxes.copy()
        nms_boxes[:, 2] = nms_boxes[:, 2] - nms_boxes[:, 0]
        nms_boxes[:, 3] = nms_boxes[:, 3] - nms_boxes[:, 1]
        
        indices = cv2.dnn.NMSBoxes(nms_boxes.tolist(), scores.tolist(), self.confidence_thres, self.iou_thres)
        if len(indices) > 0:
            indices = indices.flatten()
            return self.rescale_boxes(boxes[indices], scores[indices], class_ids[indices].astype(int))
        return [], [], []

    def rescale_boxes(self, boxes, scores, class_ids):
        ratio = min(self.input_width / self.img_width, self.input_height / self.img_height)
        start_y = (self.input_height - int(self.img_height * ratio)) // 2
        start_x = (self.input_width - int(self.img_width * ratio)) // 2

        boxes[:, [0, 2]] = (boxes[:, [0, 2]] - start_x) / ratio
        boxes[:, [1, 3]] = (boxes[:, [1, 3]] - start_y) / ratio
        
        boxes[:, 0] = np.clip(boxes[:, 0], 0, self.img_width)
        boxes[:, 1] = np.clip(boxes[:, 1], 0, self.img_height)
        boxes[:, 2] = np.clip(boxes[:, 2], 0, self.img_width)
        boxes[:, 3] = np.clip(boxes[:, 3], 0, self.img_height)
        return boxes.astype(int), scores, class_ids

    def infer(self, image):
        import time
        t_start = time.time()
        input_tensor = self.preprocess(image)
        outputs = self.session.run([self.output_name], {self.input_name: input_tensor})
        boxes, scores, class_ids = self.postprocess(outputs)
        
        # Draw detections on a copy of the image
        res_img = image.copy()
        for box, score, class_id in zip(boxes, scores, class_ids):
            x1, y1, x2, y2 = box
            label = self.class_names[int(class_id)] if int(class_id) < len(self.class_names) else str(class_id)
            cv2.rectangle(res_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(res_img, f"{label} {score:.2f}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
        inference_time = (time.time() - t_start) * 1000
        return res_img, inference_time, (boxes, scores, class_ids)

class YOLOLoader:
    def __init__(self, config):
        self.config = config
        self.models = {}

    def load_model(self, name):
        if name in self.models:
            return self.models[name]
        
        cfg = next((m for m in self.config.MODEL_LIST if m["name"] == name), None)
        if not cfg:
            rospy.logerr(f"Model config not found for: {name}")
            return None

        try:
            if cfg.get("task_type") == "classify":
                model = ClassifierModel(
                    model_path=cfg["path"],
                    num_classes=cfg["num_classes"]
                )
            else:
                # Default to YOLO
                model = YOLOv10Model(
                    onnx_path=cfg["path"],
                    input_size=cfg["input_size"],
                    confidence_thres=cfg["conf_thres"],
                    iou_thres=cfg["iou_thres"],
                    class_names=cfg["class_names"],
                    version=cfg.get("version", "yolov10n")
                )
                
            self.models[name] = model
            return model
        except Exception as e:
            rospy.logerr(f"Failed to load model {name}: {e}")
            return None

    def load_yolo_model(self, name):
        return self.load_model(name)
