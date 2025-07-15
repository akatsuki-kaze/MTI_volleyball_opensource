import cv2
import numpy as np

def calculate_gray_variance(image, bbox):
    """计算YOLO识别框内物体的平均灰度值的方差"""
    # 处理 xyxy 格式 (x1, y1, x2, y2)
    if len(bbox) == 4:
        x1, y1, x2, y2 = map(int, bbox)
        w, h = x2 - x1, y2 - y1
    # 处理 xywh 格式 (x, y, width, height)
    elif len(bbox) == 6:  # YOLOv8 的 boxes 可能包含更多信息
        x1, y1, x2, y2 = map(int, bbox[:4])
        w, h = x2 - x1, y2 - y1
    else:
        raise ValueError(f"不支持的边界框格式: {bbox}")
    
    # 提取框内区域
    roi = image[y1:y1+h, x1:x1+w]
    
    # 转换为灰度图
    if len(roi.shape) == 3:
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    else:
        gray_roi = roi
    
    # 计算平均灰度值
    mean_gray = np.mean(gray_roi)
    
    # 计算方差
    variance = np.var(gray_roi)
    
    return variance, mean_gray

def process_yolo_results(images, yolo_results):
    """处理YOLO识别结果，计算各框内灰度方差并提取最大方差框的中心坐标"""
    image = images
    if image is None:
        raise FileNotFoundError(f"无法读取图像")
    
    center = []
    max_variance = -1
    max_bbox = None
    
    # 处理每个识别框
    for r in yolo_results:
        # 尝试转换不同格式的边界框
        if hasattr(r, 'boxes'):
            # YOLOv8 格式
            for box in r.boxes:
                bbox = box.xyxy[0].cpu().numpy()  # 获取 xyxy 格式
                variance, _ = calculate_gray_variance(image, bbox)
                
                # 更新最大方差及其对应的框
                if variance > max_variance:
                    max_variance = variance
                    max_bbox = bbox
        else:
            # 假设是 [x, y, w, h] 格式
            variance, _ = calculate_gray_variance(image, r)
            if variance > max_variance:
                max_variance = variance
                max_bbox = r
    
    # 计算最大方差框的中心坐标
    if max_bbox is not None:
        if len(max_bbox) == 4:  # xyxy 格式
            x1, y1, x2, y2 = max_bbox
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
        else:  # xywh 格式
            x, y, w, h = max_bbox
            center_x = x + w / 2
            center_y = y + h / 2
        center = [center_x, center_y]
    
    return center, max_variance

# 示例用法
if __name__ == "__main__":
    # 示例图像路径
    image_path = "path_to_your_image.jpg"
    
    # 示例YOLO识别结果
    yolo_results = [
        [100, 150, 300, 400],  # 框1 (x1, y1, x2, y2)
        [300, 200, 450, 400],  # 框2
        [450, 100, 630, 320]   # 框3
    ]
    
    try:
        # 模拟读取图像
        image = cv2.imread(image_path)
        if image is None:
            # 创建一个示例图像用于测试
            image = np.zeros((500, 700, 3), dtype=np.uint8)
            cv2.rectangle(image, (100, 150), (300, 400), (255, 255, 255), -1)
            
        center, max_variance = process_yolo_results(image, yolo_results)
        print(f"最大方差框的中心坐标: {center}")
        #print(f"最大方差值: {max_variance}")
    except Exception as e:
        print(f"错误: {e}")