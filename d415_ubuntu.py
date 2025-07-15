import pyrealsense2 as rs
import numpy as np
import cv2
import json
import torch
from ultralytics import YOLO
import Serial_ubuntu as sc
import pid_akatsuki as pid
from pathlib import Path
import asyncio

#=========================================================
pipeline = rs.pipeline()  #定义流程pipeline
config = rs.config()   #定义配置config
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)  #配置depth流
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)   #配置color流
profile = pipeline.start(config)  #流程开始
align_to = rs.stream.color  #与color流对齐
align = rs.align(align_to)

#=========================================================
# 全局变量,用于存储模型和摄像头对象
model = None
cap = None
track = 0
act_model = 0  # 用于控制动作的变量
X,Y,RIGHT_X,RIGHT_Y= 0,0,0,0    # 控制移动的变量
# 设置模型配置
script_dir = Path(__file__).parent
file_path = script_dir / 'model' / 'yolo11s.pt'

model_config = {
    'model_path': str(file_path),
    'download_url': 'https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt'
}

# 推理参数
predict_config = {
    'conf_thres': 0.6,
    'iou_thres': 0.2,
    'imgsz': 640,
    'line_width': 2,
    'device': 'cuda:0' if torch.cuda.is_available() else 'cpu'
}
def init_inference(confidence, iou):
    global model
    # 加载 YOLO 模型
    model = YOLO(model_config['model_path']).to(predict_config['device'])
    return True
0
def get_aligned_images():
    frames = pipeline.wait_for_frames()  #等待获取图像帧
    aligned_frames = align.process(frames)  #获取对齐帧
    aligned_depth_frame = aligned_frames.get_depth_frame()  #获取对齐帧中的depth帧
    color_frame = aligned_frames.get_color_frame()   #获取对齐帧中的color帧

    ############### 相机参数的获取 #######################
    intr = color_frame.profile.as_video_stream_profile().intrinsics   #获取相机内参
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  #获取深度参数(像素坐标系转相机坐标系会用到)
    camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                         'ppx': intr.ppx, 'ppy': intr.ppy,
                         'height': intr.height, 'width': intr.width,
                         'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                         }
    # 保存内参到本地
    with open('./intrinsics.json', 'w') as fp:
        json.dump(camera_parameters, fp)
    #######################################################
    
    depth_image = np.asanyarray(aligned_depth_frame.get_data())  #深度图(默认16位)
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  #深度图(8位)
    depth_image_3d = np.dstack((depth_image_8bit,depth_image_8bit,depth_image_8bit))  #3通道深度图
    color_image = np.asanyarray(color_frame.get_data())  # RGB图
    
    #返回相机内参、深度参数、彩色图、深度图、齐帧中的depth帧
    return intr, depth_intrin, color_image, depth_image, aligned_depth_frame

async def detect():
    global frame1,rgb,center1
    # 读取摄像头的帧
    frame1 = rgb.copy()
    # 使用 YOLO 模型进行推理,并传入置信度和 IoU 阈值
    results1 = model(frame1, conf=0.5, iou=0.7)
    detect_len = len(results1[0].boxes)  #获取检测结果的长度
    # 获取检测结果的坐标并绘制检测框    
    boxes1 = []
    if detect_len != 0:
        for r in results1:
            for box in r.boxes:
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                boxes1.append(xyxy)
                cv2.rectangle(frame1, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0, 255, 0), 2)
                center1 = np.array([x_center1, y_center1])
    else:
        print("未检测到目标")
        center1 = np.array([320, 5])

# 动作组以及跟踪动作组
def action(data1,data2,target_X=0.0,target_dis=0.0):
    # 返回两个PID控制器的输出值
    return int(actX.update(current=data1,target=target_X)) , abs(255-int(act_dis.update(current=data2,target=target_dis)))

def action_group(act):
    print("动作组:", act)
    print("暂时保密喵")

if __name__ == "__main__":
    model = None
    global actX,act_dis
    x_center1 = 320
    y_center1 = 240
    center1 = np.array([x_center1, y_center1])

    actX = pid.ProportionalPID(
        kd= 260,
        ki= 0,
        kp= 180,
        deadband= 0.01
    )
    act_dis = pid.ProportionalPID(
        kd= 220,
        ki= 0,
        kp= 160,
        deadband= 0.001
    )

    try:
        #==============================================================================================================================================
        #摇杆选择串口
        robot = sc.SerialCommunicator()
        while True:
                com_port ="/dev/ttyUSB0".format(comport=sc.com_switch(0))
                print("\033[32m{comport}\033[0m".format(comport=com_port))
                
                if robot.open(port=com_port, baudrate=115200, bytesize=8, parity='N', stopbits=1):
                    print("串口已打开")
                    break
                else:
                    print("\033[031m重新选择串口\033[0m")
        #===============================================================================================================================================
        # 获取函数和控制变量
        read_joystick, process_events, running = sc.button_toggle()
        
        # 创建并启动线程
        joystick_thread = sc.threading.Thread(target=read_joystick)
        event_thread = sc.threading.Thread(target=process_events)

        joystick_thread.daemon = True  # 设置为守护线程
        event_thread.daemon = True
        
        joystick_thread.start()
        event_thread.start()    # 启动摇杆读取线程
        
        #==============================================================================================================================================
        if init_inference(confidence=predict_config["conf_thres"], iou=predict_config["iou_thres"]):
            while 1:
                
                #事件遍历
                sc.pygame.event.pump()

                #获取对齐的图像与相机内参
                intr, depth_intrin, rgb, depth, aligned_depth_frame = get_aligned_images() 
                
                #神经网络推理
                #=======================================================================

                if model is None:
                    print("推理未初始化,请先调用 init_inference 函数。")

                asyncio.run(detect())   #协程推理
                #=======================================================================

                x = int(center1[0])
                y = int(center1[1])
                #(x, y)点的真实深度值
                dis = aligned_depth_frame.get_distance(x, y)  
                camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], dis)  #(x, y)点在相机坐标系下的真实值,为一个三维向量。其中camera_coordinate[2]仍为dis,camera_coordinate[0]和camera_coordinate[1]为相机坐标系下的xy真实距离。
                if camera_coordinate != (0.0,0.0,0.0):
                    last_coordinate = camera_coordinate
                    print(camera_coordinate)
                else:
                    print(last_coordinate)

                # 显示彩色图像
                cv2.imshow('RGB image',frame1)

                key = cv2.waitKey(1)
                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('q') or key == 27:
                    pipeline.stop()
                    break
                #=========================================================================================================================
                # 处理摇杆输入
                if sc.joystick.get_button(6) == 1:
                    break
                
                # 执行动作组
                if sc.joystick.get_button(13) == 1:
                    if camera_coordinate[2] !=0.0:
                        act_group = 1
                    else:
                        act_group = 2
                else:
                    act_group = 0

                action_group(act_group)
                act_group = 0

                sc.time.sleep(0.01)
                robot.send()
        cv2.destroyAllWindows()
    except Exception as e:
        print(f"发生异常:{e}")

    except KeyboardInterrupt:
        print("程序被用户中断")

    finally:
        sc.msg[1],sc.msg[2],sc.msg[3],sc.msg[4] = 127,127,127,127
        robot.send()
        # 关闭串口
        robot.close()
        print("串口已关闭")
        # 等待线程结束
        running = False  # 设置停止标志
        sc.pygame.quit()
        print("\n程序正在退出...")