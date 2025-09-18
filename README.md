MTI战队视觉方案开源
==================

## 前言
---------------

在初步研究了排球比赛的规则之后，我们组内成员偏向于认为视觉方案在这次比赛中可能会有一个比较重要的作用。但是在实际比赛中，我们发现部分队伍因为灯光影响而放弃视觉方案。部分学校则是采取了风格特异的抗干扰方案排除灯光影响。例如某大学是采取了现场标记数据集，而另一个大学则是数据集一开始就使用蓝色为主的排球，由此达成抗干扰效果。尽管由于其他问题我们的视觉识别并未起到作用，但是鉴于实地测试表现我决定仍然将其开源。电控部分代码则开源在[这个仓库](https://github.com/Enecell/CURC-Robocon2025-Volleyball_MTI)。本代码在[这里](https://github.com/akatsuki-kaze/MTI_volleyball_opensource/tree/main/fjut_volleyball_R1)开源,在闲鱼购买请联系原作者1652107474@qq.com<br>

## 一、代码功能综述
---------------
本代码结构为d415_ubuntu.py作为主程序，anti_light.py作为抗灯光部分，Serial_akatsuki_simple.py作为串口通讯相关的函数处理（包括数据内容以及编码发送），pid_akatsuki.py是用于跟踪的pid控制器。run.bat是为win环境准备的启动程序（win版本主程序为d415）。<br>
本代码不依赖ros，需求python版本3.9及以上，3.10经过测试稳定运行。要求库已经内置在requirementa.txt中。ubuntu需要使用python命令启动。程序启动时需要接入xinput支持的手柄，realsense d4系列带rgb摄像头（d410可以使用深度摄像头传输rgb流，d430不行），以及ch340（我们的方案是自带ch340的nrf24射频）

## 二、主函数代码解析
-----------------
12行开始代码部分
```
pipeline = rs.pipeline()  #定义流程pipeline
config = rs.config()   #定义配置config
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)  #配置depth流
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)   #配置color流
profile = pipeline.start(config)  #流程开始
align_to = rs.stream.color  #与color流对齐
align = rs.align(align_to)
```
以上是realsense相关的初始化<br>
```
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
```
这里是关于yolo推理部分的初始化，其中```X,Y,RIGHT_X,RIGHT_Y= 0,0,0,0```会在稍后作为传入信息数组的变量，提前进行全局声明和初始化。```model_config```是模型的路径，仍然保留了网络下载路径，但是实际代码中没有相关功能。```init_inference```函数用于初始化模型，若初始化成功会返回true。<br>
接下来直接转到识别部分
```async def detect():
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
```
这段代码对应yolo的识别部分。由于anti_light代码在nuc上，后续会惊醒补充。<br>
接下来来到代码的主函数部分，108行开始：
``` model = None
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
```
提前声明部分变量，实例化需要用到的两个pid控制器（对应水平和竖直方向）<br>
接下来是串口选择部分，这个部分目前只有win版本有效，ubuntu版本已经删除。选择串口时，用手柄的hat（十字键）进行选择，上下为加减5，左右加减1（ubuntu版本功能为上下加减1）。选择完成后，按下手柄A按键确定串口。<br>
170行的```asyncio.run(detect())   #协程推理```使用了asyncio库进行协程，达到优化推理产生的控制延迟的效果。实际上发现在一定程度上也优化了推理性能。<br>
173行开始的这段代码中
```
    x = int(center1[0])
    y = int(center1[1])
    #(x, y)点的真实深度值
    dis = aligned_depth_frame.get_distance(x, y)  
    #(x, y)点在相机坐标系下的真实值,为一个三维向量。其中camera_coordinate[2]仍为dis,camera_coordinate[0]和camera_coordinate[1]为相机坐标系下的xy真实距离。
    camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], dis)  
    if camera_coordinate != (0.0,0.0,0.0):
        last_coordinate = camera_coordinate
        print(camera_coordinate)
    else:
        print(last_coordinate)
```
这段代码会对像素坐标进行获取。由于深度摄像头在距离过近的目标识别会输出0，0，0的坐标值，因此会对目标最后的位置进行检测，决定是否写入坐标值<br>

接下来是198行，对应动作组
```
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
```
这段代码在手柄左摇杆按下时会进入自动跟踪状态，同时通过目标不同状态选择动作组。在执行程序```action_group```结束后```act_group```会归为0，防止动作重复执行。
```
    sc.time.sleep(0.01)
    robot.send()
```
这段为发送所有的数据组，sleep为阻塞线程，防止串口发送频率太快导致报错<br>
[BACK](akatsuki-kaze.github.io)
