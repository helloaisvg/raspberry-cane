import argparse  # 导入用于处理命令行参数的库
import cv2  # 导入 OpenCV 库，用于图像处理
import numpy as np  # 导入 NumPy 库，用于数值计算
import onnxruntime as ort  # 导入 ONNX 运行时库，用于加载和推理 ONNX 模型
import time  # 导入时间库，用于计时

class yolov5_lite():
    def __init__(self, model_pb_path, label_path, confThreshold=0.5, nmsThreshold=0.5):
        so = ort.SessionOptions()  # 创建 ONNX 会话选项
        so.log_severity_level = 3  # 设置日志严重性等级
        self.net = ort.InferenceSession(model_pb_path, so)  # 加载 ONNX 模型
        self.classes = list(map(lambda x: x.strip(), open(label_path, 'r').readlines()))  # 读取类别标签
        print(self.classes)  # 打印类别标签
        self.confThreshold = confThreshold  # 设置置信度阈值
        self.nmsThreshold = nmsThreshold  # 设置非极大值抑制阈值
        self.input_shape = (self.net.get_inputs()[0].shape[2], self.net.get_inputs()[0].shape[3])  # 获取输入尺寸

    def letterBox(self, srcimg, keep_ratio=True):
        top, left, newh, neww = 0, 0, self.input_shape[0], self.input_shape[1]  # 初始化边界框参数
        if keep_ratio and srcimg.shape[0] != srcimg.shape[1]:  # 如果保持比例且源图不是正方形
            hw_scale = srcimg.shape[0] / srcimg.shape[1]  # 计算高宽比
            if hw_scale > 1:  # 如果高大于宽
                newh, neww = self.input_shape[0], int(self.input_shape[1] / hw_scale)  # 设置新的高度和宽度
                img = cv2.resize(srcimg, (neww, newh), interpolation=cv2.INTER_AREA)  # 调整图像大小
                left = int((self.input_shape[1] - neww) * 0.5)  # 计算左边距
                img = cv2.copyMakeBorder(img, 0, 0, left, self.input_shape[1] - neww - left, cv2.BORDER_CONSTANT,
                                         value=0)  # 添加边框
            else:  # 如果宽大于高
                newh, neww = int(self.input_shape[0] * hw_scale), self.input_shape[1]  # 设置新的高度和宽度
                img = cv2.resize(srcimg, (neww, newh), interpolation=cv2.INTER_AREA)  # 调整图像大小
                top = int((self.input_shape[0] - newh) * 0.5)  # 计算上边距
                img = cv2.copyMakeBorder(img, top, self.input_shape[0] - newh - top, 0, 0, cv2.BORDER_CONSTANT, value=0)  # 添加边框
        else:
            img = cv2.resize(srcimg, self.input_shape, interpolation=cv2.INTER_AREA)  # 如果不保持比例，直接调整大小
        return img, newh, neww, top, left  # 返回处理后的图像及其新尺寸和边距

    def postprocess(self, frame, outs, pad_hw):
        newh, neww, padh, padw = pad_hw  # 解包填充参数
        frameHeight = frame.shape[0]  # 获取帧的高度
        frameWidth = frame.shape[1]  # 获取帧的宽度
        ratioh, ratiow = frameHeight / newh, frameWidth / neww  # 计算高度和宽度的比例
        classIds = []  # 存储类别 ID
        confidences = []  # 存储置信度
        boxes = []  # 存储边界框坐标

        for detection in outs:  # 遍历检测结果
            score = detection[4]
            print(f"Score value: {score}, Score shape: {score.shape if hasattr(score, 'shape') else None}")  # 打印 score 的值和形状用于调试
            if isinstance(score, np.ndarray) and score.size > 1:
                print("Score is an array, using the first element.")
                score = score[0]  # 如果 score 是数组且长度大于 1，取第一个元素

            classId = np.argmax(detection[5:])  # 获取类别 ID
            if score > self.confThreshold:  # 如果置信度高于阈值
                def get_single_value(value):
                    if isinstance(value, np.ndarray) and value.size > 1:
                        print("Value is an array, using the first element.")
                        return value[0]
                    return value

                x1_value = get_single_value(detection[0])
                y1_value = get_single_value(detection[1])
                x2_value = get_single_value(detection[2])
                y2_value = get_single_value(detection[3])

                x1 = int((x1_value - padw) * ratiow)  # 计算左上角 X 坐标
                y1 = int((y1_value - padh) * ratioh)  # 计算左上角 Y 坐标
                x2 = int((x2_value - padw) * ratiow)  # 计算右下角 X 坐标
                y2 = int((y2_value - padh) * ratioh)  # 计算右下角 Y 坐标

                classIds.append(classId)  # 添加类别 ID
                confidences.append(score)  # 添加置信度
                boxes.append([x1, y1, x2, y2])  # 添加边界框坐标
                # 计算目标中心
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                print(f"Detected object center: (center_x: {center_x}, center_y: {center_y})")

                # 计算图像的中心点
                frame_center_x = frame.shape[1] // 2
                frame_center_y = frame.shape[0] // 2
                offset_x = center_x - frame_center_x
                offset_y = center_y - frame_center_y

                print(f"Object offset from center: (offset_x: {offset_x}, offset_y: {offset_y})")

        # 使用非极大值抑制（NMS）消除冗余重叠框
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.confThreshold, self.nmsThreshold)

        for ind in indices:  # 遍历保留的框
            ind = ind.item()  # 将索引转换为 Python 整数
            # 在帧上绘制预测框
            frame = self.drawPred(frame, classIds[ind], confidences[ind], boxes[ind][0], boxes[ind][1], boxes[ind][2], boxes[ind][3])
        return frame  # 返回处理后的帧

    def drawPred(self, frame, classId, conf, x1, y1, x2, y2):
        # 在帧上绘制边界框
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), thickness=2)  # 绘制矩形框

        label = '%.2f' % conf  # 格式化置信度
        text = '%s:%s' % (self.classes[int(classId)], label)  # 创建文本标签

        # 在边界框顶部显示标签
        labelSize, baseLine = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)  # 获取文本大小
        y1 = max(y1, labelSize[1])  # 确保框不覆盖文本
        cv2.putText(frame, text, (x1, y1 - 10), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0), thickness=1)  # 绘制文本
        return frame  # 返回更新后的帧

    def detect(self, srcimg):
        # 进行目标检测
        img, newh, neww, top, left = self.letterBox(srcimg)  # 调整图像大小
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # 转换颜色通道
        img = img.astype(np.float32) / 255.0  # 将像素值归一化到 [0, 1]
        blob = np.expand_dims(np.transpose(img, (2, 0, 1)), axis=0)  # 转换为模型输入格式

        t1 = time.time()  # 记录推理开始时间
        outs = self.net.run(None, {self.net.get_inputs()[0].name: blob})[0]  # 进行推理
        cost_time = time.time() - t1  # 计算推理耗时
        print(outs.shape)  # 打印输出形状

        srcimg = self.postprocess(srcimg, outs, (newh, neww, top, left))  # 后处理，生成最终检测结果
        infer_time = 'Inference Time: ' + str(int(cost_time * 1000)) + 'ms'  # 格式化推理时间
        cv2.putText(srcimg, infer_time, (5, 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 0, 0), thickness=1)  # 显示推理时间
        return srcimg  # 返回带有检测结果的图像


if __name__ == "__main__":
    parser = argparse.ArgumentParser()  # 创建命令行参数解析器
    # parser.add_argument('--imgpath', type=str, default='image.jpg', help="image path")  # 可选：图像路径参数
    parser.add_argument('--modelpath', type=str, default='best2.onnx', help="onnx filepath")  # 模型文件路径参数
    parser.add_argument('--classfile', type=str, default='coco.names', help="classname filepath")  # 类别文件路径参数
    parser.add_argument('--confThreshold', default=0.5, type=float, help='class confidence')  # 置信度阈值参数
    parser.add_argument('--nmsThreshold', default=0.6, type=float, help='nms iou thresh')  # NMS 阈值参数
    args = parser.parse_args()  # 解析命令行参数

    net = yolov5_lite(args.modelpath, args.classfile, confThreshold=args.confThreshold, nmsThreshold=args.nmsThreshold)  # 初始化 YOLOv5 模型

    # 摄像头捕获
    video = 0  # 默认使用摄像头
    cap = cv2.VideoCapture(video)  # 打开摄像头
    flag_det = False  # 初始化检测标志
    while True:  # 循环读取视频帧
        success, img0 = cap.read()  # 读取一帧图像
        if success:  # 如果成功读取图像

            if flag_det:  # 如果检测标志为真
                t1 = time.time()  # 记录开始时间
                img0 = net.detect(img0.copy())  # 进行目标检测
                t2 = time.time()  # 记录结束时间

                str_FPS = "FPS: %.2f" % (1. / (t2 - t1))  # 计算帧率

                cv2.putText(img0, str_FPS, (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 3)  # 显示帧率

            cv2.imshow("video", img0)  # 显示视频帧
        key = cv2.waitKey(1) & 0xFF  # 等待按键输入
        if key == ord('q'):  # 如果按下 'q' 键
            break  # 退出循环
        elif key & 0xFF == ord('s'):  # 如果按下 's' 键
            flag_det = not flag_det  # 切换检测标志
            print(flag_det)  # 打印当前检测状态

    cap.release()  # 释放摄像头资源
    cv2.destroyAllWindows()  # 关闭所有 OpenCV 窗口