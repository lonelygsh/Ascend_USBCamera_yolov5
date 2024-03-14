## Ascend USBCamera yolov5

### 介绍

本仓的样例基于 Orangepi ai pro 昇腾 310B 平台，对官方样例[USB 摄像头 yolov5 目标检测](https://gitee.com/ascend/EdgeAndRobotics/tree/master/Samples/YOLOV5USBCamera)进行了在 ROS2 环境中的部署与优化。

修改内容如下：

1、通过 v4l2 直接获取 USB 摄像头 MJPG 编码的图片再通过 DVPP 硬件 JPEGD 解码为 YUV420 格式的图片 代替 官方用 ffmpeg 直接获取 YUYV 格式图像，将获取的 USB 摄像头的视频流由 10 帧提升到的 30 帧（市面上大多数 USB 摄像头的规格是采集 720P 的图片都只支持 MJPG 30 帧，而 YUYV 只能采集到 10 帧甚至更低）。

2、新增一个 ROS2 网络节点，将获取的 YUV 格式数据通过**ROS2 的节点零拷贝机制**（共享内存减少 CPU 的消耗）传输给网络节点，在网络节点中对 YUV 格式数据进行 DVPP 的硬件 VENC 编码为 H264 视频流，通过 ffmpeg 传输给 PC 端，同时网络节点会接受 yolov5 的检测结果，同时将检测结果也传输给 PC 端，这样能避免板端使用 opencv 对图像进行拷贝，颜色转换以及画框显示等操作，很大程度减少了对 CPU 资源的占用。

3、新增 PC 端对板端 H264 视频流以及检测结果接收解码处理显示代码。

### 使用

```
-/Ascend_USBCamera_yolov5
    |--/Ascend310B4：昇腾310B板端代码
        |--/config：模型及配置文件
            |--/face_recognition：我训练的人脸识别模型
            |--/sample_recognition：官方示例的模型
        |--/src：ROS2节点
            |--/my_interfaces：ROS2自定义的消息节点
            |--/socket：ROS2网络节点
            |--/vision：yolov5视觉代码
                |--/face_recognition：针对官方示例流程优化后的ROS2节点
                |--/sample_recognition：适配ROS2的官方示例代码的ROS2节点
            |--/zero_copy：零拷贝通信相关
                |--/config：零拷贝通信的配置
                |--/ros2_shm_msgs：零拷贝的库
    |--/PC：PC端代码
```

使用前请务必配置好 humble 版本的 ROS2 环境，并在上面的/ros2_shm_msgs 文件夹下下载编译配置好[ros2_shm_msgs](https://github.com/ZhenshengLee/ros2_shm_msgs)。可以根据 ros2_shm_msgs 的示例进行测试，必须保证配置好 ROS2 节点间的零拷贝通信。

编译与运行:

在 PC 中：

```
make
./orangepi_imageShow
```

在 ROS2 的工作空间下

```
colcon build
ros2 launch face_recognition face_recognition_launch.py
```

请注意，在编译前要修改 yolov5 视觉代码中的模型路径以及网络节点中的 IP 以及 PC 端代码中的 IP。

经过测试，在 PC 端显示的结果来看，板端在使用 yolov5 目标检测时能跑满摄像头的 30 帧而且 3 核控制核 control core 的占用均不会超过 50%。
