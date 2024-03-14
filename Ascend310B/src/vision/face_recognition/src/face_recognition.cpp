#include "face_recognition/face_recognition.h"

bool stopFlag = false;
acllite::Queue<ModelInputData> ModelInputDataQueue(30);
acllite::Queue<ModelOnputData> ModelOnputDataQueue(30);

bool sortScore(const BoundBox &a, const BoundBox &b)
{
    return a.score > b.score;
}

void handle_sigint(int sig)
{
    (void)sig;
    stopFlag = true;
    rclcpp::shutdown();
}

void Face_recognition::CameraInit()
{
    // 打开摄像头
    camera_fd = open(device_path, O_RDWR);
    if (camera_fd == -1)
    {
        perror("Opening video device");
        return;
    }
    // 设置摄像头采集图像的宽高和编码格式
    v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.width = cameraImage_width;
    fmt.fmt.pix.height = cameraImage_height;
    if (ioctl(camera_fd, VIDIOC_S_FMT, &fmt) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "ioctl Error 1");
        perror("ioctl");
        return;
    }

    // 申请内核缓冲区队列
    v4l2_requestbuffers reqbuf = {};
    reqbuf.count = 4; // 申请四个缓冲区
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(camera_fd, VIDIOC_REQBUFS, &reqbuf) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "ioctl Error 2");
        perror("ioctl");
        return;
    }

    for (unsigned int i = 0; i < reqbuf.count; i++)
    {
        v4l2Buf.index = i;
        v4l2Buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2Buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(camera_fd, VIDIOC_QUERYBUF, &v4l2Buf) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "查询内核空间失败");
            perror("ioctl");
            return;
        }

        imageBuf[i] = mmap(NULL, v4l2Buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, camera_fd, v4l2Buf.m.offset);
        if (imageBuf[i] == MAP_FAILED)
        {
            RCLCPP_ERROR(this->get_logger(), "mmap Error ");
            perror("mmap");
            return;
        }

        if (ioctl(camera_fd, VIDIOC_QBUF, &v4l2Buf) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "放回失败");
            perror("ioctl");
            return;
        }
    }

    // 开始采集
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(camera_fd, VIDIOC_STREAMON, &type) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "ioctl Error 4");
        perror("ioctl");
        return;
    }
}

void Face_recognition::DestroyResource()
{
    aclError ret;

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "reset device failed");
    }
    RCLCPP_INFO(this->get_logger(), "end to reset device is %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "finalize acl failed");
    }
    RCLCPP_INFO(this->get_logger(), "end to finalize acl");
}

void Face_recognition::populateLoanedMessage(rclcpp::LoanedMessage<imageTopic> &loanedMsg)
{
    imageTopic &msg = loanedMsg.get();
    SharedMemoryPtr_cvimage->header.stamp = now();
    SharedMemoryPtr_cvimage->toImageMsg(msg);
    // RCLCPP_INFO(this->get_logger(), "Publishing ");
}

void Face_recognition::GetPreProcessImage()
{
    CameraInit();

    aclError aclRet = aclrtSetCurrentContext(PreProcessContext_);
    CHECK_RET(aclRet == ACL_SUCCESS, LOG_PRINT("[ERROR] aclrtSetCurrentContext failed. ERROR: %d", aclRet); return);
    // create stream
    aclrtCreateStream(&PreProcessStream_);
    // aclrtGetRunMode(&runMode);
    RCLCPP_INFO(this->get_logger(), "create PreProcessStream_ success");

    // 3.创建图片数据处理通道时的通道描述信息，dvppChannelDesc_是acldvppChannelDesc类型
    dvppChannelDesc_ = acldvppCreateChannelDesc();
    // 4.创建图片数据处理的通道。
    acldvppCreateChannel(dvppChannelDesc_);
    RCLCPP_INFO(this->get_logger(), "dvpp init resource success");

    acllite::ImageData jpegdImage;
    acllite::ImageData resizeImage;
    acllite::ImageSize modelSize(modelWidth, modelHeight);

    // auto prevTime = std::chrono::steady_clock::now();
    while (!stopFlag)
    {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(camera_fd, &fds);
        struct timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;
        int r = select(camera_fd + 1, &fds, NULL, NULL, &tv);

        if (r == -1)
        {
            perror("select");
        }
        else if (r == 0)
        {
            // timeout, check stopFlag
            if (stopFlag)
            {
                break;
            }
        }
        else
        {
            // Request a frame from the camera
            if (ioctl(camera_fd, VIDIOC_DQBUF, &v4l2Buf) == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "camera Request Error");
                perror("ioctl");
                break;
            }

            // auto curTime = std::chrono::steady_clock::now();
            // double duration = std::chrono::duration<double>(curTime - prevTime).count();
            // double fps = 1.0 / duration;
            // std::cout << "duration: " << duration << std::endl;
            // std::cout << "FPS: " << fps << std::endl;
            // prevTime = curTime;

            // cv::Mat MjpgImg(cameraImage_height * 3 / 2, cameraImage_width, CV_8UC1);
            // memcpy(MjpgImg.data, imageBuf[v4l2Buf.index], jpegdBufSize);
            // SharedMemoryPtr_cvimage->image = MjpgImg;
            // auto loanedMsg = ImagePublisher_ZeroCopy_->borrow_loaned_message();
            // populateLoanedMessage(loanedMsg);
            // ImagePublisher_ZeroCopy_->publish(std::move(loanedMsg));

            jpegdImage = JpegD(imageBuf[v4l2Buf.index], v4l2Buf.length, PIXEL_FORMAT_YUV_SEMIPLANAR_420);

            // cv::Mat MjpgImg(cameraImage_height * 3 / 2, cameraImage_width, CV_8UC1);
            // memcpy(MjpgImg.data, imageBuf[v4l2Buf.index], jpegdBufSize);
            cv::Mat MjpgImg(jpegdImage.height * 3 / 2, jpegdImage.width, CV_8UC1, jpegdImage.data.get());
            SharedMemoryPtr_cvimage->image = MjpgImg;
            auto loanedMsg = ImagePublisher_ZeroCopy_->borrow_loaned_message();
            populateLoanedMessage(loanedMsg);
            ImagePublisher_ZeroCopy_->publish(std::move(loanedMsg));

            // Release the frame back to the camera
            if (ioctl(camera_fd, VIDIOC_QBUF, &v4l2Buf) == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "camera Release Error");
                perror("ioctl");
                break;
            }

            ResizeUpperLeft(jpegdImage, resizeImage, modelSize);
            ModelInputData inputData;
            inputData.data = resizeImage.data;
            inputData.size = resizeImage.size;
            inputData.videoEnd = false;
            // cv::Mat yuvImg(jpegdImage.height * 3 / 2, jpegdImage.width, CV_8UC1);
            // memcpy(yuvImg.data, (unsigned char *)jpegdImage.data.get(), jpegdImage.size);
            // cv::cvtColor(yuvImg, inputData.srcImg, cv::COLOR_YUV2BGR_NV12);

            // cv::imshow("usb-show-demo", inputData.srcImg);
            // char c = (char)cv::waitKey(1);
            // if (c == 'q')
            //     break;

            while (1)
            {
                if (ModelInputDataQueue.Push(inputData))
                {
                    break;
                }
                usleep(15);
            }
        }
    }
    // 停止采集
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(camera_fd, VIDIOC_STREAMOFF, &type) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "ioctl Error 4");
        perror("ioctl");
        return;
    }

    for (int i = 0; i < 4; i++)
    {
        munmap(imageBuf[i], v4l2Buf.length);
    }
    close(camera_fd);

    ModelInputData inputData;
    inputData.videoEnd = true;
    while (1)
    {
        if (ModelInputDataQueue.Push(inputData))
        {
            break;
        }
        usleep(100);
    }

    aclError ret;
    if (PreProcessStream_ != nullptr)
    {
        ret = aclrtDestroyStream(PreProcessStream_);
        if (ret != ACL_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "destroy PreProcessStream_ failed");
        }
        PreProcessStream_ = nullptr;
    }
    if (PreProcessContext_ != nullptr)
    {
        ret = aclrtDestroyContext(PreProcessContext_);
        if (ret != ACL_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "destroy PreProcessContext_ failed");
        }
        PreProcessContext_ = nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "end to destroy PreProcessStream_ PreProcessContext_");
}

void Face_recognition::yolov5Handle()
{
    aclError aclRet = aclrtSetCurrentContext(Yolov5Context_);
    CHECK_RET(aclRet == ACL_SUCCESS, LOG_PRINT("[ERROR] aclrtSetCurrentContext failed. ERROR: %d", aclRet); return);
    // create stream
    aclrtCreateStream(&Yolov5Stream_);
    RCLCPP_INFO(this->get_logger(), "create Yolov5Stream_ success");

    acllite::ModelProc modelProcess;
    bool ret = modelProcess.Load(modelPath);
    CHECK_RET(ret, LOG_PRINT("[ERROR] load model %s failed.", modelPath.c_str()); return);
    while (1)
    {
        if (!ModelInputDataQueue.Empty())
        {
            ModelInputData inputData = ModelInputDataQueue.Pop();
            if (inputData.videoEnd)
            {
                break;
            }
            else
            {
                ret = modelProcess.CreateInput(static_cast<void *>(inputData.data.get()), inputData.size);
                CHECK_RET(ret, LOG_PRINT("[ERROR] Create model input failed."); return);
                ModelOnputData outputData;
                // outputData.srcImg = inputData.srcImg;
                outputData.videoEnd = inputData.videoEnd;
                modelProcess.Execute(outputData.inferOutputs);
                CHECK_RET(ret, LOG_PRINT("[ERROR] model execute failed."); break);
                while (1)
                {
                    if (ModelOnputDataQueue.Push(outputData))
                    {
                        break;
                    }
                    usleep(10);
                }
            }
        }
        else
        {
            usleep(5);
        }
    }
    modelProcess.DestroyResource();
    ModelOnputData outputData;
    outputData.videoEnd = true;
    while (1)
    {
        if (ModelOnputDataQueue.Push(outputData))
        {
            break;
        }
        usleep(100);
    }

    aclError aclret;
    if (Yolov5Stream_ != nullptr)
    {
        aclret = aclrtDestroyStream(Yolov5Stream_);
        if (aclret != ACL_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "destroy Yolov5Stream_ failed");
        }
        Yolov5Stream_ = nullptr;
    }
    if (Yolov5Context_ != nullptr)
    {
        aclret = aclrtDestroyContext(Yolov5Context_);
        if (aclret != ACL_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "destroy Yolov5Context_ failed");
        }
        Yolov5Context_ = nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "end to destroy Yolov5Stream_ Yolov5Context_");
}

void Face_recognition::getResult(std::vector<acllite::InferenceOutput> &inferOutputs, uint32_t src_width, uint32_t src_hight, uint32_t model_width, uint32_t model_height)
{
    uint32_t outputDataBufId = 0;
    float *classBuff = static_cast<float *>(inferOutputs[outputDataBufId].data.get());
    float confidenceThreshold = 0.25;
    size_t classNum = 1;
    size_t offset = 5;
    size_t totalNumber = classNum + offset;
    size_t modelOutputBoxNum = 25200;
    size_t startIndex = 5;

    // 遍历模型输出的每一个边界框，计算每个类别的置信度，并找到置信度最大的类别。如果这个最大的类别置信度大于预设的阈值，并且类别索引小于类别数量，那么就将这个边界框添加到boxes向量中
    vector<BoundBox> boxes;
    size_t yIndex = 1;
    size_t widthIndex = 2;
    size_t heightIndex = 3;
    size_t classConfidenceIndex = 4;
    float widthScale = (float)(src_width) / model_width;
    float heightScale = (float)(src_hight) / model_height;
    float finalScale = (widthScale > heightScale) ? widthScale : heightScale;
    for (size_t i = 0; i < modelOutputBoxNum; ++i)
    {
        float maxValue = 0;
        float maxIndex = 0;
        for (size_t j = startIndex; j < totalNumber; ++j)
        {
            float value = classBuff[i * totalNumber + j] * classBuff[i * totalNumber + classConfidenceIndex];
            if (value > maxValue)
            {
                maxIndex = j - startIndex;
                maxValue = value;
            }
        }
        float classConfidence = classBuff[i * totalNumber + classConfidenceIndex];
        if (classConfidence >= confidenceThreshold)
        {
            size_t index = i * totalNumber + maxIndex + startIndex;
            float finalConfidence = classConfidence * classBuff[index];
            BoundBox box;
            box.x = classBuff[i * totalNumber] * finalScale;
            box.y = classBuff[i * totalNumber + yIndex] * finalScale;
            box.width = classBuff[i * totalNumber + widthIndex] * finalScale;
            box.height = classBuff[i * totalNumber + heightIndex] * finalScale;
            box.score = finalConfidence;
            box.classIndex = maxIndex;
            box.index = i;
            if (maxIndex < classNum)
            {
                boxes.push_back(box);
            }
        }
    }

    // 使用非极大值抑制消除多余的检测框
    vector<BoundBox> result;
    result.clear();
    float NMSThreshold = 0.45;
    int32_t maxLength = model_width > model_height ? model_width : model_height;
    // 将boxes向量按照分数进行排序，分数高的边界框排在前面
    std::sort(boxes.begin(), boxes.end(), sortScore);
    BoundBox boxMax;
    BoundBox boxCompare;
    while (boxes.size() != 0)
    {
        size_t index = 1;
        // 当前最高分的边界框添加到result向量中。
        result.push_back(boxes[0]);
        while (boxes.size() > index)
        {
            // 计算boxMax的信息。
            boxMax.score = boxes[0].score;
            boxMax.classIndex = boxes[0].classIndex;
            boxMax.index = boxes[0].index;
            boxMax.x = boxes[0].x + maxLength * boxes[0].classIndex;
            boxMax.y = boxes[0].y + maxLength * boxes[0].classIndex;
            boxMax.width = boxes[0].width;
            boxMax.height = boxes[0].height;

            // 计算boxCompare的信息。
            boxCompare.score = boxes[index].score;
            boxCompare.classIndex = boxes[index].classIndex;
            boxCompare.index = boxes[index].index;
            boxCompare.x = boxes[index].x + boxes[index].classIndex * maxLength;
            boxCompare.y = boxes[index].y + boxes[index].classIndex * maxLength;
            boxCompare.width = boxes[index].width;
            boxCompare.height = boxes[index].height;

            // 计算boxMax和boxCompare的交集区域的坐标和尺寸。
            float xLeft = max(boxMax.x, boxCompare.x);
            float yTop = max(boxMax.y, boxCompare.y);
            float xRight = min(boxMax.x + boxMax.width, boxCompare.x + boxCompare.width);
            float yBottom = min(boxMax.y + boxMax.height, boxCompare.y + boxCompare.height);
            float width = max(0.0f, xRight - xLeft);
            float hight = max(0.0f, yBottom - yTop);

            // 计算交集区域的面积和IoU。
            float area = width * hight;
            float iou = area / (boxMax.width * boxMax.height + boxCompare.width * boxCompare.height - area);
            if (iou > NMSThreshold) // 如果IoU大于阈值，那么就将boxCompare从boxes向量中删除。
            {
                boxes.erase(boxes.begin() + index);
                continue;
            }
            ++index;
        }
        boxes.erase(boxes.begin()); // 将处理过的最高分边界框从boxes向量中删除。
    }

    int half = 2;
    for (size_t i = 0; i < result.size(); ++i)
    {
        if (result[i].score < 0.7)
        {
            continue;
        }

        // int leftUpX = result[i].x - result[i].width / half;
        // int leftUpY = result[i].y - result[i].height / half;
        // int rightBottomX = result[i].x + result[i].width / half;
        // int rightBottomY = result[i].y + result[i].height / half;

        my_interfaces::msg::DetectionBox faceBox;
        faceBox.bounding_box.x = result[i].x - result[i].width / half;
        faceBox.bounding_box.y = result[i].y - result[i].height / half;
        faceBox.bounding_box.width = result[i].width;
        faceBox.bounding_box.height = result[i].height;
        faceBox.confidence = result[i].score;
        faceBox.index = result[i].classIndex;
        faceRecognition_publisher_->publish(faceBox);
    }
    return;
}

auto prevTime = std::chrono::steady_clock::now();
auto curTime = std::chrono::steady_clock::now();
void Face_recognition::getResult(std::vector<acllite::InferenceOutput> &inferOutputs, cv::Mat &srcImage, uint32_t model_width, uint32_t model_height)
{
    uint32_t outputDataBufId = 0;
    float *classBuff = static_cast<float *>(inferOutputs[outputDataBufId].data.get());
    float confidenceThreshold = 0.25;
    size_t classNum = 1;
    size_t offset = 5;
    size_t totalNumber = classNum + offset;
    size_t modelOutputBoxNum = 25200;
    size_t startIndex = 5;
    int srcWidth = srcImage.cols;
    int srcHeight = srcImage.rows;

    // 遍历模型输出的每一个边界框，计算每个类别的置信度，并找到置信度最大的类别。如果这个最大的类别置信度大于预设的阈值，并且类别索引小于类别数量，那么就将这个边界框添加到boxes向量中
    vector<BoundBox> boxes;
    size_t yIndex = 1;
    size_t widthIndex = 2;
    size_t heightIndex = 3;
    size_t classConfidenceIndex = 4;
    float widthScale = (float)(srcWidth) / model_width;
    float heightScale = (float)(srcHeight) / model_height;
    float finalScale = (widthScale > heightScale) ? widthScale : heightScale;
    for (size_t i = 0; i < modelOutputBoxNum; ++i)
    {
        float maxValue = 0;
        float maxIndex = 0;
        for (size_t j = startIndex; j < totalNumber; ++j)
        {
            float value = classBuff[i * totalNumber + j] * classBuff[i * totalNumber + classConfidenceIndex];
            if (value > maxValue)
            {
                maxIndex = j - startIndex;
                maxValue = value;
            }
        }
        float classConfidence = classBuff[i * totalNumber + classConfidenceIndex];
        if (classConfidence >= confidenceThreshold)
        {
            size_t index = i * totalNumber + maxIndex + startIndex;
            float finalConfidence = classConfidence * classBuff[index];
            BoundBox box;
            box.x = classBuff[i * totalNumber] * finalScale;
            box.y = classBuff[i * totalNumber + yIndex] * finalScale;
            box.width = classBuff[i * totalNumber + widthIndex] * finalScale;
            box.height = classBuff[i * totalNumber + heightIndex] * finalScale;
            box.score = finalConfidence;
            box.classIndex = maxIndex;
            box.index = i;
            if (maxIndex < classNum)
            {
                boxes.push_back(box);
            }
        }
    }

    // 使用非极大值抑制消除多余的检测框
    vector<BoundBox> result;
    result.clear();
    float NMSThreshold = 0.45;
    int32_t maxLength = model_width > model_height ? model_width : model_height;
    // 将boxes向量按照分数进行排序，分数高的边界框排在前面
    std::sort(boxes.begin(), boxes.end(), sortScore);
    BoundBox boxMax;
    BoundBox boxCompare;
    while (boxes.size() != 0)
    {
        size_t index = 1;
        // 当前最高分的边界框添加到result向量中。
        result.push_back(boxes[0]);
        while (boxes.size() > index)
        {
            // 计算boxMax的信息。
            boxMax.score = boxes[0].score;
            boxMax.classIndex = boxes[0].classIndex;
            boxMax.index = boxes[0].index;
            boxMax.x = boxes[0].x + maxLength * boxes[0].classIndex;
            boxMax.y = boxes[0].y + maxLength * boxes[0].classIndex;
            boxMax.width = boxes[0].width;
            boxMax.height = boxes[0].height;

            // 计算boxCompare的信息。
            boxCompare.score = boxes[index].score;
            boxCompare.classIndex = boxes[index].classIndex;
            boxCompare.index = boxes[index].index;
            boxCompare.x = boxes[index].x + boxes[index].classIndex * maxLength;
            boxCompare.y = boxes[index].y + boxes[index].classIndex * maxLength;
            boxCompare.width = boxes[index].width;
            boxCompare.height = boxes[index].height;

            // 计算boxMax和boxCompare的交集区域的坐标和尺寸。
            float xLeft = max(boxMax.x, boxCompare.x);
            float yTop = max(boxMax.y, boxCompare.y);
            float xRight = min(boxMax.x + boxMax.width, boxCompare.x + boxCompare.width);
            float yBottom = min(boxMax.y + boxMax.height, boxCompare.y + boxCompare.height);
            float width = max(0.0f, xRight - xLeft);
            float hight = max(0.0f, yBottom - yTop);

            // 计算交集区域的面积和IoU。
            float area = width * hight;
            float iou = area / (boxMax.width * boxMax.height + boxCompare.width * boxCompare.height - area);
            if (iou > NMSThreshold) // 如果IoU大于阈值，那么就将boxCompare从boxes向量中删除。
            {
                boxes.erase(boxes.begin() + index);
                continue;
            }
            ++index;
        }
        boxes.erase(boxes.begin()); // 将处理过的最高分边界框从boxes向量中删除。
    }

    const double fountScale = 0.5;
    const uint32_t lineSolid = 2;
    const uint32_t labelOffset = 11;
    const cv::Scalar fountColor(0, 0, 255);
    const vector<cv::Scalar> colors{
        cv::Scalar(237, 149, 100), cv::Scalar(0, 215, 255),
        cv::Scalar(50, 205, 50), cv::Scalar(139, 85, 26)};

    int half = 2;
    for (size_t i = 0; i < result.size(); ++i)
    {
        if (result[i].score < 0.7)
        {
            continue;
        }
        cv::Point leftUpPoint, rightBottomPoint;
        leftUpPoint.x = result[i].x - result[i].width / half;
        leftUpPoint.y = result[i].y - result[i].height / half;
        rightBottomPoint.x = result[i].x + result[i].width / half;
        rightBottomPoint.y = result[i].y + result[i].height / half;
        cv::rectangle(srcImage, leftUpPoint, rightBottomPoint, colors[i % colors.size()], lineSolid);
        string className = label[result[i].classIndex];
        string markString = to_string(result[i].score) + ":" + className;
        cv::putText(srcImage, markString, cv::Point(leftUpPoint.x, leftUpPoint.y + labelOffset),
                    cv::FONT_HERSHEY_COMPLEX, fountScale, fountColor);
    }

    curTime = std::chrono::steady_clock::now();
    double duration = std::chrono::duration<double>(curTime - prevTime).count();
    double fps = 1.0 / duration;
    std::cout << "duration: " << duration << std::endl;
    std::cout << "FPS: " << fps << std::endl;
    prevTime = curTime;
    cv::imshow("usb-show-demo", srcImage);
    cv::waitKey(1);
    return;
}

void Face_recognition::resultHandle()
{
    aclError aclRet = aclrtSetCurrentContext(ResultHandleContext_);
    CHECK_RET(aclRet == ACL_SUCCESS, LOG_PRINT("[ERROR] aclrtSetCurrentContext failed. ERROR: %d", aclRet); return);
    // create stream
    aclrtCreateStream(&ResultHandleStream_);
    RCLCPP_INFO(this->get_logger(), "create ResultHandleStream_ success");
    // JpegeInit(100);

    while (1)
    {
        if (!ModelOnputDataQueue.Empty())
        {
            ModelOnputData outputData = ModelOnputDataQueue.Pop();
            if (outputData.videoEnd)
            {
                break;
            }
            getResult(outputData.inferOutputs, cameraImage_width, cameraImage_height, modelWidth, modelHeight);
            // getResult(outputData.inferOutputs, outputData.srcImg, modelWidth, modelHeight);
        }
        else
        {
            usleep(10);
        }
    }
    // DestroyJpegeResource();

    if (ResultHandleStream_ != nullptr)
    {
        aclRet = aclrtDestroyStream(ResultHandleStream_);
        if (aclRet != ACL_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "destroy ResultHandleStream_ failed");
        }
        ResultHandleStream_ = nullptr;
    }
    if (ResultHandleContext_ != nullptr)
    {
        aclRet = aclrtDestroyContext(ResultHandleContext_);
        if (aclRet != ACL_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "destroy ResultHandleContext_ failed");
        }
        ResultHandleContext_ = nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "end to destroy ResultHandleStream_ ResultHandleContext_");
}

Face_recognition::Face_recognition(const rclcpp::NodeOptions &options) : Node("face_recognition", options)
{
    signal(SIGINT, handle_sigint);

    device_path = "/dev/video0";
    aclConfig_path = "/root/my_robot/dev_ws/config/face_recognition/acl.json";
    modelPath = "/root/my_robot/dev_ws/config/face_recognition/face.om";

    SharedMemoryPtr_cvimage->header.frame_id = "camera_link";
    SharedMemoryPtr_cvimage->encoding = "8UC1";

    // 1、acl初始化
    aclInit(aclConfig_path);
    RCLCPP_INFO(this->get_logger(), "acl init success");
    // 2.运行管理资源申请create Device,Context,Stream
    aclrtSetDevice(deviceId_);
    RCLCPP_INFO(this->get_logger(), "open device %d success", deviceId_);
    // create context (set current)
    aclrtCreateContext(&PreProcessContext_, deviceId_);
    aclrtCreateContext(&Yolov5Context_, deviceId_);
    aclrtCreateContext(&ResultHandleContext_, deviceId_);

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    ImagePublisher_ZeroCopy_ = this->create_publisher<imageTopic>("camera_image", qos);
    faceRecognition_publisher_ = this->create_publisher<my_interfaces::msg::DetectionBox>("yolov5Result_Box", 10);

    preProcessImage_thread_ = std::shared_ptr<std::thread>(
        new std::thread(std::bind(&Face_recognition::GetPreProcessImage, this)));
    yolov5Handle_thread_ = std::shared_ptr<std::thread>(
        new std::thread(std::bind(&Face_recognition::yolov5Handle, this)));
    resultHandle_thread_ = std::shared_ptr<std::thread>(
        new std::thread(std::bind(&Face_recognition::resultHandle, this)));
}

Face_recognition::~Face_recognition()
{
    if (preProcessImage_thread_->joinable())
    {
        preProcessImage_thread_->join();
    }
    if (yolov5Handle_thread_->joinable())
    {
        yolov5Handle_thread_->join();
    }
    if (resultHandle_thread_->joinable())
    {
        resultHandle_thread_->join();
    }
    DestroyResource();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<Face_recognition>(options));
    rclcpp::shutdown();
    return 0;
}
