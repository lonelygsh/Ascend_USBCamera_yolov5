#include "samples_recognition/recognition.h"

// v4l2-ctl --list-formats-ext
// v4l2-ctl --set-fmt-video=width=1280,height=720,pixelformat=1

aclrtContext context = nullptr;
uint32_t modelWidth = 640;
uint32_t modelHeight = 640;
Queue<MsgData> msgDataQueue(32);
Queue<MsgOut> msgOutQueue(32);
bool exitFlag = false;

bool sortScore(BoundBox box1, BoundBox box2)
{
    return box1.score > box2.score;
}

void GetResult(std::vector<InferenceOutput> &inferOutputs,
               cv::Mat &srcImage, uint32_t modelWidth, uint32_t modelHeight)
{
    uint32_t outputDataBufId = 0;
    float *classBuff = static_cast<float *>(inferOutputs[outputDataBufId].data.get());
    float confidenceThreshold = 0.25;
    size_t classNum = 80;
    size_t offset = 5;
    size_t totalNumber = classNum + offset;
    size_t modelOutputBoxNum = 25200;
    size_t startIndex = 5;
    int srcWidth = srcImage.cols;
    int srcHeight = srcImage.rows;

    vector<BoundBox> boxes;
    size_t yIndex = 1;
    size_t widthIndex = 2;
    size_t heightIndex = 3;
    size_t classConfidenceIndex = 4;
    float widthScale = (float)(srcWidth) / modelWidth;
    float heightScale = (float)(srcHeight) / modelHeight;
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
    vector<BoundBox> result;
    result.clear();
    float NMSThreshold = 0.45;
    int32_t maxLength = modelWidth > modelHeight ? modelWidth : modelHeight;
    std::sort(boxes.begin(), boxes.end(), sortScore);
    BoundBox boxMax;
    BoundBox boxCompare;
    while (boxes.size() != 0)
    {
        size_t index = 1;
        result.push_back(boxes[0]);
        while (boxes.size() > index)
        {
            boxMax.score = boxes[0].score;
            boxMax.classIndex = boxes[0].classIndex;
            boxMax.index = boxes[0].index;
            boxMax.x = boxes[0].x + maxLength * boxes[0].classIndex;
            boxMax.y = boxes[0].y + maxLength * boxes[0].classIndex;
            boxMax.width = boxes[0].width;
            boxMax.height = boxes[0].height;

            boxCompare.score = boxes[index].score;
            boxCompare.classIndex = boxes[index].classIndex;
            boxCompare.index = boxes[index].index;
            boxCompare.x = boxes[index].x + boxes[index].classIndex * maxLength;
            boxCompare.y = boxes[index].y + boxes[index].classIndex * maxLength;
            boxCompare.width = boxes[index].width;
            boxCompare.height = boxes[index].height;
            float xLeft = max(boxMax.x, boxCompare.x);
            float yTop = max(boxMax.y, boxCompare.y);
            float xRight = min(boxMax.x + boxMax.width, boxCompare.x + boxCompare.width);
            float yBottom = min(boxMax.y + boxMax.height, boxCompare.y + boxCompare.height);
            float width = max(0.0f, xRight - xLeft);
            float hight = max(0.0f, yBottom - yTop);
            float area = width * hight;
            float iou = area / (boxMax.width * boxMax.height + boxCompare.width * boxCompare.height - area);
            if (iou > NMSThreshold)
            {
                boxes.erase(boxes.begin() + index);
                continue;
            }
            ++index;
        }
        boxes.erase(boxes.begin());
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
    printf("show\r\n");
    cv::imshow("usb-show-demo", srcImage);
    cv::waitKey(1);
    return;
}

void *GetInput(void *arg)
{
    bool ret = SetCurContext(context);
    CHECK_RET(ret, LOG_PRINT("[ERROR] set cur context for pthread  %ld failed.", pthread_self()); return NULL);
    int32_t deviceId = *(int32_t *)arg;
    string devPath = "/dev/video0";
    CameraRead cap(devPath, deviceId);
    CHECK_RET(cap.IsOpened(), LOG_PRINT("[ERROR] open %s failed.", devPath.c_str()); return NULL);
    ImageProc imageProcess;
    ImageData frame;
    ImageSize modelSize(modelWidth, modelHeight);
    LOG_PRINT("[INFO] start to decode...");
    int failCount = 0;
    // auto prevTime = std::chrono::steady_clock::now();
    while (1)
    {
        ret = cap.Read(frame);

        // // 显示frame图像
        // cv::Mat frameImg(frame.height, frame.width, CV_8UC2);
        // memcpy(frameImg.data, (unsigned char *)frame.data.get(), frame.size);
        // cv::cvtColor(frameImg, frameImg, cv::COLOR_YUV2BGR_YUYV);
        // cv::imshow("Frame Image", frameImg);
        // 初始化时间戳
        if (ret)
        {
            // // 获取当前时间戳
            // auto curTime = std::chrono::steady_clock::now();
            // // 计算时间差，单位为秒
            // double duration = std::chrono::duration<double>(curTime - prevTime).count();
            // // 计算帧率
            // double fps = 1.0 / duration;
            // std::cout << "duration: " << duration << std::endl;
            // // 输出帧率
            // std::cout << "FPS: " << fps << std::endl;
            // // 更新时间戳
            // prevTime = curTime;

            ImageData dst;
            imageProcess.Resize(frame, dst, modelSize, RESIZE_PROPORTIONAL_UPPER_LEFT);
            MsgData msgData;
            msgData.data = dst.data;
            msgData.size = dst.size;
            msgData.videoEnd = false;
            cv::Mat yuyvImg(frame.height, frame.width, CV_8UC2);
            memcpy(yuyvImg.data, (unsigned char *)frame.data.get(), frame.size);
            cv::cvtColor(yuyvImg, msgData.srcImg, cv::COLOR_YUV2BGR_YUYV);

            while (1)
            {
                if (msgDataQueue.Push(msgData))
                {
                    break;
                }
                usleep(100);
            }
        }
        else
        {
            LOG_PRINT("[INFO] frame read end.");
            break;
        }
        // 等待按键，然后关闭窗口
        // cv::waitKey(1);
    }
    // cv::destroyAllWindows();
    cap.Release();
    // 发送视频结束消息
    MsgData msgData;
    msgData.videoEnd = true;
    while (1)
    {
        if (msgDataQueue.Push(msgData))
        {
            break;
        }
        usleep(100);
    }
    LOG_PRINT("[INFO] preprocess add end msgData. tid : %ld", pthread_self());
    return NULL;
}

void *ModelExecute(void *arg)
{
    bool ret = SetCurContext(context);
    CHECK_RET(ret, LOG_PRINT("[ERROR] set cur context for pthread  %ld failed.", pthread_self()); return NULL);
    ModelProc modelProcess;
    // string modelPath = "./config/sample_recognition/yolov5s.om";
    string modelPath = "/root/my_robot/dev_ws/config/sample_recognition/yolov5s.om";
    ret = modelProcess.Load(modelPath);
    CHECK_RET(ret, LOG_PRINT("[ERROR] load model %s failed.", modelPath.c_str()); return NULL);
    while (1)
    {
        if (!msgDataQueue.Empty())
        {
            MsgData msgData = msgDataQueue.Pop();
            if (msgData.videoEnd)
            {
                break;
            }
            else
            {
                ret = modelProcess.CreateInput(static_cast<void *>(msgData.data.get()), msgData.size);
                CHECK_RET(ret, LOG_PRINT("[ERROR] Create model input failed."); break);
                MsgOut msgOut;
                msgOut.srcImg = msgData.srcImg;
                msgOut.videoEnd = msgData.videoEnd;
                modelProcess.Execute(msgOut.inferOutputs);
                CHECK_RET(ret, LOG_PRINT("[ERROR] model execute failed."); break);
                while (1)
                {
                    if (msgOutQueue.Push(msgOut))
                    {
                        break;
                    }
                    usleep(100);
                }
            }
        }
    }
    modelProcess.DestroyResource();
    MsgOut msgOut;
    msgOut.videoEnd = true;
    while (1)
    {
        if (msgOutQueue.Push(msgOut))
        {
            break;
        }
        usleep(100);
    }
    LOG_PRINT("[INFO] infer msg end. tid : %ld", pthread_self());
    return NULL;
}

void *PostProcess(void *arg)
{
    while (1)
    {
        if (!msgOutQueue.Empty())
        {
            MsgOut msgOut = msgOutQueue.Pop();
            usleep(100);
            if (msgOut.videoEnd)
            {
                break;
            }
            GetResult(msgOut.inferOutputs, msgOut.srcImg, modelWidth, modelHeight);
        }
    }
    LOG_PRINT("[INFO] *************** all get done ***************");
    exitFlag = true;
    return NULL;
}

Yolov5s_recongnition::Yolov5s_recongnition(const rclcpp::NodeOptions &options) : Node("Yolov5s_recongnition", options), deviceId(0), aclResource(deviceId)
{

    bool ret = aclResource.Init();
    CHECK_RET(ret, LOG_PRINT("[ERROR] InitACLResource failed."); exit(1));
    context = aclResource.GetContext();

    pthread_t preTids, exeTids, posTids;
    pthread_create(&preTids, NULL, GetInput, (void *)&deviceId);
    pthread_create(&exeTids, NULL, ModelExecute, NULL);
    pthread_create(&posTids, NULL, PostProcess, NULL);

    pthread_detach(preTids);
    pthread_detach(exeTids);
    pthread_detach(posTids);

    while (!exitFlag)
    {
        sleep(10);
    }
}

Yolov5s_recongnition::~Yolov5s_recongnition()
{
    aclResource.Release();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<Yolov5s_recongnition>(options));
    rclcpp::shutdown();

    return 0;
}
