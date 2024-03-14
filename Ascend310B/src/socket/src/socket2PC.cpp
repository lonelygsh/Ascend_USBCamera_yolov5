#include "socket/socket2PC.h"

static bool vencRunFlag = true;

Socket2PC::Socket2PC(const rclcpp::NodeOptions &options) : Node("video_stream_node", options)
{
    // streamer.start(8080);
    aclConfig_path = "/root/my_robot/dev_ws/config/face_recognition/acl.json";
    // 1、acl初始化
    aclInit(aclConfig_path);
    RCLCPP_INFO(this->get_logger(), "acl init success");
    // 2.运行管理资源申请create Device,Context,Stream
    aclrtSetDevice(deviceId_);
    RCLCPP_INFO(this->get_logger(), "open device %d success", deviceId_);
    // create context (set current)
    aclrtCreateContext(&vencContext_, deviceId_);
    aclrtCreateContext(&vencCallbackContext_, deviceId_);
    aclError aclRet = aclrtSetCurrentContext(vencContext_);
    CHECK_RET(aclRet == ACL_SUCCESS, LOG_PRINT("[ERROR] aclrtSetCurrentContext failed. ERROR: %d", aclRet); return);
    aclrtCreateStream(&vencStream_);
    RCLCPP_INFO(this->get_logger(), "create VencStream_ success");

    vencCallback_thread_ = std::shared_ptr<std::thread>(
        new std::thread(std::bind(&Socket2PC::vencCallback, this)));
    std::ostringstream oss;
    oss << vencCallback_thread_->get_id();
    uint64_t tid = std::stoull(oss.str());
    RCLCPP_INFO(this->get_logger(), "envc callback thread start threadId = %lu", tid);
    bool ret = VencInit(tid);
    if (ret == false)
    {
        RCLCPP_INFO(this->get_logger(), "envc init fail");
    }

    int initResult = FFmpeg_socketInit();
    if (initResult < 0)
    {
        printf("FFmpeg init fail!!!!\r\n");
    }
    // int initResult = FFmpeg_H264DecoderInit();
    // if (initResult < 0)
    // {
    //     printf("FFmpeg init fail!!!!\r\n");
    // }

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(8888);

    // 将IP地址从文本转换为二进制形式
    if (inet_pton(AF_INET, PC_IP, &serv_addr.sin_addr) <= 0)
    {
        printf("\nInvalid address/ Address not supported \n");
        return;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\nConnection Failed \n");
        return;
    }

    face_subscript_ = this->create_subscription<my_interfaces::msg::DetectionBox>("/yolov5Result_Box", 10, std::bind(&Socket2PC::FaceDection_callback, this, std::placeholders::_1));
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    ImageSubscription_ZeroCopy = create_subscription<Topic>("camera_image", qos,
                                                            std::bind(&Socket2PC::imageCallback, this, std::placeholders::_1));
}

Socket2PC::~Socket2PC()
{
    close(sock);
    FFmpeg_socketRelease();
    // int releaseResult = FFmpeg_VideoDecoderRelease();
    // if (releaseResult < 0)
    // {
    //     printf("FFmpeg release fail!!!!\r\n");
    // }
    vencRunFlag = false;
    aclError aclRet;
    DestroyVencResource();
    if (vencStream_ != nullptr)
    {
        aclRet = aclrtDestroyStream(vencStream_);
        if (aclRet != ACL_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "destroy vencStream_ failed");
        }
        vencStream_ = nullptr;
    }
    if (vencContext_ != nullptr)
    {
        aclRet = aclrtDestroyContext(vencContext_);
        if (aclRet != ACL_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "destroy vencContext_ failed");
        }
        vencContext_ = nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "end to destroy vencStream_ vencContext_");
    if (vencCallback_thread_->joinable())
    {
        vencCallback_thread_->join();
    }
    // streamer.stop();
}

void Socket2PC::FaceDection_callback(my_interfaces::msg::DetectionBox::SharedPtr msg)
{
    char buffer[sizeof(*msg)];
    memcpy(buffer, msg.get(), sizeof(*msg));
    send(sock, buffer, sizeof(buffer), 0);
}

void Socket2PC::imageCallback(const Topic::SharedPtr msg)
{
    SharedMemoryPtr_cvimage = shm_msgs::toCvShare(msg);
    void *inBufferDev = SharedMemoryPtr_cvimage->image.data;
    uint32_t inBufferSize = SharedMemoryPtr_cvimage->image.total() * SharedMemoryPtr_cvimage->image.elemSize();
    DoVencProcess(inBufferDev, inBufferSize);

    // cv::Mat bgrImg;
    // cv::cvtColor(SharedMemoryPtr_cvimage->image, bgrImg, cv::COLOR_YUV2BGR_NV12);
    // cv::imshow("imshow", bgrImg);
    // cv::waitKey(1);

    // cv::cvtColor(yuvImg, inputData.srcImg, cv::COLOR_YUV2BGR_NV12);
    // streamer.publish("/jpg", std::string(mjpg_data.begin(), mjpg_data.end()));
}

void callback(acldvppPicDesc *input, acldvppStreamDesc *outputStreamDesc, void *userdata)
{
    if (outputStreamDesc == nullptr)
    {
        printf("output is null\r\n");
        return;
    }
    void *outputDev = acldvppGetStreamDescData(outputStreamDesc);
    uint32_t retCode = acldvppGetStreamDescRetCode(outputStreamDesc);
    if (retCode == 0)
    {
        uint32_t streamDescSize = acldvppGetStreamDescSize(outputStreamDesc);
        if (outputDev == nullptr)
        {
            printf("dataDev is nullptr!\r\n");
            return;
        }

        // 创建一个新的AVPacket
        AVPacket pkt;
        av_init_packet(&pkt);
        pkt.data = (uint8_t *)outputDev;
        pkt.size = streamDescSize;

        // 设置时间戳
        static int64_t pts = 0;
        pkt.pts = pts++;
        pkt.dts = pkt.pts;

        // 将AVPacket写入输出上下文
        av_interleaved_write_frame(outputContext, &pkt);

        // // 测试是否编码H264成功
        // int framePara[5];
        // unsigned char *inbuf = static_cast<unsigned char *>(outputDev);
        // unsigned char *outRGBBuf = new unsigned char[framePara[0] * framePara[1] * 3];

        // int decodeResult = FFmpeg_H264Decode(inbuf, streamDescSize, framePara, outRGBBuf, NULL);
        // if (decodeResult > 0)
        // {
        //     cv::Mat img(framePara[1], framePara[0], CV_8UC3, outRGBBuf);
        //     cv::imshow("Image", img);
        //     cv::waitKey(1);
        // }

        // delete[] outRGBBuf;
    }
    else
    {
        printf("venc encode frame failed, retCode = %u.\r\n", retCode);
    }
}

void Socket2PC::vencCallback()
{
    aclError aclRet = aclrtSetCurrentContext(vencCallbackContext_);
    CHECK_RET(aclRet == ACL_SUCCESS, LOG_PRINT("[ERROR] aclrtSetCurrentContext failed. ERROR: %d", aclRet); return);

    while (vencRunFlag)
    {
        // Notice: timeout 1000ms
        (void)aclrtProcessReport(1000);
    }

    aclError ret;
    if (vencCallbackContext_ != nullptr)
    {
        ret = aclrtDestroyContext(vencCallbackContext_);
        if (ret != ACL_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "destroy vencCallbackContext_ failed");
        }
        vencCallbackContext_ = nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "end to destroy vencCallbackContext_");
}

bool Socket2PC::VencInit(uint64_t threadId)
{
    vencWidthStride_ = ALIGN_UP2(cameraImage_width);
    vencHeightStride_ = ALIGN_UP2(cameraImage_height);
    venc_threadId_ = threadId;

    aclError ret;
    // aclrtRunMode runMode;
    // ret = aclrtGetRunMode(&runMode);
    // printf("runmode is %d\r\n");

    vencChannelDesc_ = aclvencCreateChannelDesc();
    if (vencChannelDesc_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "fail to create venc channel desc");
        return false;
    }

    // set process callback thread
    ret = aclvencSetChannelDescThreadId(vencChannelDesc_, venc_threadId_);
    if (ret != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "fail to set threadId, errorCode = %d", static_cast<int32_t>(ret));
        return false;
    }

    // set callback func
    ret = aclvencSetChannelDescCallback(vencChannelDesc_, callback);
    if (ret != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "fail to set venc Callback, errorCode = %d", static_cast<int32_t>(ret));
        return false;
    }

    // set output stream type
    ret = aclvencSetChannelDescEnType(vencChannelDesc_, H264_BASELINE_LEVEL);
    if (ret != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "fail to set venc EnType, errorCode = %d", static_cast<int32_t>(ret));
        return false;
    }

    // set input picture type
    ret = aclvencSetChannelDescPicFormat(vencChannelDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    if (ret != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "fail to set venc PicFormat, errorCode = %d", static_cast<int32_t>(ret));
        return false;
    }

    // set input picture width
    ret = aclvencSetChannelDescPicWidth(vencChannelDesc_, cameraImage_width);
    if (ret != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "fail to set venc PicWidth, errorCode = %d", static_cast<int32_t>(ret));
        return false;
    }

    // set input picture height
    ret = aclvencSetChannelDescPicHeight(vencChannelDesc_, cameraImage_height);
    if (ret != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "fail to set venc PicWidth, errorCode = %d", static_cast<int32_t>(ret));
        return false;
    }

    // set key frame interval
    ret = aclvencSetChannelDescKeyFrameInterval(vencChannelDesc_, 16);
    if (ret != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "fail to set venc FrameInterval, errorCode = %d", static_cast<int32_t>(ret));
        return false;
    }

    // create vdec channel
    ret = aclvencCreateChannel(vencChannelDesc_);
    if (ret != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "fail to create venc channel, errorCode = %d", static_cast<int32_t>(ret));
        return false;
    }

    vencFrameConfig_ = aclvencCreateFrameConfig();
    if (vencFrameConfig_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "fail to create frame config");
        return false;
    }

    return true;
}

void Socket2PC::DestroyVencResource()
{
    if (vencChannelDesc_ != nullptr)
    {
        (void)aclvencDestroyChannel(vencChannelDesc_);
        (void)aclvencDestroyChannelDesc(vencChannelDesc_);
        vencChannelDesc_ = nullptr;
    }

    if (vencInputPicputDesc_ != nullptr)
    {
        (void)acldvppDestroyPicDesc(vencInputPicputDesc_);
        vencInputPicputDesc_ = nullptr;
    }

    if (vencFrameConfig_ != nullptr)
    {
        (void)aclvencDestroyFrameConfig(vencFrameConfig_);
        vencFrameConfig_ = nullptr;
    }

    if (vencInBufferDev_ != nullptr)
    {
        (void)acldvppFree(vencInBufferDev_);
        vencInBufferDev_ = nullptr;
    }
}

#define ACL_REQUIRES_OK(expr)          \
    do                                 \
    {                                  \
        const aclError __ret = (expr); \
        if (__ret != ACL_SUCCESS)      \
        {                              \
            return false;              \
        }                              \
    } while (false)

bool Socket2PC::SetVencInputPicDescYUV420P()
{
    vencInputPicputDesc_ = acldvppCreatePicDesc();
    if (vencInputPicputDesc_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "fail to create output pic desc");
        return false;
    }
    auto ret = acldvppSetPicDescData(vencInputPicputDesc_, vencInBufferDev_);
    if (ret != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "fail to set PicDescData, errorCode = %d", static_cast<int32_t>(ret));
        return false;
    }
    ret = acldvppSetPicDescSize(vencInputPicputDesc_, vencInBufferSize_);
    if (ret != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "fail to set PicDescSize, errorCode = %d", static_cast<int32_t>(ret));
        return false;
    }
    ACL_REQUIRES_OK(acldvppSetPicDescFormat(vencInputPicputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420));
    ACL_REQUIRES_OK(acldvppSetPicDescWidth(vencInputPicputDesc_, cameraImage_width));
    ACL_REQUIRES_OK(acldvppSetPicDescHeight(vencInputPicputDesc_, cameraImage_height));
    ACL_REQUIRES_OK(acldvppSetPicDescWidthStride(vencInputPicputDesc_, vencWidthStride_));
    ACL_REQUIRES_OK(acldvppSetPicDescHeightStride(vencInputPicputDesc_, vencHeightStride_));
    return true;
}

bool Socket2PC::SetFrameConfig(uint8_t eos, uint8_t forceIFrame)
{
    // set eos
    aclError ret = aclvencSetFrameConfigEos(vencFrameConfig_, eos);
    if (ret != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "fail to set eos, errorCode = %d", static_cast<int32_t>(ret));
        return false;
    }

    ret = aclvencSetFrameConfigForceIFrame(vencFrameConfig_, forceIFrame);
    if (ret != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "fail to set venc ForceIFrame, errorCode = %d", static_cast<int32_t>(ret));
        return false;
    }

    return true;
}

int keyframe = 1;
bool Socket2PC::DoVencProcess(void *inBufferDev, uint32_t inBufferSize)
{
    vencInBufferSize_ = inBufferSize;

    aclError aclret;
    // 此时是从机模式
    //  aclrtRunMode runMode;
    //  aclret = aclrtGetRunMode(&runMode);
    //  printf("mode is %d\r\n", runMode);
    aclret = acldvppMalloc(&vencInBufferDev_, vencInBufferSize_);
    if (aclret != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "acl malloc dvpp data failed, dataSize = %u, errorCode = %d.",
                     inBufferSize, static_cast<int32_t>(aclret));
        return false;
    }
    aclret = aclrtMemcpy(vencInBufferDev_, inBufferSize, inBufferDev, inBufferSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
    if (aclret != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] aclrtMemcpy failed. ERROR: %d", aclret);
        acldvppFree(vencInBufferDev_);
        return false;
    }

    bool err = SetVencInputPicDescYUV420P();
    if (err != true)
    {
        RCLCPP_ERROR(this->get_logger(), "fail to create picture description");
        return false;
    }

    if (keyframe % 17 == 1)
    {
        err = SetFrameConfig(0, 0);
        if (err != true)
        {
            RCLCPP_ERROR(this->get_logger(), "fail to set frame config");
            return false;
        }
    }
    if (keyframe % 17 == 0)
    {
        err = SetFrameConfig(1, 0);
        if (err != true)
        {
            RCLCPP_ERROR(this->get_logger(), "fail to set frame config");
            return false;
        }
    }

    if (keyframe % 17 != 0)
    {
        aclret = aclvencSendFrame(vencChannelDesc_, vencInputPicputDesc_, nullptr, vencFrameConfig_, nullptr);
        if (aclret != ACL_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "fail to send frame, errorCode = %d", static_cast<int32_t>(aclret));
            return false;
        }
    }
    else
    {
        aclret = aclvencSendFrame(vencChannelDesc_, nullptr, nullptr, vencFrameConfig_, nullptr);
        if (aclret != ACL_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "fail to send frame, errorCode = %d", static_cast<int32_t>(aclret));
            return false;
        }
    }
    keyframe++;

    return true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<Socket2PC>(options));
    rclcpp::shutdown();
    return 0;
}
