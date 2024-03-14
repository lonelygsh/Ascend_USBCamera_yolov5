#include "rclcpp/rclcpp.hpp"
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <memory>
#include <thread>
#include <boost/asio.hpp>
#include <atomic>

#include "face_recognition/label.h"
#include "acllite_common/Common.h"
#include "acl/ops/acl_dvpp.h"
#include "acllite_common/Queue.h"
#include "acllite_om_execute/ModelProc.h"

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "shm_msgs/opencv_conversions.hpp"
#include "shm_msgs/msg/image.hpp"
#include "my_interfaces/msg/detection_box.hpp"

using namespace shm_msgs;
using namespace std;

#define cameraImage_width 1280
#define cameraImage_height 720
#define jpegdBufSize 1382400 // cameraImage_width*cameraImage_height*3/2
#define modelWidth 640
#define modelHeight 640

using namespace std;
using imageTopic = shm_msgs::msg::Image4m;

struct ModelInputData
{
    std::shared_ptr<uint8_t> data = nullptr;
    uint32_t size = 0;
    bool videoEnd = false;
    // cv::Mat srcImg;
};

struct ModelOnputData
{
    // cv::Mat srcImg;
    bool videoEnd = false;
    vector<acllite::InferenceOutput> inferOutputs;
};

struct imageBuffer
{
    std::shared_ptr<uint8_t> data;
    uint32_t imageBufferSize;
};

typedef struct BoundBox
{
    float x;
    float y;
    float width;
    float height;
    float score;
    size_t classIndex;
    size_t index;
} BoundBox;

class Face_recognition : public rclcpp::Node
{
public:
    explicit Face_recognition(const rclcpp::NodeOptions &options);
    ~Face_recognition();

private:
    rclcpp::Publisher<imageTopic>::SharedPtr ImagePublisher_ZeroCopy_;
    std::shared_ptr<shm_msgs::CvImage> SharedMemoryPtr_cvimage{std::make_shared<shm_msgs::CvImage>()};
    void populateLoanedMessage(rclcpp::LoanedMessage<imageTopic> &loanedMsg);
    rclcpp::Publisher<my_interfaces::msg::DetectionBox>::SharedPtr faceRecognition_publisher_;

    // 相机相关
    void CameraInit();
    const char *device_path;
    int camera_fd;
    void *imageBuf[4];   // 映射到用户空间的V4L2缓冲区的地址
    v4l2_buffer v4l2Buf; // 描述一个V4L2缓冲区的属性
    // dvpp相关
    const char *aclConfig_path;
    int32_t deviceId_ = 0;
    aclrtContext PreProcessContext_ = nullptr;
    aclrtStream PreProcessStream_ = nullptr;
    aclrtContext Yolov5Context_ = nullptr;
    aclrtStream Yolov5Stream_ = nullptr;
    aclrtContext ResultHandleContext_ = nullptr;
    aclrtStream ResultHandleStream_ = nullptr;
    acldvppChannelDesc *dvppChannelDesc_;

    // JPEGD解码后的图片的变量
    acldvppPicDesc *jpegdDstPicDesc_;
    void *jpegdDstBuffer_;
    uint32_t jpegdDstBufferSize_;
    uint32_t jpegdDstWidth_;
    uint32_t jpegdDstHeight_;
    uint32_t jpegdDstWidthStride_;
    uint32_t jpegdDstHeightStride_;

    // VPC改变尺寸后的图片的变量
    acldvppPicDesc *resizeDstPicDesc_;
    void *resizeDstBuffer_;
    uint32_t resizeDstBufferSize_;
    uint32_t resizeDstWidth_;
    uint32_t resizeDstHeight_;
    uint32_t resizeDstWidthStride_;
    uint32_t resizeDstHeightStride_;
    acldvppRoiConfig *cropArea_;
    acldvppRoiConfig *pasteArea_;

    // JPEGE编码相关的变量
    acldvppChannelDesc *jpegeChannelDesc_;
    acldvppJpegeConfig *jpegeConfig;
    acldvppPicDesc *jpegeSrcPicDesc_;
    void *jpegeSrcBuffer_;
    uint32_t jpegeSrcBufferSize_;
    void *jpegeDstBuffer_;
    uint32_t jpegeDstBufferSize_;
    uint32_t jpegeSrcWidth_;
    uint32_t jpegeSrcHeight_;
    uint32_t jpegeSrcWidthStride_;
    uint32_t jpegeSrcHeightStride_;
    uint32_t jpegeEncodeLevel;


    void DestroyResource();
    acllite::ImageData JpegD(void *&inputData, uint32_t size, acldvppPixelFormat dstFormat = PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    void ResizeUpperLeft(acllite::ImageData &src, acllite::ImageData &dst, acllite::ImageSize dsize);
    bool SetJpegdPicDescNV12(uint32_t srcWidth, uint32_t srcHeight, acldvppPixelFormat dstFormat);
    bool SetVpcInputPicDescYUV420SP(acllite::ImageData &src);
    bool SetVpcOutputPicDescYUV420SP(acllite::ImageSize dsize, acldvppPixelFormat format);
    bool SetVpcInputPicDescYUV422P(acllite::ImageData &src);
    void JpegeInit(uint32_t jpege_encodeLevel);
    imageBuffer Jpege(void *&inputData);
    void DestroyJpegeResource();

    // yolov5相关
    string modelPath;
    // dvpp图片预处理线程
    std::shared_ptr<std::thread> preProcessImage_thread_;
    void GetPreProcessImage();
    // 模型处理图片线程
    std::shared_ptr<std::thread> yolov5Handle_thread_;
    void yolov5Handle();
    // 模型处理图片线程
    std::shared_ptr<std::thread> resultHandle_thread_;
    void resultHandle();
    void getResult(std::vector<acllite::InferenceOutput> &inferOutputs, uint32_t src_width, uint32_t src_hight, uint32_t model_width, uint32_t model_height);
    void getResult(std::vector<acllite::InferenceOutput> &inferOutputs, cv::Mat &srcImage, uint32_t model_width, uint32_t model_height);
};