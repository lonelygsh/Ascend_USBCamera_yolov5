#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "shm_msgs/msg/image.hpp"
#include "shm_msgs/opencv_conversions.hpp"
// #include <nadjieb/mjpeg_streamer.hpp>
#include "acllite_common/Common.h"
#include "acl/ops/acl_dvpp.h"
#include "my_interfaces/msg/detection_box.hpp"

extern "C"
{
#include "socket/FFmpegVideoDecoder.h"
}
using namespace shm_msgs;
// using MJPEGStreamer = nadjieb::MJPEGStreamer;
using Topic = shm_msgs::msg::Image4m;

#define cameraImage_width 1280
#define cameraImage_height 720
#define vencInputSize 1382400

class Socket2PC : public rclcpp::Node
{
public:
    explicit Socket2PC(const rclcpp::NodeOptions &options);
    ~Socket2PC();

private:
    void imageCallback(const Topic::SharedPtr msg);
    void sendImage(const cv::Mat &image);
    rclcpp::Subscription<Topic>::SharedPtr ImageSubscription_ZeroCopy;
    shm_msgs::CvImageConstPtr SharedMemoryPtr_cvimage;
    rclcpp::Subscription<my_interfaces::msg::DetectionBox>::SharedPtr face_subscript_;
    void FaceDection_callback(my_interfaces::msg::DetectionBox::SharedPtr msg);
    // MJPEGStreamer streamer;

    // 编码回调函数线程
    std::shared_ptr<std::thread> vencCallback_thread_;
    void vencCallback();
    bool VencInit(uint64_t threadId);
    void DestroyVencResource();
    bool SetVencInputPicDescYUV420P();
    bool SetFrameConfig(uint8_t eos, uint8_t forceIFrame);
    bool DoVencProcess(void *inBufferDev, uint32_t inBufferSize);

    const char *aclConfig_path;
    int32_t deviceId_ = 0;
    aclrtContext vencContext_ = nullptr;
    aclrtStream vencStream_ = nullptr;
    aclrtContext vencCallbackContext_ = nullptr;
    // venc编码的相关变量
    aclvencChannelDesc *vencChannelDesc_;
    aclvencFrameConfig *vencFrameConfig_;
    acldvppPicDesc *vencInputPicputDesc_;
    void *vencInBufferDev_;
    uint32_t vencInBufferSize_;
    uint32_t vencWidthStride_;
    uint32_t vencHeightStride_;
    uint64_t venc_threadId_;

    // socket通信相关
    struct sockaddr_in serv_addr;
    int sock = 0;
};
