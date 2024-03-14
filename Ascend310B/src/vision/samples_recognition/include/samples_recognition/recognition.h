#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "acllite_dvpp_lite/ImageProc.h"
#include "acllite_om_execute/ModelProc.h"
#include "acllite_media/CameraRead.h"
#include "acllite_common/Queue.h"
#include "label.h"

using namespace std;
using namespace acllite;
using namespace cv;

struct MsgData
{
    std::shared_ptr<uint8_t> data = nullptr;
    uint32_t size = 0;
    bool videoEnd = false;
    cv::Mat srcImg;
};

struct MsgOut
{
    cv::Mat srcImg;
    bool videoEnd = false;
    vector<InferenceOutput> inferOutputs;
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

class Yolov5s_recongnition : public rclcpp::Node
{
public:
    explicit Yolov5s_recongnition(const rclcpp::NodeOptions &options);
    ~Yolov5s_recongnition();

private:
    int32_t deviceId;
    AclLiteResource aclResource;
};