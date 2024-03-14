extern "C"
{
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <arpa/inet.h>
#include <unistd.h>

const std::string label[] = {"face"};
// const std::string label[] = {"person", "bicycle", "car", "motorbike",
//                              "aeroplane", "bus", "train", "truck", "boat",
//                              "traffic light", "fire hydrant", "stop sign", "parking meter",
//                              "bench", "bird", "cat", "dog", "horse",
//                              "sheep", "cow", "elephant", "bear", "zebra",
//                              "giraffe", "backpack", "umbrella", "handbag", "tie",
//                              "suitcase", "frisbee", "skis", "snowboard", "sports ball",
//                              "kite", "baseball bat", "baseball glove", "skateboard",
//                              "surfboard",
//                              "tennis racket", "bottle", "wine glass", "cup",
//                              "fork", "knife", "spoon", "bowl", "banana",
//                              "apple", "sandwich", "orange", "broccoli", "carrot",
//                              "hot dog", "pizza", "donut", "cake", "chair",
//                              "sofa", "potted plant", "bed", "dining table", "toilet",
//                              "TV monitor", "laptop", "mouse", "remote", "keyboard",
//                              "cell phone", "microwave", "oven", "toaster", "sink",
//                              "refrigerator", "book", "clock", "vase", "scissors",
//                              "teddy bear", "hair drier", "toothbrush"};

typedef struct yolov5Box
{
    uint16_t index;
    double confidence;
    int32_t x;
    int32_t y;
    int32_t width;
    int32_t height;
};

yolov5Box global_detection_box;
std::mutex mtx;

// 声明全局变量
AVFormatContext *inputContext = NULL;
AVCodecContext *codecContext = NULL;
struct SwsContext *swsContext = NULL;

void socket_getdata()
{
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    // 创建监听套接字,ipv4,tcp
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // 绑定ip地址和端口号
    address.sin_family = AF_INET;
    address.sin_port = htons(8888);
    address.sin_addr.s_addr = INADDR_ANY;

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    // 监听套接字
    if (listen(server_fd, 3) < 0)
    {
        perror("listen failed");
        exit(EXIT_FAILURE);
    }

    while (1)
    {
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
        {
            perror("accept failed");
            exit(EXIT_FAILURE);
        }

        // 检查是否是来自开发板端的连接请求
        if (address.sin_addr.s_addr != inet_addr("192.168.3.40"))
        {
            printf("Connection from an unknown client, closing the connection...\n");
            close(new_socket);
            continue;
        }
        yolov5Box detection_box;
        // 在这个循环中，服务器会一直接收数据，直到客户端断开连接
        while (1)
        {

            // 接收数据
            int result = read(new_socket, &detection_box, sizeof(detection_box));

            // 如果read函数返回0，那么客户端已经断开连接
            if (result == 0)
            {
                printf("Client disconnected, waiting for a new connection...\n");
                break;
            }

            mtx.lock();
            global_detection_box = detection_box;
            mtx.unlock();

            // printf("data is %d %f %d %d %d %d\r\n", detection_box.index, detection_box.confidence, detection_box.x, detection_box.y, detection_box.width, detection_box.height);
        }

        close(new_socket);
    }

    close(server_fd);
}

int main()
{
    // 创建新线程接受检测框数据
    std::thread socket_thread(socket_getdata);

    // 初始化FFmpeg
    av_register_all();
    avformat_network_init();
    // 设置超时参数
    AVDictionary *options = NULL;
    av_dict_set(&options, "probesize", "2048", 0);
    av_dict_set(&options, "max_analyze_duration", "10", 0);

    // 打开输入流
    if (avformat_open_input(&inputContext, "udp://192.168.3.40:12345", NULL, &options) != 0)
    {
        printf("Failed to open input stream.\\rs\\n");
        return -1;
    }

    // 查找流信息
    if (avformat_find_stream_info(inputContext, NULL) < 0)
    {
        printf("Failed to retrieve input stream information.\\n");
        return -1;
    }

    // 查找视频流
    int videoStreamIndex = -1;
    for (int i = 0; i < inputContext->nb_streams; i++)
    {
        if (inputContext->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
        {
            videoStreamIndex = i;
            break;
        }
    }
    if (videoStreamIndex == -1)
    {
        printf("No video stream found.\\n");
        return -1;
    }

    // 打开解码器
    AVCodecParameters *codecParameters = inputContext->streams[videoStreamIndex]->codecpar;
    AVCodec *codec = avcodec_find_decoder(codecParameters->codec_id);
    codecContext = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(codecContext, codecParameters);
    avcodec_open2(codecContext, codec, NULL);

    // 创建SwsContext
    swsContext = sws_getContext(codecContext->width, codecContext->height, codecContext->pix_fmt, codecContext->width, codecContext->height, AV_PIX_FMT_BGR24, SWS_BICUBIC, NULL, NULL, NULL);

    // 创建OpenCV窗口
    cv::namedWindow("H264 Stream", cv::WINDOW_AUTOSIZE);

    // 读取帧
    AVPacket packet;
    AVFrame *frame = av_frame_alloc();
    AVFrame *frameBGR = av_frame_alloc();
    int numBytes = av_image_get_buffer_size(AV_PIX_FMT_BGR24, codecContext->width, codecContext->height, 1);
    uint8_t *buffer = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));
    av_image_fill_arrays(frameBGR->data, frameBGR->linesize, buffer, AV_PIX_FMT_BGR24, codecContext->width, codecContext->height, 1);

    int frameCounter = 0;
    int lastFPS = 30;
    auto startTime = std::chrono::steady_clock::now();
    while (av_read_frame(inputContext, &packet) >= 0)
    {
        if (packet.stream_index == videoStreamIndex)
        {
            // 解码帧
            avcodec_send_packet(codecContext, &packet);
            avcodec_receive_frame(codecContext, frame);

            // 转换颜色空间
            sws_scale(swsContext, (uint8_t const *const *)frame->data, frame->linesize, 0, codecContext->height, frameBGR->data, frameBGR->linesize);

            // 显示帧
            cv::Mat img(frame->height, frame->width, CV_8UC3, frameBGR->data[0], frameBGR->linesize[0]);

            const double fountScale = 0.5;
            const uint32_t labelOffset = 11;
            const cv::Scalar fountColor(0, 0, 255);
            // 添加显示框
            mtx.lock();
            yolov5Box faceBox = global_detection_box;
            memset(&global_detection_box, 0, sizeof(global_detection_box));
            cv::rectangle(img, cv::Point(faceBox.x, faceBox.y), cv::Point(faceBox.x + faceBox.width, faceBox.y + faceBox.height), cv::Scalar(0, 255, 0), 2);
            std::string className = label[faceBox.index];
            std::string markString = std::to_string(faceBox.confidence) + ":" + className;
            cv::putText(img, markString, cv::Point(faceBox.x, faceBox.y + labelOffset),
                        cv::FONT_HERSHEY_COMPLEX, fountScale, fountColor);
            mtx.unlock();

            frameCounter++;
            auto curTime = std::chrono::steady_clock::now();
            double duration = std::chrono::duration<double>(curTime - startTime).count();
            std::string fps_text = "FPS: " + std::to_string(lastFPS);
            cv::putText(img, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

            if (duration >= 1.0)
            {
                lastFPS = frameCounter;
                frameCounter = 0;
                startTime = curTime;
            }
            cv::imshow("H264 Stream", img);
            cv::waitKey(1);
        }
        av_packet_unref(&packet);
    }

    // 释放资源
    av_free(buffer);
    av_frame_free(&frameBGR);
    av_frame_free(&frame);
    avcodec_close(codecContext);
    avformat_close_input(&inputContext);
    cv::destroyAllWindows();

    socket_thread.join();
    return 0;
}