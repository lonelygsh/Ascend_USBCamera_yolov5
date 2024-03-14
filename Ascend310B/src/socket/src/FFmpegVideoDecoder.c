#include "socket/FFmpegVideoDecoder.h"

struct AVCodecContext *pAVCodecCtx_decoder = NULL;
struct AVCodec *pAVCodec_decoder;
struct AVPacket mAVPacket_decoder;
struct AVFrame *pAVFrame_decoder = NULL;
struct SwsContext *pImageConvertCtx_decoder = NULL;
struct AVFrame *pFrameYUV_decoder = NULL;

int FFmpeg_VideoDecoderInit(AVCodecParameters *codecParameters)
{
    if (!codecParameters)
    {
        printf("Source codec context is NULL."); // printf需替换为printf
        return -1;
    }
    FFmpeg_VideoDecoderRelease();
    avcodec_register_all();

    pAVCodec_decoder = avcodec_find_decoder(codecParameters->codec_id);
    if (!pAVCodec_decoder)
    {
        printf("Can not find codec:%d\n", codecParameters->codec_id);
        return -2;
    }

    pAVCodecCtx_decoder = avcodec_alloc_context3(pAVCodec_decoder);
    if (!pAVCodecCtx_decoder)
    {
        printf("Failed to alloc codec context.");
        FFmpeg_VideoDecoderRelease();
        return -3;
    }

    if (avcodec_parameters_to_context(pAVCodecCtx_decoder, codecParameters) < 0)
    {
        printf("Failed to copy avcodec parameters to codec context.");
        FFmpeg_VideoDecoderRelease();
        return -3;
    }

    if (avcodec_open2(pAVCodecCtx_decoder, pAVCodec_decoder, NULL) < 0)
    {
        printf("Failed to open h264 decoder");
        FFmpeg_VideoDecoderRelease();
        return -4;
    }

    av_init_packet(&mAVPacket_decoder);

    pAVFrame_decoder = av_frame_alloc();
    pFrameYUV_decoder = av_frame_alloc();

    return 0;
}

int FFmpeg_H264DecoderInit()
{
    avcodec_register_all();
    AVCodec *pAVCodec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!pAVCodec)
    {
        printf("can not find H264 codec\n");
        return -1;
    }

    AVCodecContext *pAVCodecCtx = avcodec_alloc_context3(pAVCodec);
    if (pAVCodecCtx == NULL)
    {
        printf("Could not alloc video context!\n");
        return -2;
    }

    AVCodecParameters *codecParameters = avcodec_parameters_alloc();
    if (avcodec_parameters_from_context(codecParameters, pAVCodecCtx) < 0)
    {
        printf("Failed to copy avcodec parameters from codec context.");
        avcodec_parameters_free(&codecParameters);
        avcodec_free_context(&pAVCodecCtx);
        return -3;
    }

    int ret = FFmpeg_VideoDecoderInit(codecParameters);
    avcodec_parameters_free(&codecParameters);
    avcodec_free_context(&pAVCodecCtx);

    return ret;
}

AVFormatContext *outputContext = NULL;
AVStream *stream = NULL;

int FFmpeg_socketInit()
{
    // 初始化FFmpeg
    av_register_all();
    avformat_network_init();

    // 创建输出上下文
    avformat_alloc_output_context2(&outputContext, NULL, "flv", PC_IP_Port); // 你的目标IP地址和端口

    // 添加视频流
    stream = avformat_new_stream(outputContext, NULL);
    stream->codecpar->codec_id = AV_CODEC_ID_H264;
    stream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    stream->codecpar->width = 1280; // 视频宽度
    stream->codecpar->height = 720; // 视频高度

    // 打开输出URL（UDP流）
    if (avio_open(&outputContext->pb, PC_IP_Port, AVIO_FLAG_WRITE) < 0)
    {
        printf("Failed to open output URL.\n");
        return -1;
    }

    // 写文件头部
    avformat_write_header(outputContext, NULL);

    return 0;
}

int FFmpeg_socketRelease()
{
    // 写入文件尾部
    av_write_trailer(outputContext);

    // 关闭输出上下文
    avio_close(outputContext->pb);
    avformat_free_context(outputContext);

    // 关闭FFmpeg库
    avformat_network_deinit();
}

int FFmpeg_VideoDecoderRelease()
{
    if (pAVCodecCtx_decoder != NULL)
    {
        avcodec_free_context(&pAVCodecCtx_decoder);
        pAVCodecCtx_decoder = NULL;
    }

    if (pAVFrame_decoder != NULL)
    {
        av_packet_unref(&mAVPacket_decoder);
        av_free(pAVFrame_decoder);
        pAVFrame_decoder = NULL;
    }

    if (pFrameYUV_decoder)
    {
        av_frame_unref(pFrameYUV_decoder);
        av_free(pFrameYUV_decoder);
        pFrameYUV_decoder = NULL;
    }

    if (pImageConvertCtx_decoder)
    {
        sws_freeContext(pImageConvertCtx_decoder);
    }

    av_packet_unref(&mAVPacket_decoder);

    return 0;
}

int FFmpeg_H264Decode(unsigned char *inbuf, int inbufSize, int *framePara, unsigned char *outRGBBuf, unsigned char **outYUVBuf)
{
    if (!pAVCodecCtx_decoder || !pAVFrame_decoder || !inbuf || inbufSize <= 0 || !framePara || (!outRGBBuf && !outYUVBuf))
    {
        return -1;
    }
    av_frame_unref(pAVFrame_decoder);
    av_frame_unref(pFrameYUV_decoder);

    framePara[0] = framePara[1] = 0;
    mAVPacket_decoder.data = inbuf;
    mAVPacket_decoder.size = inbufSize;

    int ret = avcodec_send_packet(pAVCodecCtx_decoder, &mAVPacket_decoder);
    if (ret == 0)
    {
        ret = avcodec_receive_frame(pAVCodecCtx_decoder, pAVFrame_decoder);
        if (ret == 0)
        {
            framePara[0] = pAVFrame_decoder->width;
            framePara[1] = pAVFrame_decoder->height;

            if (outYUVBuf)
            {
                *outYUVBuf = (unsigned char *)pAVFrame_decoder->data;
                framePara[2] = pAVFrame_decoder->linesize[0];
                framePara[3] = pAVFrame_decoder->linesize[1];
                framePara[4] = pAVFrame_decoder->linesize[2];
            }
            else if (outRGBBuf)
            {
                pFrameYUV_decoder->data[0] = outRGBBuf;
                pFrameYUV_decoder->data[1] = NULL;
                pFrameYUV_decoder->data[2] = NULL;
                pFrameYUV_decoder->data[3] = NULL;
                int linesize[4] = {pAVCodecCtx_decoder->width * 3, pAVCodecCtx_decoder->height * 3, 0, 0};
                pImageConvertCtx_decoder = sws_getContext(pAVCodecCtx_decoder->width, pAVCodecCtx_decoder->height, AV_PIX_FMT_YUV420P, pAVCodecCtx_decoder->width, pAVCodecCtx_decoder->height, AV_PIX_FMT_RGB24, SWS_FAST_BILINEAR, NULL, NULL, NULL);
                sws_scale(pImageConvertCtx_decoder, (const uint8_t *const *)pAVFrame_decoder->data, pAVFrame_decoder->linesize, 0, pAVCodecCtx_decoder->height, pFrameYUV_decoder->data, linesize);
                sws_freeContext(pImageConvertCtx_decoder);

                return 1;
            }
        }
        else if (ret == AVERROR(EAGAIN))
        {
            return 0;
        }
        else
        {
            return -1;
        }
    }

    return 0;
}