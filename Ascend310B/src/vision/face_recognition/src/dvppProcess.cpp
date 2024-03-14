#include "face_recognition/face_recognition.h"

// 设置JPEG解码为NV12格式的图片描述符
bool Face_recognition::SetJpegdPicDescNV12(uint32_t srcWidth, uint32_t srcHeight, acldvppPixelFormat dstFormat)
{ // case YUV420SP NV12 8bit / YUV420SP NV21 8bit
    jpegdDstWidth_ = ALIGN_UP2(srcWidth);
    jpegdDstHeight_ = ALIGN_UP2(srcHeight);
    jpegdDstWidthStride_ = ALIGN_UP64(jpegdDstWidth_);
    jpegdDstHeightStride_ = ALIGN_UP16(jpegdDstHeight_);
    aclError aclRet = acldvppMalloc(&jpegdDstBuffer_, jpegdDstBufferSize_);
    CHECK_RET(aclRet == ACL_SUCCESS, LOG_PRINT("[ERROR] Malloc dvpp memory for jpegd failed. ERROR: %d", aclRet); return false);

    jpegdDstPicDesc_ = acldvppCreatePicDesc();
    CHECK_RET(jpegdDstPicDesc_ != nullptr, LOG_PRINT("[ERROR] Create dvpp pic desc for jpegd failed."); return false);
    acldvppSetPicDescData(jpegdDstPicDesc_, jpegdDstBuffer_);
    acldvppSetPicDescSize(jpegdDstPicDesc_, jpegdDstBufferSize_);
    acldvppSetPicDescFormat(jpegdDstPicDesc_, dstFormat);
    acldvppSetPicDescWidth(jpegdDstPicDesc_, jpegdDstWidth_);
    acldvppSetPicDescHeight(jpegdDstPicDesc_, jpegdDstHeight_);
    acldvppSetPicDescWidthStride(jpegdDstPicDesc_, jpegdDstWidthStride_);
    acldvppSetPicDescHeightStride(jpegdDstPicDesc_, jpegdDstHeightStride_);
    return true;
}

bool Face_recognition::SetVpcInputPicDescYUV420SP(acllite::ImageData &src)
{
    // case YUV420SP NV12 8bit / YUV420SP NV21 8bit
    jpegdDstWidth_ = ALIGN_UP2(src.width);
    jpegdDstHeight_ = ALIGN_UP2(src.height);

    if (src.alignWidth == 0 || src.alignHeight == 0)
    {
        jpegdDstWidthStride_ = jpegdDstWidth_;
        jpegdDstHeightStride_ = ALIGN_UP2(jpegdDstHeight_);
    }
    else
    {
        jpegdDstWidthStride_ = src.alignWidth;
        jpegdDstHeightStride_ = src.alignHeight;
    }

    jpegdDstBufferSize_ = YUV420SP_SIZE(jpegdDstWidthStride_, jpegdDstHeightStride_);
    jpegdDstPicDesc_ = acldvppCreatePicDesc();
    CHECK_RET(jpegdDstPicDesc_ != nullptr, LOG_PRINT("[ERROR] Create dvpp pic desc for resize input failed."); return false);
    acldvppSetPicDescData(jpegdDstPicDesc_, src.data.get());
    acldvppSetPicDescSize(jpegdDstPicDesc_, jpegdDstBufferSize_);
    acldvppSetPicDescFormat(jpegdDstPicDesc_, src.format);
    acldvppSetPicDescWidth(jpegdDstPicDesc_, jpegdDstWidth_);
    acldvppSetPicDescHeight(jpegdDstPicDesc_, jpegdDstHeight_);
    acldvppSetPicDescWidthStride(jpegdDstPicDesc_, jpegdDstWidthStride_);
    acldvppSetPicDescHeightStride(jpegdDstPicDesc_, jpegdDstHeightStride_);
    return true;
}

bool Face_recognition::SetVpcOutputPicDescYUV420SP(acllite::ImageSize dsize, acldvppPixelFormat format)
{
    // case YUV420SP NV12 8bit / YUV420SP NV21 8bit
    resizeDstWidth_ = ALIGN_UP2(dsize.width);
    resizeDstHeight_ = ALIGN_UP2(dsize.height);
    resizeDstWidthStride_ = resizeDstWidth_;
    resizeDstHeightStride_ = ALIGN_UP2(resizeDstHeight_);

    resizeDstBufferSize_ = YUV420SP_SIZE(resizeDstWidthStride_, resizeDstHeightStride_);
    aclError aclRet = acldvppMalloc(&resizeDstBuffer_, resizeDstBufferSize_);
    CHECK_RET(aclRet == ACL_SUCCESS, LOG_PRINT("[ERROR] Malloc dvpp memory for resize output failed. ERROR: %d", aclRet); return false);

    resizeDstPicDesc_ = acldvppCreatePicDesc();
    CHECK_RET(resizeDstPicDesc_ != nullptr, LOG_PRINT("[ERROR] Create dvpp pic desc for resize output failed."); return false);
    acldvppSetPicDescData(resizeDstPicDesc_, resizeDstBuffer_);
    acldvppSetPicDescSize(resizeDstPicDesc_, resizeDstBufferSize_);
    acldvppSetPicDescFormat(resizeDstPicDesc_, format);
    acldvppSetPicDescWidth(resizeDstPicDesc_, resizeDstWidth_);
    acldvppSetPicDescHeight(resizeDstPicDesc_, resizeDstHeight_);
    acldvppSetPicDescWidthStride(resizeDstPicDesc_, resizeDstWidthStride_);
    acldvppSetPicDescHeightStride(resizeDstPicDesc_, resizeDstHeightStride_);
    return true;
}

bool Face_recognition::SetVpcInputPicDescYUV422P(acllite::ImageData &src)
{
    // case YUV422Packed 8bit
    jpegdDstWidth_ = ALIGN_UP2(src.width);
    jpegdDstHeight_ = src.height;

    if (src.alignWidth == 0 || src.alignHeight == 0)
    {
        jpegdDstWidthStride_ = jpegdDstWidth_ * 2;
        jpegdDstHeightStride_ = jpegdDstHeight_;
    }
    else
    {
        jpegdDstWidthStride_ = src.alignWidth;
        jpegdDstHeightStride_ = src.alignHeight;
    }

    jpegdDstBufferSize_ = YUV422P_SIZE(jpegdDstWidthStride_, jpegdDstHeightStride_);
    jpegdDstPicDesc_ = acldvppCreatePicDesc();
    CHECK_RET(jpegdDstPicDesc_ != nullptr, LOG_PRINT("[ERROR] Create dvpp pic desc for resize input failed."); return false);
    acldvppSetPicDescData(jpegdDstPicDesc_, src.data.get());
    acldvppSetPicDescSize(jpegdDstPicDesc_, jpegdDstBufferSize_);
    acldvppSetPicDescFormat(jpegdDstPicDesc_, src.format);
    acldvppSetPicDescWidth(jpegdDstPicDesc_, jpegdDstWidth_);
    acldvppSetPicDescHeight(jpegdDstPicDesc_, jpegdDstHeight_);
    acldvppSetPicDescWidthStride(jpegdDstPicDesc_, jpegdDstWidthStride_);
    acldvppSetPicDescHeightStride(jpegdDstPicDesc_, jpegdDstHeightStride_);
    return true;
}

void Face_recognition::ResizeUpperLeft(acllite::ImageData &src, acllite::ImageData &dst, acllite::ImageSize dsize)
{
    if (dsize.width <= 0 || dsize.height <= 0)
    {
        LOG_PRINT("[ERROR] dsize(%d, %d) not supported", dsize.width, dsize.height);
    }
    acldvppPixelFormat format;
    switch (src.format)
    {
    case PIXEL_FORMAT_YUV_SEMIPLANAR_420:
        format = src.format;
        SetVpcInputPicDescYUV420SP(src);
        SetVpcOutputPicDescYUV420SP(dsize, format);
        break;
    case PIXEL_FORMAT_YVU_SEMIPLANAR_420:
        format = src.format;
        SetVpcInputPicDescYUV420SP(src);
        SetVpcOutputPicDescYUV420SP(dsize, format);
        break;
    case PIXEL_FORMAT_YUYV_PACKED_422:
        format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
        SetVpcInputPicDescYUV422P(src);
        SetVpcOutputPicDescYUV420SP(dsize, format);
        break;
    default:
        LOG_PRINT("[ERROR] input format not supported.");
        return;
    }
    // crop area
    // must even
    uint32_t cropLeftOffset = 0;
    // must even
    uint32_t cropTopOffset = 0;
    // must odd
    uint32_t cropRightOffset = (((cropLeftOffset + src.width) >> 1) << 1) - 1;
    // must odd
    uint32_t cropBottomOffset = (((cropTopOffset + src.height) >> 1) << 1) - 1;
    // data created to describe area location
    cropArea_ = acldvppCreateRoiConfig(cropLeftOffset, cropRightOffset,
                                       cropTopOffset, cropBottomOffset);
    CHECK_RET(cropArea_ != nullptr, LOG_PRINT("[ERROR] acldvppCreateRoiConfig for crop config failed."); return);

    // paste area
    float rx = (float)src.width / (float)dsize.width;
    float ry = (float)src.height / (float)dsize.height;
    int dx = 0;
    int dy = 0;
    float r = 0.0f;
    if (rx > ry)
    {
        dx = 0;
        r = rx;
        dy = (dsize.height - src.height / r) / 2;
    }
    else
    {
        dy = 0;
        r = ry;
        dx = (dsize.width - src.width / r) / 2;
    }

    // must even
    uint32_t pasteLeftOffset = 0;
    // must even
    uint32_t pasteTopOffset = 0;
    // must odd
    uint32_t pasteRightOffset = (((dsize.width - 2 * dx) >> 1) << 1) - 1;
    ;
    // must odd
    uint32_t pasteBottomOffset = (((dsize.height - 2 * dy) >> 1) << 1) - 1;

    pasteArea_ = acldvppCreateRoiConfig(pasteLeftOffset, pasteRightOffset,
                                        pasteTopOffset, pasteBottomOffset);
    CHECK_RET(pasteArea_ != nullptr, LOG_PRINT("[ERROR] acldvppCreateRoiConfig for paste config failed."); return);

    // crop and patse pic
    aclError aclRet = acldvppVpcCropAndPasteAsync(dvppChannelDesc_, jpegdDstPicDesc_,
                                                  resizeDstPicDesc_, cropArea_,
                                                  pasteArea_, PreProcessStream_);
    CHECK_RET(aclRet == ACL_SUCCESS, LOG_PRINT("[ERROR] acldvppVpcCropAndPasteAsync failed. ERROR: %d", aclRet); return);

    aclRet = aclrtSynchronizeStream(PreProcessStream_);
    CHECK_RET(aclRet == ACL_SUCCESS, LOG_PRINT("[ERROR] crop and paste aclrtSynchronizeStream failed. ERROR: %d", aclRet); return);

    dst.data = SHARED_PTR_DVPP_BUF(resizeDstBuffer_);
    dst.size = resizeDstBufferSize_;
    dst.width = resizeDstWidth_;
    dst.height = resizeDstHeight_;
    dst.alignWidth = resizeDstWidthStride_;
    dst.alignHeight = resizeDstHeightStride_;
    dst.format = format;
    if (cropArea_ != nullptr)
    {
        (void)acldvppDestroyRoiConfig(cropArea_);
        cropArea_ = nullptr;
    }
    if (pasteArea_ != nullptr)
    {
        (void)acldvppDestroyRoiConfig(pasteArea_);
        pasteArea_ = nullptr;
    }
    if (jpegdDstPicDesc_ != nullptr)
    {
        (void)acldvppDestroyPicDesc(jpegdDstPicDesc_);
        jpegdDstPicDesc_ = nullptr;
    }
    if (resizeDstPicDesc_ != nullptr)
    {
        (void)acldvppDestroyPicDesc(resizeDstPicDesc_);
        resizeDstPicDesc_ = nullptr;
    }
    return;
}

acllite::ImageData Face_recognition::JpegD(void *&inputData, uint32_t size, acldvppPixelFormat dstFormat)
{
    // struct ImageData
    // get image info
    aclError aclRet;
    acllite::ImageData dst;
    uint32_t width = cameraImage_width;
    uint32_t height = cameraImage_height;
    // int32_t ch = 3;
    jpegdDstBufferSize_ = jpegdBufSize;
    // acldvppJpegFormat srcFormat;
    // aclRet = acldvppJpegGetImageInfoV2(inputData, size, &width, &height, &ch, &srcFormat);
    // CHECK_RET(aclRet == ACL_SUCCESS, LOG_PRINT("[ERROR] Get image info from jpeg pic failed. ERROR: %d", aclRet); return dst);
    // printf("Info %d %d %d %d\r\n", width, height, ch, srcFormat);

    // aclRet = acldvppJpegPredictDecSize(inputData, size, dstFormat, &jpegdDstBufferSize_);
    // CHECK_RET(aclRet == ACL_SUCCESS, LOG_PRINT("[ERROR] Get image size predict from jpeg pic failed. ERROR: %d", aclRet); return dst);
    // printf("Info %d\r\n", jpegdDstBufferSize_);

    // malloc device memory for jpegD input
    void *deviceMem = nullptr;
    aclRet = acldvppMalloc(&deviceMem, size);
    CHECK_RET(aclRet == ACL_SUCCESS, LOG_PRINT("[ERROR] acldvppMalloc failed. ERROR: %d", aclRet); return dst);
    // copy to device
    aclRet = aclrtMemcpy(deviceMem, size, inputData, size, ACL_MEMCPY_DEVICE_TO_DEVICE);
    if (aclRet != ACL_SUCCESS)
    {
        LOG_PRINT("[ERROR] aclrtMemcpy failed. ERROR: %d", aclRet);
        acldvppFree(deviceMem);
        return dst;
    }
    // 使用的是v4l2内存映射来获取摄像头数据，在析构函数中处理了这部分内存的释放，所以这里不需要释放inputData
    // delete[] ((uint8_t *)inputData);

    // set jpegD output desc
    bool ret = SetJpegdPicDescNV12(width, height, dstFormat);
    CHECK_RET(ret, LOG_PRINT("[ERROR] set jpegd output desc failed. ERROR: %d", ret); return dst);
    // jpegD
    aclRet = acldvppJpegDecodeAsync(dvppChannelDesc_,
                                    reinterpret_cast<void *>(deviceMem),
                                    size, jpegdDstPicDesc_, PreProcessStream_);
    CHECK_RET(aclRet == ACL_SUCCESS, LOG_PRINT("[ERROR] acldvppJpegDecodeAsync failed. ERROR: %d", aclRet); return dst);
    aclRet = aclrtSynchronizeStream(PreProcessStream_);
    CHECK_RET(aclRet == ACL_SUCCESS, LOG_PRINT("[ERROR] acldvppJpegDecodeAsync sync stream failed. ERROR: %d", aclRet); return dst);
    dst.data = SHARED_PTR_DVPP_BUF(jpegdDstBuffer_);
    dst.size = jpegdDstBufferSize_;
    dst.width = jpegdDstWidth_;
    dst.height = jpegdDstHeight_;
    dst.alignWidth = jpegdDstWidthStride_;
    dst.alignHeight = jpegdDstHeightStride_;
    dst.format = dstFormat;
    // release jpegd input mem and output pic desc
    acldvppFree(deviceMem);
    if (jpegdDstPicDesc_ != nullptr)
    {
        acldvppDestroyPicDesc(jpegdDstPicDesc_);
        jpegdDstPicDesc_ = nullptr;
    }
    return dst;
}

uint32_t AlignmentHelper(uint32_t origSize, uint32_t alignment)
{
    if (alignment == 0)
    {
        return 0;
    }
    uint32_t alignmentH = alignment - 1;
    return (origSize + alignmentH) / alignment * alignment;
}

void Face_recognition::JpegeInit(uint32_t jpege_encodeLevel)
{
    // runMode为ACL_DEVICE
    // aclrtRunMode runMode;
    // aclError aclret = aclrtGetRunMode(&runMode);
    // printf("mode is %d\r\n", runMode);
    if (jpege_encodeLevel > 100)
        jpege_encodeLevel = 100;
    jpegeEncodeLevel = jpege_encodeLevel;
    jpegeChannelDesc_ = acldvppCreateChannelDesc();
    acldvppCreateChannel(jpegeChannelDesc_);
    jpegeConfig = acldvppCreateJpegeConfig();
    acldvppSetJpegeConfigLevel(jpegeConfig, jpegeEncodeLevel);

    jpegeSrcWidth_ = cameraImage_width;
    jpegeSrcHeight_ = cameraImage_height;
    uint32_t widthAlignment = 16;
    uint32_t heightAlignment = 2;
    uint32_t sizeAlignment = 3;
    jpegeSrcWidthStride_ = AlignmentHelper(jpegeSrcWidth_, widthAlignment);
    jpegeSrcHeightStride_ = AlignmentHelper(jpegeSrcHeight_, heightAlignment);
    if (jpegeSrcWidthStride_ == 0 || jpegeSrcHeightStride_ == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "ComputeEncodeInputSize AlignmentHelper failed");
        return;
    }
    jpegeSrcBufferSize_ = jpegeSrcWidthStride_ * jpegeSrcHeightStride_ * sizeAlignment;
}

void Face_recognition::DestroyJpegeResource()
{
    if (jpegeConfig != nullptr)
    {
        (void)acldvppDestroyJpegeConfig(jpegeConfig);
        jpegeConfig = nullptr;
    }
    if (jpegeSrcPicDesc_ != nullptr)
    {
        (void)acldvppDestroyPicDesc(jpegeSrcPicDesc_);
        jpegeSrcPicDesc_ = nullptr;
    }
    if (jpegeSrcBuffer_ != nullptr)
    {
        (void)acldvppFree(jpegeSrcBuffer_);
        jpegeSrcBuffer_ = nullptr;
    }
}

imageBuffer Face_recognition::Jpege(void *&inputData)
{
    imageBuffer image_buffer = {nullptr, 0};

    aclError aclRet = acldvppMalloc(&jpegeSrcBuffer_, jpegeSrcBufferSize_);
    if (aclRet != ACL_SUCCESS)
    {
        acldvppFree(jpegeSrcBuffer_);
        RCLCPP_ERROR(this->get_logger(), "malloc device data buffer failed, aclRet is %d", aclRet);
        return image_buffer;
    }
    aclRet = aclrtMemcpy(jpegeSrcBuffer_, jpegeSrcBufferSize_, inputData, jpegeSrcBufferSize_, ACL_MEMCPY_DEVICE_TO_DEVICE);
    if (aclRet != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "memcpy failed. Input host buffer size is %u",
                     jpegeSrcBufferSize_);
        acldvppFree(jpegeSrcBuffer_);
        // delete[] inputData;
        return image_buffer;
    }

    jpegeSrcPicDesc_ = acldvppCreatePicDesc();
    if (jpegeSrcPicDesc_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "acldvppCreatePicDesc jpegeSrcPicDesc_ failed");
        return image_buffer;
    }
    acldvppSetPicDescData(jpegeSrcPicDesc_, jpegeSrcBuffer_);
    acldvppSetPicDescFormat(jpegeSrcPicDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(jpegeSrcPicDesc_, jpegeSrcWidth_);
    acldvppSetPicDescHeight(jpegeSrcPicDesc_, jpegeSrcHeight_);
    acldvppSetPicDescWidthStride(jpegeSrcPicDesc_, jpegeSrcWidthStride_);
    acldvppSetPicDescHeightStride(jpegeSrcPicDesc_, jpegeSrcHeightStride_);
    acldvppSetPicDescSize(jpegeSrcPicDesc_, jpegeSrcBufferSize_);

    acldvppJpegPredictEncSize(jpegeSrcPicDesc_, jpegeConfig, &jpegeDstBufferSize_);
    aclRet = acldvppMalloc(&jpegeDstBuffer_, jpegeDstBufferSize_);
    if (aclRet != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "malloc jpegeDstBuffer_ failed, aclRet is %d", aclRet);
        return image_buffer;
    }
    acldvppSetJpegeConfigLevel(jpegeConfig, jpegeEncodeLevel);

    // call Asynchronous api
    aclRet = acldvppJpegEncodeAsync(jpegeChannelDesc_, jpegeSrcPicDesc_, jpegeDstBuffer_,
                                    &jpegeDstBufferSize_, jpegeConfig, ResultHandleStream_);
    if (aclRet != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "acldvppJpegEncodeAsync failed, aclRet = %d", aclRet);
        return image_buffer;
    }
    aclRet = aclrtSynchronizeStream(ResultHandleStream_);
    if (aclRet != ACL_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "encode aclrtSynchronizeStream failed, aclRet = %d", aclRet);
        return image_buffer;
    }

    if (jpegeSrcPicDesc_ != nullptr)
    {
        (void)acldvppDestroyPicDesc(jpegeSrcPicDesc_);
        jpegeSrcPicDesc_ = nullptr;
    }

    acldvppFree(jpegeSrcBuffer_);
    image_buffer.data = SHARED_PTR_DVPP_BUF(jpegeDstBuffer_);
    image_buffer.imageBufferSize = jpegeDstBufferSize_;
    return image_buffer;
}
