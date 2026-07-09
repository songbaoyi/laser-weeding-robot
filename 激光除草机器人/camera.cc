#include "camera.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>  

bool g_bExit = false;

void PressEnterToExit(void)
{
    int c;
    while ((c = getchar()) != '\n' && c != EOF);
    fprintf(stderr, "\nPress enter to exit.\n");
    while (getchar() != '\n');
    g_bExit = true;
    sleep(1);
}

static bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo) {
        printf("设备信息指针为空！\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
        printf("相机型号: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("相机IP: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
    } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
        printf("相机型号: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
    }
    return true;
}

HikCamera::HikCamera() : m_handle(NULL), m_is_initialized(false) {}

HikCamera::~HikCamera() {
    Destroy();
}

bool HikCamera::Init() {
    int nRet = MV_OK;
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 1. 初始化SDK
    nRet = MV_CC_Initialize();
    if (MV_OK != nRet) {
        printf("海康SDK初始化失败！错误码: 0x%x\n", nRet);
        return false;
    }

    // 2. 枚举设备
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet || stDeviceList.nDeviceNum <= 0) {
        printf("未找到任何相机！错误码: 0x%x\n", nRet);
        MV_CC_Finalize();
        return false;
    }

    // 3. 打印设备信息并选择第一个
    printf("找到 %d 台相机:\n", stDeviceList.nDeviceNum);
    for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
        printf("[设备 %d]:\n", i);
        PrintDeviceInfo(stDeviceList.pDeviceInfo[i]);
    }
    printf("自动选择第0台相机\n");

    // 4. 创建句柄并打开设备
    nRet = MV_CC_CreateHandle(&m_handle, stDeviceList.pDeviceInfo[0]);
    if (MV_OK != nRet) {
        printf("创建相机句柄失败！错误码: 0x%x\n", nRet);
        MV_CC_Finalize();
        return false;
    }

    nRet = MV_CC_OpenDevice(m_handle);
    if (MV_OK != nRet) {
        printf("打开相机失败！错误码: 0x%x\n", nRet);
        MV_CC_DestroyHandle(m_handle);
        MV_CC_Finalize();
        return false;
    }

    // 5. 优化GigE相机包大小
    if (stDeviceList.pDeviceInfo[0]->nTLayerType == MV_GIGE_DEVICE) {
        int nPacketSize = MV_CC_GetOptimalPacketSize(m_handle);
        if (nPacketSize > 0) {
            MV_CC_SetIntValueEx(m_handle, "GevSCPSPacketSize", nPacketSize);
        }
    }

    // 6. 设置连续采集模式
    nRet = MV_CC_SetEnumValue(m_handle, "TriggerMode", 0);
    if (MV_OK != nRet) {
        printf("设置触发模式失败！错误码: 0x%x\n", nRet);
        Destroy();
        return false;
    }

    // 7. 开始采集
    nRet = MV_CC_StartGrabbing(m_handle);
    if (MV_OK != nRet) {
        printf("开始采集失败！错误码: 0x%x\n", nRet);
        Destroy();
        return false;
    }

    m_is_initialized = true;
    printf("海康相机初始化成功！\n");
    return true;
}

bool HikCamera::GetFrame(image_buffer_t& img, MV_FRAME_OUT& frame_out) {
    if (!m_is_initialized || !m_handle) {
        printf("相机未初始化！\n");
        return false;
    }

    MV_FRAME_OUT stBayerFrame = {0};
    // 1. 获取相机原始 Bayer 帧
    int nRet = MV_CC_GetImageBuffer(m_handle, &stBayerFrame, 1000);
    if (nRet != MV_OK) {
        return false;
    }

    // 2. 配置像素格式转换参数
    MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
    stConvertParam.nWidth  = stBayerFrame.stFrameInfo.nWidth;
    stConvertParam.nHeight = stBayerFrame.stFrameInfo.nHeight;
    stConvertParam.pSrcData    = stBayerFrame.pBufAddr;
    stConvertParam.nSrcDataLen = stBayerFrame.stFrameInfo.nFrameLen;
    stConvertParam.enSrcPixelType  = PixelType_Gvsp_BayerBG8;    
    stConvertParam.enDstPixelType  = PixelType_Gvsp_RGB8_Packed; 

    // 3. 分配 RGB 目标内存 
    unsigned int nDstSize = stConvertParam.nWidth * stConvertParam.nHeight * 3;
    stConvertParam.pDstBuffer = (unsigned char*)malloc(nDstSize);
    if (!stConvertParam.pDstBuffer) {
        MV_CC_FreeImageBuffer(m_handle, &stBayerFrame);
        return false;
    }
    stConvertParam.nDstBufferSize = nDstSize;

    // 4. 调用海康 SDK 进行硬件加速转换
    nRet = MV_CC_ConvertPixelType(m_handle, &stConvertParam);
    if (nRet != MV_OK) {
        free(stConvertParam.pDstBuffer);
        MV_CC_FreeImageBuffer(m_handle, &stBayerFrame);
        return false;
    }

    // 5. 填充 frame_out
    frame_out.pBufAddr = stConvertParam.pDstBuffer;
    frame_out.stFrameInfo.nWidth  = stConvertParam.nWidth;
    frame_out.stFrameInfo.nHeight = stConvertParam.nHeight;
    frame_out.stFrameInfo.nFrameLen = nDstSize;

    // 6. 填充官方 image_buffer_t
    img.virt_addr = (uint8_t*)frame_out.pBufAddr;
    img.width  = frame_out.stFrameInfo.nWidth;
    img.height = frame_out.stFrameInfo.nHeight;
    img.format = IMAGE_FORMAT_RGB888;  // 
    img.size   = frame_out.stFrameInfo.nFrameLen;

    // 7. 释放原始 Bayer 帧
    MV_CC_FreeImageBuffer(m_handle, &stBayerFrame);

    return true;
}

void HikCamera::Destroy() {
    if (m_handle) {
        MV_CC_StopGrabbing(m_handle);
        MV_CC_CloseDevice(m_handle);
        MV_CC_DestroyHandle(m_handle);
        m_handle = NULL;
    }
    if (m_is_initialized) {
        MV_CC_Finalize();
        m_is_initialized = false;
    }
    printf("海康相机资源已释放\n");
}

