#pragma once
#include "MvCameraControl.h"
#include "image_utils.h"

// 海康工业相机封装类
class HikCamera {
public:
    HikCamera();
    ~HikCamera();

    // 初始化相机（自动选择第一个设备）
    bool Init();

    // 获取一帧图像并完成 Bayer -> RGB 转换
    // 返回值：成功返回 true，img 填充有效数据，frame_out 用于 main.cc 后续释放 malloc 内存
    bool GetFrame(image_buffer_t& img, MV_FRAME_OUT& frame_out);

    // 释放相机资源
    void Destroy();

    void* GetHandle() { return m_handle; };

private:
    void* m_handle;       // 海康相机句柄
    bool m_is_initialized;
};

// 全局退出标志
extern bool g_bExit;
void PressEnterToExit();

