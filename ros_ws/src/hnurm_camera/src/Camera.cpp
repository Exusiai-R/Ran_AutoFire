#include "hnurm_camera/Camera.h"

#include <chrono>
#include <cstdio>
#include <iostream>
#include <opencv2/core.hpp>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "hnurm_camera/MvErrorDefine.h"

using namespace cv;
using namespace std;

namespace hnurm
{
HKcam::HKcam(const std::shared_ptr<rclcpp::Node>& node) : _node(node), _logger(node->get_logger())
{
  nRet   = MV_OK;
  handle = nullptr;

  // Initialize SDK (must be called before any other SDK functions)
  nRet = MV_CC_Initialize();
  if (MV_OK != nRet) {
    RCLCPP_ERROR(_logger, "MV_CC_Initialize failed! nRet [0x%x]", nRet);
    // Continue anyway, but SDK may not work properly
  }
  else {
    RCLCPP_INFO(_logger, "SDK initialized successfully");
  }

  init_params();
  std::string camera_id = _node->get_parameter("camera_id").as_string();
  RCLCPP_INFO(_logger, "Camera id set to: %s", camera_id.c_str());

  if (OpenCam(camera_id))  // here open and config camera
  {
    RCLCPP_INFO(_logger, "Camera %s opened successfully.", camera_id.c_str());
  }
  else {
    RCLCPP_ERROR(_logger, "Camera %s failed to open!", camera_id.c_str());
  }
}

HKcam::~HKcam()
{
  // 在析构函数中安全地关闭相机
  // 使用try-catch确保析构函数不会抛出异常
  try {
    if (handle) {
      CloseCam();
      handle = nullptr;  // 确保句柄被清空
    }
  } catch (...) {
    RCLCPP_ERROR(_logger, "Exception in HKcam destructor");
  }

  // Finalize SDK (must be called after all SDK functions are done)
  nRet = MV_CC_Finalize();
  if (MV_OK != nRet) {
    RCLCPP_WARN(_logger, "MV_CC_Finalize failed! nRet [0x%x]", nRet);
  }
  else {
    RCLCPP_INFO(_logger, "SDK finalized successfully");
  }
}

void HKcam::init_params()
{
  _nImageOrientation = static_cast<int>(_node->declare_parameter("image_orientation", 0));

  _nWidth            = static_cast<int>(_node->declare_parameter("nWidth", 1'440));
  _nHeight           = static_cast<int>(_node->declare_parameter("nHeight", 1080));
  _nOffsetX          = static_cast<int>(_node->declare_parameter("nOffsetX", 0));
  _nOffsetY          = static_cast<int>(_node->declare_parameter("nOffsetY", 0));
  _bReverseX         = _node->declare_parameter("bReverseX", false);
  _bReverseY         = _node->declare_parameter("bReverseY", false);
  _nPixelFormat      = static_cast<int>(_node->declare_parameter("nPixelFormat", 0x01080009));
  _fFPS              = static_cast<float>(_node->declare_parameter("fFPS", 404.));
  _fExposureTime     = static_cast<float>(_node->declare_parameter("fExposureTime", 2000.));
  _fGain             = static_cast<float>(_node->declare_parameter("fGain", 8.));
  _nBlackLevelValue  = static_cast<int>(_node->declare_parameter("nBlackLevelValue", 30));
  _bEnableBlackLevel = _node->declare_parameter("bEnableBlackLevel", false);
  _nBayerCvtQuality  = static_cast<int>(_node->declare_parameter("nBayerCvtQuality", 3));
  _nGainAuto         = static_cast<int>(_node->declare_parameter("nGainAuto", 0));
  _bAcquisitionFrameRateEnable = _node->declare_parameter("bAcquisitionFrameRateEnable", false);
  _nAcquisitionBurstFrameCount =
    static_cast<int>(_node->declare_parameter("nAcquisitionBurstFrameCount", 1));
}

void HKcam::SetParam()
{
  MV_CC_SetEnumValue(handle, "ExposureMode", 1);

  nRet = MV_CC_SetIntValueEx(handle, "Width", _nWidth);
  if (MV_OK == nRet)
    RCLCPP_INFO(_logger, "set Width = %d OK!", _nWidth);
  else
    RCLCPP_WARN(_logger, "set Width failed! nRet [%x]", nRet);

  // 设置高度
  nRet = MV_CC_SetIntValueEx(handle, "Height", _nHeight);
  if (MV_OK == nRet)
    RCLCPP_INFO(_logger, "set height = %d OK!", _nHeight);
  else
    RCLCPP_WARN(_logger, "set height failed! nRet [%x]", nRet);

  // 设置水平偏移
  nRet = MV_CC_SetIntValueEx(handle, "OffsetX", _nOffsetX);
  if (MV_OK == nRet)
    RCLCPP_INFO(_logger, "set OffsetX = %d OK!", _nOffsetX);
  else
    RCLCPP_WARN(_logger, "set OffsetX failed! nRet [%x]", nRet);

  // 设置垂直偏移
  nRet = MV_CC_SetIntValueEx(handle, "OffsetY", _nOffsetY);
  if (MV_OK == nRet)
    RCLCPP_INFO(_logger, "set OffsetY = %d OK!", _nOffsetY);
  else
    RCLCPP_WARN(_logger, "set OffsetY failed! nRet [%x]", nRet);

  // 设置水平镜像
  nRet = MV_CC_SetBoolValue(handle, "ReverseX", _bReverseX);
  if (MV_OK == nRet)
    RCLCPP_INFO(_logger, "set ReverseX = %d OK!", _bReverseX);
  else
    RCLCPP_WARN(_logger, "set ReverseX Failed! nRet = [%x]", nRet);

  // 设置垂直镜像
  nRet = MV_CC_SetBoolValue(handle, "ReverseY", _bReverseY);
  if (MV_OK == nRet)
    RCLCPP_INFO(_logger, "Set ReverseY = %d OK!", _bReverseY);
  else
    RCLCPP_WARN(_logger, "Set ReverseY Failed! nRet = [%x]", nRet);

  // 设置像素格式
  nRet = MV_CC_SetEnumValue(handle, "PixelFormat", _nPixelFormat);
  if (MV_OK == nRet)
    RCLCPP_INFO(_logger, "set PixelFormat = %x OK!", _nPixelFormat);
  else
    RCLCPP_WARN(_logger, "set PixelFormat failed! nRet [%x]", nRet);

  // 设置采集帧率
  nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", _fFPS);
  if (MV_OK == nRet)
    RCLCPP_INFO(_logger, "set AcquisitionFrameRate = %f OK!", _fFPS);
  else
    RCLCPP_WARN(_logger, "set AcquisitionFrameRate failed! nRet [%x]", nRet);

  // 设置使能采集帧率控制
  nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", _bAcquisitionFrameRateEnable);
  if (MV_OK == nRet)
    RCLCPP_INFO(_logger, "Set AcquisitionFrameRateEnable = %d OK!", _bAcquisitionFrameRateEnable);
  else
    RCLCPP_WARN(_logger, "Set AcquisitionFrameRateEnable Failed! nRet = [%x]", nRet);

  // 设置触发采集帧数
  if (_bAcquisitionFrameRateEnable) {
    nRet = MV_CC_SetIntValueEx(handle, "AcquisitionBurstFrameCount", _nAcquisitionBurstFrameCount);
    if (MV_OK == nRet)
      RCLCPP_INFO(_logger, "set AcquisitionBurstFrameCount = %d OK!", _nAcquisitionBurstFrameCount);
    else
      RCLCPP_WARN(_logger, "set AcquisitionBurstFrameCount failed! nRet [%x]", nRet);
  }
  else {
    RCLCPP_INFO(_logger, "AcquisitionBurstFrameCount disabled, skipping value setting");
  }

  // 设置曝光时间
  nRet = MV_CC_SetFloatValue(handle, "ExposureTime", _fExposureTime);
  if (MV_OK == nRet)
    RCLCPP_INFO(_logger, "set ExposureTime = %f OK!", _fExposureTime);
  else
    RCLCPP_WARN(_logger, "set ExposureTime failed! nRet [%x]", nRet);

  // 设置增益
  nRet = MV_CC_SetFloatValue(handle, "Gain", _fGain);
  if (MV_OK == nRet)
    RCLCPP_INFO(_logger, "set Gain = %f OK!", _fGain);
  else
    RCLCPP_WARN(_logger, "set Gain failed! nRet [%x]", nRet);

  // 设置黑电平使能
  nRet = MV_CC_SetBoolValue(handle, "BlackLevelEnable", _bEnableBlackLevel);
  if (MV_OK == nRet)
    RCLCPP_INFO(_logger, "Set BlackLevelEnable = %d OK!", _bEnableBlackLevel);
  else
    RCLCPP_WARN(_logger, "Set BlackLevelEnable Failed! nRet = [%x]", nRet);

  // 设置黑电平
  if (_bEnableBlackLevel) {
    nRet = MV_CC_SetIntValueEx(handle, "BlackLevel", _nBlackLevelValue);
    if (MV_OK == nRet)
      RCLCPP_INFO(_logger, "set BlackLevel = %d OK!", _nBlackLevelValue);
    else
      RCLCPP_WARN(_logger, "set BlackLevel failed! nRet [%x]", nRet);
  }
  else {
    RCLCPP_INFO(_logger, "BlackLevel disabled, skipping value setting");
  }

  // 设置Bayer转换质量
  nRet = MV_CC_SetBayerCvtQuality(handle, _nBayerCvtQuality);
  if (MV_OK == nRet)
    RCLCPP_INFO(_logger, "Set BayerCvtQuality = %d OK!", _nBayerCvtQuality);
  else
    RCLCPP_WARN(_logger, "Set BayerCvtQuality Failed! nRet = [%x]", nRet);

  // 设置自动增益
  nRet = MV_CC_SetEnumValue(handle, "GainAuto", _nGainAuto);
  if (MV_OK == nRet)
    RCLCPP_INFO(_logger, "set GainAuto = %d OK!", _nGainAuto);
  else
    RCLCPP_WARN(_logger, "set GainAuto failed! nRet [%x]", nRet);
}

bool HKcam::OpenCam(const string& cameraID)
{
  //        nRet = MV_OK;
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  // 枚举设备
  // enum device
  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (!stDeviceList.nDeviceNum) {
    RCLCPP_ERROR(_logger, "Find No Devices! nRet = [%x]", nRet);
    return false;
  }

  // 选择首个相机连接
  // select the first camera connected
  unsigned int nIndex = 0;

  while (true) {
    // 选择设备并创建句柄
    // select device and create handle
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet) {
      RCLCPP_ERROR(_logger, "MV_CC_CreateHandle fail! nRet [%x]", nRet);
      return false;
    }

    // 获取设备id
    // get device id
    stringstream ss;
    ss << stDeviceList.pDeviceInfo[nIndex]->SpecialInfo.stUsb3VInfo.chDeviceGUID;
    ss >> _id;
    RCLCPP_INFO(_logger, "camera id %s", _id.c_str());

    // 若指定了相机id，则判断是否为指定相机
    // if camera id is specified, check if it is the specified camera
    if (!cameraID.empty()) {
      if (cameraID != _id)  // 若不是指定相机，则关闭句柄并继续枚举
      {
        RCLCPP_WARN(
          _logger, "camera id %s not matched to desired %s", _id.c_str(), cameraID.c_str()
        );
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        nIndex++;
        if (nIndex >= stDeviceList.nDeviceNum)  // 若已枚举完所有相机，则返回
        {
          RCLCPP_ERROR(_logger, "Find No Devices!");
          return false;
        }
        continue;
      }
      else {
        RCLCPP_INFO(_logger, "ready to open camera %s", _id.c_str());
      }
    }
    else {
      RCLCPP_INFO(_logger, "camera id not set, ready to open camera %s", _id.c_str());
    }

    // 打开设备
    // open device
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
      RCLCPP_ERROR(_logger, "MV_CC_OpenDevice fail! nRet [%x]", nRet);
      MV_CC_DestroyHandle(handle);
      return false;
    }

    // 设置触发模式为off
    // set trigger mode as off
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet) {
      RCLCPP_ERROR(_logger, "MV_CC_SetTriggerMode fail! nRet [%x]", nRet);
      MV_CC_CloseDevice(handle);
      MV_CC_DestroyHandle(handle);
      return false;
    }

    // 设置参数
    // set param
    SetParam();

    // 获取数据包大小
    // Get payload size
    memset(&stParam, 0, sizeof(MVCC_INTVALUE_EX));
    nRet = MV_CC_GetIntValueEx(handle, "PayloadSize", &stParam);
    if (MV_OK != nRet) {
      RCLCPP_ERROR(_logger, "Get PayloadSize fail! nRet [0x%x]", nRet);
      MV_CC_CloseDevice(handle);
      MV_CC_DestroyHandle(handle);
      return false;
    }

    // 开始取流
    // Start grab stream
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
      RCLCPP_ERROR(_logger, "Start Grabbing fail! nRet [0x%x]", nRet);
      MV_CC_CloseDevice(handle);
      MV_CC_DestroyHandle(handle);
      return false;
    }

    // 初始化帧输出结构
    // Initialize frame output structure
    stOutFrame = {0};
    memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));
    return true;
  }
}

// todo: this function takes too long to execute than expected when camera is not connected
bool HKcam::CloseCam()
{
  // 如果句柄为空，说明相机已经关闭或从未打开
  if (!handle) {
    RCLCPP_INFO(_logger, "Camera already closed or not opened");
    return true;
  }

  bool success = true;

  try {
    // 停止取流
    // end grab image
    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet) {
      RCLCPP_WARN(_logger, "MV_CC_StopGrabbing fail! nRet [%x]", nRet);
      success = false;
    }

    // 关闭设备
    // close device
    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet) {
      RCLCPP_WARN(_logger, "MV_CC_CloseDevice fail! nRet [%x]", nRet);
      success = false;
    }

    // 销毁句柄
    // destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet) {
      RCLCPP_WARN(_logger, "MV_CC_DestroyHandle fail! nRet [%x]", nRet);
      success = false;
    }

    // 无论成功与否，都清空句柄，避免重复关闭
    handle = nullptr;

    return success;
  } catch (...) {
    RCLCPP_ERROR(_logger, "Exception during camera close");
    handle = nullptr;  // 异常情况下也清空句柄
    return false;
  }
}

bool HKcam::GetFrame(std::vector<uint8_t>& img, builtin_interfaces::msg::Time& out_timestamp)
{
  nRet = MV_OK;
  
  nRet = MV_CC_GetImageBuffer(handle, &stOutFrame, 100);
  if (nRet != MV_OK) {
    RCLCPP_ERROR(_logger, "MV_CC_GetImageBuffer failed with error: 0x%x", nRet);
    return false;
  }

  // Pre-allocate output buffer if needed
  size_t required_size = stOutFrame.stFrameInfo.nHeight * stOutFrame.stFrameInfo.nWidth * 3;
  if (img.size() != required_size) {
    img.resize(required_size);
  }

  // convert bayer
  prepare_convert(img.data());

  // do conversion
  nRet = MV_CC_ConvertPixelTypeEx(handle, &stConvertParam);
  if (nRet != MV_OK) {
    RCLCPP_ERROR(_logger, "Do conversion fail! nRet [0x%x]\n", nRet);
  }

  // Camera timestamp is in milliseconds, convert to nanoseconds
  int64_t timestamp_ms = stOutFrame.stFrameInfo.nHostTimeStamp;
  int64_t timestamp_ns = timestamp_ms * 1000000LL;  // Convert ms to ns
  
  // Convert nanoseconds to seconds and nanoseconds
  out_timestamp.sec = static_cast<int32_t>(timestamp_ns / 1000000000LL);
  out_timestamp.nanosec = static_cast<uint32_t>(timestamp_ns % 1000000000LL);

  if (nullptr != stOutFrame.pBufAddr) {
    nRet = MV_CC_FreeImageBuffer(handle, &stOutFrame);
    if (nRet != MV_OK) {
      RCLCPP_ERROR(_logger, "Free Image Buffer fail! nRet [0x%x]\n", nRet);
    }
  }

  return nRet == MV_OK;
}

void HKcam::prepare_convert(uint8_t* dstData)
{
  stConvertParam.enSrcPixelType = stOutFrame.stFrameInfo.enPixelType;
  stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
  stConvertParam.pSrcData       = stOutFrame.pBufAddr;
  stConvertParam.nSrcDataLen    = stOutFrame.stFrameInfo.nFrameLen;
  stConvertParam.nWidth         = stOutFrame.stFrameInfo.nWidth;
  stConvertParam.nHeight        = stOutFrame.stFrameInfo.nHeight;
  stConvertParam.nDstBufferSize =
    stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3;
  stConvertParam.pDstBuffer = dstData;
}

}  // namespace hnurm
