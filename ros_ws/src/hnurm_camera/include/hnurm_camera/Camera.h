#ifndef HKCAM
#define HKCAM

#include <atomic>
#include <memory>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "hnurm_camera/MvCameraControl.h"

namespace hnurm
{
class HKcam
{
public:
  explicit HKcam(const std::shared_ptr<rclcpp::Node>& node);

  ~HKcam();

  void init_params();

  bool OpenCam(const std::string& cameraId = "");

  bool CloseCam();

  bool GetFrame(std::vector<uint8_t>& img, builtin_interfaces::msg::Time& out_timestamp);

  void SetParam();

  void prepare_convert(uint8_t* dstData);

private:
  // state num
  int nRet;

  // handle for manipulating the Camera
  void* handle;

  // camera param
  MVCC_INTVALUE_EX stParam{};

  // image address and info
  MV_FRAME_OUT stOutFrame{};

  // format of frame ,read from camera
  MV_CC_PIXEL_CONVERT_PARAM_EX stConvertParam{};

  rclcpp::Node::SharedPtr _node;
  rclcpp::Logger          _logger;

  std::string _id = "none";

public:
  int   _nImageOrientation           = 0;
  int   _nWidth                      = 1'440;
  int   _nHeight                     = 1'080;
  int   _nOffsetX                    = 0;
  int   _nOffsetY                    = 0;
  bool  _bReverseX                   = false;
  bool  _bReverseY                   = false;
  int   _nPixelFormat                = 0x01080009;
  int   _nAcquisitionBurstFrameCount = 1;
  float _fFPS                        = 404.0;
  bool  _bAcquisitionFrameRateEnable = false;
  float _fExposureTime               = 2'000;
  float _fGain                       = 8;
  int   _nBlackLevelValue            = 30;
  bool  _bEnableBlackLevel           = false;
  int   _nBayerCvtQuality            = 1;
  int   _nGainAuto                   = 0;

};  // HKcam

}  // namespace hnurm
#endif
