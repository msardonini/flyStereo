#pragma once
#include <atomic>
#include <chrono>
#include <memory>
#include <queue>
#include <thread>

#include "depthai/depthai.hpp"
#include "flyStereo/sensor_io/mavlink/fly_stereo/mavlink.h"
#include "flyStereo/sensor_io/stereo_system_src_interface.h"
#include "opencv2/core/core.hpp"

/**
 * @brief Object to interface with the OakD stereo camera sensor
 *
 */
class OakD : public StereoSystemSrcInterface {
 public:
  /**
   * @brief Construct a new Oak D object
   *
   * @param fps The requested framerate of the camera
   * @param rectify If true, the returned images will be rectified
   */
  OakD(int fps = 30, bool rectify = true);

  /**
   * @brief Destroy the Oak D object
   *
   */
  virtual ~OakD() override = default;

  /**
   * @brief Get A synchronized set of images and corresponding IMU data
   *
   * @param left_image the image from the left camera
   * @param right_image the image from the right camera
   * @param imu_data the IMU data
   * @param current_frame_time the timestamp of the current frame
   * @return int 0 on success, -1 on failure
   */
  int GetSynchronizedData(cv::Mat_<uint8_t> &left_image, cv::Mat_<uint8_t> &right_image,
                          std::vector<mavlink_imu_t> &imu_data, uint64_t &current_frame_time) override;

 protected:
  /**
   * @brief Initialize communication with the OakD stereo camera sensor. This must be called before data can be
   * received.
   *
   * @return int 0 on success, -1 on failure
   */
  void Init(int fps, bool rectify);

  /**
   * @brief Gets the next batch of imu data from the sensor. This function will block until data is available or the
   * timeout period has elapsed
   *
   * @param timeout The timeout period in milliseconds
   * @param has_timed_out If true, the timeout period has elapsed
   * @return std::vector<dai::IMUPacket>
   */
  std::vector<dai::IMUPacket> GetImuData(std::chrono::milliseconds timeout, bool &has_timed_out);

  /**
   * @brief Get the next stereo image pair. This function will block until data is available or the timeout period has
   * elapsed. If frames have dropped, or become out of sync, this function will return empty images until the frames are
   * synchronized again.
   *
   * @param timeout The timeout period in milliseconds
   * @param has_timed_out If true, the timeout period has elapsed
   * @return std::pair<std::shared_ptr<dai::ImgFrame>, std::shared_ptr<dai::ImgFrame>>
   */
  std::pair<std::shared_ptr<dai::ImgFrame>, std::shared_ptr<dai::ImgFrame>> GetStereoImagePair(
      std::chrono::milliseconds timeout, bool &has_timed_out);

  // Objects needed to interface with the dai camera
  dai::Pipeline pipeline_;
  std::unique_ptr<dai::Device> device_;
  std::shared_ptr<dai::node::StereoDepth> stereo_;
  std::shared_ptr<dai::DataOutputQueue> queue_left_;
  std::shared_ptr<dai::DataOutputQueue> queue_right_;
  std::shared_ptr<dai::node::XLinkOut> xout_left_;
  std::shared_ptr<dai::node::XLinkOut> xout_right_;
  std::shared_ptr<dai::node::MonoCamera> mono_left_;
  std::shared_ptr<dai::node::MonoCamera> mono_right_;
  std::shared_ptr<dai::node::IMU> imu_;
  std::shared_ptr<dai::node::XLinkOut> xout_imu_;
  std::shared_ptr<dai::DataOutputQueue> queue_imu_;

  std::queue<dai::IMUPacket> local_queue_imu_;  //< queue for imu data, used for synchronization with the camera

  std::chrono::time_point<std::chrono::steady_clock> start_time_;
};
