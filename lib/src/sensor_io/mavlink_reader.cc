#include "fly_stereo/sensor_io/mavlink_reader.h"


#include <sys/stat.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <string>
#include <chrono>
#include <iostream>

#include "spdlog/spdlog.h"

MavlinkReader::MavlinkReader() {}

MavlinkReader::~MavlinkReader() {
  is_running_.store(false);

  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }

  if (serial_dev_ != 0) {
    close(serial_dev_);
  }
}

int MavlinkReader::Init(YAML::Node input_params) {
  // Open the serial device if not in replay mode
  if (!input_params["replay_mode"] ||
      !input_params["replay_mode"]["enable"].as<bool>() ||
      input_params["replay_mode"]["enable_serial_replay"].as<bool>()) {
    std::string dev = input_params["mavlink_reader"]["device"].as<std::string>();
    serial_dev_ = open(dev.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if(serial_dev_ == -1) {
      spdlog::error("Failed to open the serial device");
      return -1;
    }
  } else {
    return 0;
  }

  SetSerialParams(serial_dev_);

  // Now that our serial port is intialized, send the signal to reset the trigger counters
  // in case the flight program has been running already
  SendCounterReset();

  is_running_.store(true);
  reader_thread_ = std::thread(&MavlinkReader::SerialReadThread, this);

  return 0;
}

int MavlinkReader::SetSerialParams(int device) {
  struct termios  config;
  //
  // Get the current configuration of the serial interface
  //
  if(tcgetattr(device, &config) < 0) {
    spdlog::error("Failed to get the serial device attributes");
    return -1;
  }

  fcntl(device, F_SETFL, fcntl(device, F_GETFL) & ~O_NONBLOCK);

  // Set the serial device configs
  config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP
    | IXON);
  config.c_oflag = 0;
  config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  config.c_cflag &= ~(CSIZE | PARENB);
  config.c_cflag |= CS8;
  //
  // One input byte is enough to return from read()
  // Inter-character timer off
  //
  config.c_cc[VMIN]  = 0;
  config.c_cc[VTIME] = 0;

  //
  // Communication speed (simple version, using the predefined
  // constants)
  //
  if(cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0) {
    spdlog::error("Failed to set the baudrate on the serial port!");
    return -1;
  }

  //
  // Finally, apply the configuration
  //
  if(tcsetattr(device, TCSAFLUSH, &config) < 0) {
    spdlog::error("Failed to set the baudrate on the serial port!");
    return -1;
  }
  return 0;
}

void MavlinkReader::SendCounterReset() {
  if (serial_dev_ == 0) {
    return;
  }
  mavlink_message_t msg;
  mavlink_reset_counters_t reset_msg;
  uint8_t buf[1024];
  reset_msg.timestamp_us = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::
    system_clock::now().time_since_epoch()).count();

  mavlink_msg_reset_counters_encode(1, 200, &msg, &reset_msg);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  if (write(serial_dev_, buf, len) < 0) {
    spdlog::error("error on write! Counter Reset");
  }
}


void MavlinkReader::SendVioMsg(const vio_t &vio) {
  if (serial_dev_ == 0) {
    return;
  }
  mavlink_message_t msg;
  mavlink_vio_t vio_msg;
  uint8_t buf[1024];
  vio_msg.timestamp_us = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::
    system_clock::now().time_since_epoch()).count();
  for (unsigned int i = 0; i < 3; i++) {
    vio_msg.position[i] = vio.position(i);
    vio_msg.velocity[i] = vio.velocity(i);
  }

  vio_msg.quat[0] = vio.quat.w();
  vio_msg.quat[1] = vio.quat.x();
  vio_msg.quat[2] = vio.quat.y();
  vio_msg.quat[3] = vio.quat.z();

  mavlink_msg_vio_encode(1, 200, &msg, &vio_msg);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  if (write(serial_dev_, buf, len) < 0) {
    spdlog::error("error on write! Vio Msg");
  }
}

void MavlinkReader::SerialReadThread() {
  unsigned char buf[1024];
  size_t buf_size = 1024;
  // std::chrono::time_point<std::chrono::system_clock> time_end;
  while (is_running_.load()) {
    ssize_t ret = read(serial_dev_, buf, buf_size);

    if (ret < 0) {
      spdlog::error("Error on read(), errno: {}", strerror(errno));
      continue;
    }

    for (int i = 0; i < ret; i++) {
      mavlink_status_t mav_status;
      mavlink_message_t mav_message;
      uint8_t msg_received = mavlink_parse_char(MAVLINK_COMM_1, buf[i], &mav_message, &mav_status);

      if (msg_received) {
        switch(mav_message.msgid) {
          case MAVLINK_MSG_ID_IMU: {
            mavlink_imu_t attitude_msg;
            mavlink_msg_imu_decode(&mav_message, &attitude_msg);

            // spdlog::info("fps: {}", 1.0 / static_cast<std::chrono::duration<double> >
            //   ((std::chrono::system_clock::now() - time_end)).count());
            // time_end = std::chrono::system_clock::now();

            // Push the message to our output queue
            std::lock_guard<std::mutex> lock(queue_mutex_);
            output_queue_.push(attitude_msg);
            cond_var_.notify_one();
            break;
          }
          case MAVLINK_MSG_ID_COMMAND: {
            mavlink_command_t mav_cmd;
            mavlink_msg_command_decode(&mav_message, &mav_cmd);
            std::lock_guard<std::mutex> lock(cmd_msg_mutex_);
            if (mav_cmd.engage) {
              spdlog::debug("Received Start Command!");
              command_on_ = true;
              cmd_msg_cond_var_.notify_one();
            } else if (mav_cmd.shutdown) {
              command_shutdown_ = true;
              cmd_msg_cond_var_.notify_one();
            }
            break;
          }
          default: {
            spdlog::debug("Unrecognized message with ID: {}", static_cast<int>(mav_message.msgid));
          }
        }
      }
    }

    // Sleep so we don't overload the CPU. This isn't an ideal method, but if
    // use a blocking call on read(), we can't break out of it on the
    // destruction of this object. It will hang forever until bytes are read,
    // which is not always the case
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

bool MavlinkReader::GetAttitudeMsg(mavlink_imu_t* attitude, bool block) {
  std::unique_lock<std::mutex> lock(queue_mutex_);

  if (block) {
    while(output_queue_.empty()) {
      std::cv_status ret = cond_var_.wait_for(lock, std::chrono::seconds(1));

      // Return false if we have hit our timeout
      if (ret == std::cv_status::timeout)
        return false;
    }
  }

  if (output_queue_.empty()) {
    return false;
  } else {
    *attitude = output_queue_.front();
    output_queue_.pop();
    return true;
  }
}

bool MavlinkReader::WaitForStartCmd() {
  std::unique_lock<std::mutex> lock(cmd_msg_mutex_);
  while(command_on_ == false) {
    std::cv_status ret = cmd_msg_cond_var_.wait_for(lock, std::chrono::seconds(1));

    // Return false if we have hit our timeout
    if (ret == std::cv_status::timeout)
      return false;
  }

  return true;
}

void MavlinkReader::ResetShutdownCmds() {
  std::unique_lock<std::mutex> lock(cmd_msg_mutex_);
  command_shutdown_ = false;
}

bool MavlinkReader::WaitForShutdownCmd() {
  std::unique_lock<std::mutex> lock(cmd_msg_mutex_);
  while(command_shutdown_ == false) {
    std::cv_status ret = cmd_msg_cond_var_.wait_for(lock, std::chrono::seconds(1));

    // Return false if we have hit our timeout
    if (ret == std::cv_status::timeout)
      return false;
  }
  return true;
}
