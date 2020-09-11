#include "fly_stereo/sensor_io/mavlink_reader.h"


#include <sys/stat.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <string>
#include <chrono>
#include <iostream>

MavlinkReader::MavlinkReader() {}

MavlinkReader::~MavlinkReader() {
  is_running_.store(false);

  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }

  if (serial_dev_ != 0) {
    close(serial_dev_);
  }

  if (replay_file_fd_ != 0) {
    close(replay_file_fd_);
  }
}

int MavlinkReader::Init(YAML::Node input_params) {
  // Open the device
  serial_dev_ = open(input_params["device"].as<std::string>().c_str(),
    O_RDWR | O_NOCTTY | O_NDELAY);
  if(serial_dev_ == -1) {
    std::cerr << "Failed to open the serial device" << std::endl;
    return -1;
  }


  if (input_params["replay_mode"].as<bool>()) {
    is_running_.store(true);
    // fcntl(serial_dev_, F_SETFL, fcntl(serial_dev_, F_GETFL) | O_NONBLOCK);
    reader_thread_ = std::thread(&MavlinkReader::SerialReadThread, this);
    return 0;
  }

  struct termios  config;
  //
  // Get the current configuration of the serial interface
  //
  if(tcgetattr(serial_dev_, &config) < 0) {
    std::cerr << "Failed to get the serial device attributes" << std::endl;
    return -1;
  }

  fcntl(serial_dev_, F_SETFL, fcntl(serial_dev_, F_GETFL) & ~O_NONBLOCK);

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
    std::cerr << "Failed to set the baudrate on the serial port!" << std::endl;
    return -1;
  }

  //
  // Finally, apply the configuration
  //
  if(tcsetattr(serial_dev_, TCSAFLUSH, &config) < 0) {
    std::cerr << "Failed to set the baudrate on the serial port!" << std::endl;
    return -1;
  }


  // Now that our serial port is intialized, send the signal to reset the trigger counters
  // in case the flight program has been running already
  SendCounterReset();


  // If we want to save this data for replay, open a file to do this
  if (input_params["replay_imu_data_file"] && !input_params["replay_mode"].as<bool>()) {
    std::string replay_file = input_params["replay_imu_data_file"].as<std::string>();

    replay_file_fd_ = open(replay_file.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_CREAT, 0660);
    // int chmod_ret = chmod(replay_file.c_str(), S_IRWXU | S_IRWXG);
    if (replay_file_fd_ <= 0) {
      std::cerr << "error opening log file!\n";
    }
  }

  is_running_.store(true);
  reader_thread_ = std::thread(&MavlinkReader::SerialReadThread, this);

  return 0;
}

void MavlinkReader::SendCounterReset() {
  mavlink_message_t msg;
  mavlink_reset_counters_t reset_msg;
  uint8_t buf[1024];
  reset_msg.timestamp_us = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::
    system_clock::now().time_since_epoch()).count();

  mavlink_msg_reset_counters_encode(1, 200, &msg, &reset_msg);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  write(serial_dev_, buf, len);
}


void MavlinkReader::SerialReadThread() {
  unsigned char buf[1024];
  size_t buf_size = 1024;
  std::chrono::time_point<std::chrono::system_clock> time_end;
  while (is_running_.load()) {
    ssize_t ret = read(serial_dev_, buf, buf_size);

    if (ret < 0) {
      std::cerr << "Error on read(), errno: " << strerror(errno) << std::endl;
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
            std::cout << "RECEIVED COMMAND MESSAGE WOOO!" << std::endl;
            if (mav_cmd.engage) {
              command_on_ = true;
              cmd_msg_cond_var_.notify_one();
            }
          }
          default:
            std::cerr << "Unrecognized message with ID:" << static_cast<int>(
              mav_message.msgid) << std::endl;
        }
      }
    }

    // If we have requested to record a replay file, write the data to it
    if (replay_file_fd_ > 0) {
      write(replay_file_fd_, buf, ret);
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

  std::mutex cmd_msg_mutex_;
  std::condition_variable cmd_msg_cond_var_;


  while(command_on_ == false) {
    std::cv_status ret = cmd_msg_cond_var_.wait_for(lock, std::chrono::seconds(1));

    // Return false if we have hit our timeout
    if (ret == std::cv_status::timeout)
      return false;
  }

  return true;
}

