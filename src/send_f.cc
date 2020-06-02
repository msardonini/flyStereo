#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

int main() {
  // Open the device
  int serial_dev_ = open("/dev/ttyTHS1", O_RDWR | O_NOCTTY | O_NDELAY);
  if(serial_dev_ == -1) {
    std::cerr << "Failed to open the serial device" << std::endl;
    return -1;
  }

  struct termios  config;
  //
  // Get the current configuration of the serial interface
  //
  if(tcgetattr(serial_dev_, &config) < 0) {
    std::cerr << "Failed to get the serial device attributes" << std::endl;
    return -1;
  }

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
  config.c_cc[VMIN]  = 1;
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

  unsigned char val = 0x00;
  while (1) {
    write(serial_dev_, &val, 1);
  }



  return 0;
}