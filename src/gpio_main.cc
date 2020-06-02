/*

Example of programming GPIO from C or C++ using the sysfs interface on
a Raspberry Pi.

Will toggle GPIO78 (physical pin 18) at a 100 millisecond rate for 10
seconds and then exit.

Jeff Tranter <<a href="mailto:jtranter@ics.com">jtranter@ics.com</a>>

*/

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

int main()
{
    // Export the desired pin by writing to /sys/class/gpio/export

    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd == -1) {
        perror("Unable to open /sys/class/gpio/export");
        exit(1);
    }

    if (write(fd, "78", 2) != 2) {
        perror("Error writing to /sys/class/gpio/export");
        exit(1);
    }

    close(fd);

    // Set the pin to be an output by writing "out" to /sys/class/gpio/gpio78/direction

    sleep(1);
    fd = open("/sys/class/gpio/gpio78/direction", O_RDWR);
    if (fd == -1) {
        perror("Unable to open /sys/class/gpio/gpio78/direction");
        exit(1);
    }

    if (write(fd, "out", 3) != 3) {
        perror("Error writing to /sys/class/gpio/gpio78/direction");
        exit(1);
    }

    close(fd);

    fd = open("/sys/class/gpio/gpio78/value", O_RDWR);
    if (fd == -1) {
        perror("Unable to open /sys/class/gpio/gpio78/value");
        exit(1);
    }

    // Toggle LED 50 ms on, 50ms off, 100 times (10 seconds)

    for (int i = 0; i < 10; i++) {
        if (write(fd, "1", 1) != 1) {
            perror("Error writing to /sys/class/gpio/gpio78/value");
            exit(1);
        }
        usleep(5000000);

        if (write(fd, "0", 1) != 1) {
            perror("Error writing to /sys/class/gpio/gpio78/value");
            exit(1);
        }
        usleep(5000000);
    }

    close(fd);

    // Unexport the pin by writing to /sys/class/gpio/unexport

    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd == -1) {
        perror("Unable to open /sys/class/gpio/unexport");
        exit(1);
    }

    if (write(fd, "78", 2) != 2) {
        perror("Error writing to /sys/class/gpio/unexport");
        exit(1);
    }

    close(fd);

    // And exit
    return 0;
}