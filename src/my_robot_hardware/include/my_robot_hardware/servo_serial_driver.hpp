#ifndef SERVO_SERIAL_DRIVER_HPP
#define SERVO_SERIAL_DRIVER_HPP

#include <string>
#include <map>
#include <cmath>
#include <iostream>
#include <sstream>
#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

class ServoSerialDriver
{
public:
    ServoSerialDriver(const std::string & port, int baudrate)
    : port_(port), baudrate_(baudrate), serial_fd_(-1)
    {
    }

    ~ServoSerialDriver()
    {
        closePort();
    }

    int init()
    {
        serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

        if (serial_fd_ < 0) {
            std::cerr << "Failed to open serial port: " << port_
                      << " error: " << strerror(errno) << std::endl;
            return -1;
        }

        if (!configurePort(baudrate_)) {
            std::cerr << "Failed to configure serial port." << std::endl;
            closePort();
            return -1;
        }

        std::cout << "ServoSerialDriver connected to "
                  << port_ << " at " << baudrate_ << std::endl;

        return 0;
    }

    void closePort()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            serial_fd_ = -1;
        }
    }

    void setTargetPositionRadian(int channel, double radian)
    {
        double degree = radianToDegree(radian);

        // Servo thường giới hạn 0–180 độ
        if (degree < 0.0) {
            degree = 0.0;
        }

        if (degree > 180.0) {
            degree = 180.0;
        }

        last_command_rad_[channel] = radian;
        last_command_deg_[channel] = degree;

        std::ostringstream ss;
        ss << "S," << channel << "," << degree << "\n";

        writeString(ss.str());
    }

    double getMeasuredPositionRadian(int channel)
    {
        /*
         * Nếu chưa có encoder:
         * Trả về command cuối cùng.
         *
         * Nếu sau này ESP32 gửi feedback encoder về:
         * Sửa hàm này để đọc vị trí thật từ ESP32.
         */

        if (last_command_rad_.find(channel) == last_command_rad_.end()) {
            return 0.0;
        }

        return last_command_rad_[channel];
    }

    void deactivate(int channel)
    {
        /*
         * Gửi lệnh tắt servo nếu firmware ESP32 có hỗ trợ.
         * Ví dụ: D,0
         */

        std::ostringstream ss;
        ss << "D," << channel << "\n";

        writeString(ss.str());
    }

private:
    std::string port_;
    int baudrate_;
    int serial_fd_;

    std::map<int, double> last_command_rad_;
    std::map<int, double> last_command_deg_;

    double radianToDegree(double radian)
    {
        return radian * 180.0 / M_PI;
    }

    speed_t baudrateToTermios(int baudrate)
    {
        switch (baudrate) {
            case 9600:
                return B9600;
            case 19200:
                return B19200;
            case 38400:
                return B38400;
            case 57600:
                return B57600;
            case 115200:
                return B115200;
            case 230400:
                return B230400;
            case 460800:
                return B460800;
            case 921600:
                return B921600;
            default:
                std::cerr << "Unsupported baudrate: " << baudrate
                          << ", fallback to 115200" << std::endl;
                return B115200;
        }
    }

    bool configurePort(int baudrate)
    {
        struct termios tty;

        if (tcgetattr(serial_fd_, &tty) != 0) {
            std::cerr << "tcgetattr failed: " << strerror(errno) << std::endl;
            return false;
        }

        cfsetospeed(&tty, baudrateToTermios(baudrate));
        cfsetispeed(&tty, baudrateToTermios(baudrate));

        // 8-bit data
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;

        // Raw mode
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;

        // Non-blocking read config
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 5;

        // Disable software flow control
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);

        // Enable receiver, ignore modem control lines
        tty.c_cflag |= (CLOCAL | CREAD);

        // No parity
        tty.c_cflag &= ~(PARENB | PARODD);

        // 1 stop bit
        tty.c_cflag &= ~CSTOPB;

        // No hardware flow control
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            std::cerr << "tcsetattr failed: " << strerror(errno) << std::endl;
            return false;
        }

        return true;
    }

    bool writeString(const std::string & data)
    {
        if (serial_fd_ < 0) {
            std::cerr << "Serial port is not open." << std::endl;
            return false;
        }

        ssize_t n = write(serial_fd_, data.c_str(), data.size());

        if (n < 0) {
            std::cerr << "Serial write failed: " << strerror(errno) << std::endl;
            return false;
        }

        if (static_cast<size_t>(n) != data.size()) {
            std::cerr << "Serial write incomplete. Sent "
                      << n << " of " << data.size() << " bytes." << std::endl;
            return false;
        }

        return true;
    }
};

#endif