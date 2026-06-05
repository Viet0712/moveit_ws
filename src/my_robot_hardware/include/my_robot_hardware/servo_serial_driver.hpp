#ifndef SERVO_SERIAL_DRIVER_HPP
#define SERVO_SERIAL_DRIVER_HPP

#include <string>
#include <map>
#include <cmath>
#include <iostream>
#include <sstream>
#include <cerrno>
#include <cstring>
#include <thread>
#include <chrono>
#include <mutex>
#include <vector>
#include <utility>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>

class ServoSerialDriver
{
public:
    ServoSerialDriver(const std::string &port, int baudrate)
        : port_(port),
          baudrate_(baudrate),
          serial_fd_(-1)
    {
    }

    ~ServoSerialDriver()
    {
        closePort();
    }

    int init()
    {
        serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

        if (serial_fd_ < 0)
        {
            std::cerr << "Failed to open serial port: " << port_
                      << " error: " << strerror(errno) << std::endl;
            return -1;
        }

        // Khóa độc quyền serial port.
        // Tránh Arduino IDE / terminal / process khác mở cùng port.
        if (ioctl(serial_fd_, TIOCEXCL) != 0)
        {
            std::cerr << "Warning: failed to set exclusive mode: "
                      << strerror(errno) << std::endl;
        }

        if (!configurePort(baudrate_))
        {
            std::cerr << "Failed to configure serial port." << std::endl;
            closePort();
            return -1;
        }

        // Quan trọng với ESP32-S3 USB CDC / JTAG serial
        setDtrRts(true, true);

        // Xóa dữ liệu cũ trong buffer
        tcflush(serial_fd_, TCIOFLUSH);

        // ESP32-S3 cần thời gian ổn định sau khi mở port
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        std::cout << "ServoSerialDriver connected to "
                  << port_ << " at " << baudrate_ << std::endl;

        // Test packet
        sendPing();

        return 0;
    }
    double mapJointRadianToDegree(double radian)
    {
        const double joint_min_rad = -1.57;
        const double joint_max_rad = 1.57;

        const double servo_min_deg = 0.0;
        const double servo_max_deg = 180.0;

        radian = clampDouble(radian, joint_min_rad, joint_max_rad);

        double degree =
            (radian - joint_min_rad) /
                (joint_max_rad - joint_min_rad) *
                (servo_max_deg - servo_min_deg) +
            servo_min_deg;

        return clampDouble(degree, servo_min_deg, servo_max_deg);
    }
    void closePort()
    {
        std::lock_guard<std::mutex> lock(write_mutex_);

        if (serial_fd_ >= 0)
        {
            tcdrain(serial_fd_);
            close(serial_fd_);
            serial_fd_ = -1;
        }
    }
    bool sendServoDegreesBatch(const std::vector<std::pair<int, double>> &commands)
    {
        if (commands.empty())
        {
            return true;
        }

        std::ostringstream ss;

        ss << "{";
        ss << "\"cmd\":\"set_all\",";
        ss << "\"servos\":[";

        bool first = true;

        for (const auto &cmd : commands)
        {
            int channel = cmd.first;
            double degree = clampDouble(cmd.second, 0.0, 180.0);

            if (channel < 0 || channel > 15)
            {
                std::cerr << "Invalid servo channel in batch: "
                          << channel << std::endl;
                continue;
            }

            if (!first)
            {
                ss << ",";
            }

            ss << "{";
            ss << "\"channel\":" << channel << ",";
            ss << "\"degree\":" << degree;
            ss << "}";

            first = false;
        }

        ss << "]";
        ss << "}\n";

        return writeString(ss.str());
    }

    void setTargetPositionRadian(int channel, double radian)
    {
        // const double joint_min_rad = -1.57;
        // const double joint_max_rad = 1.57;

        // const double servo_min_deg = 0.0;
        // const double servo_max_deg = 180.0;

        // radian = clampDouble(radian, joint_min_rad, joint_max_rad);

        // double degree =
        //     (radian - joint_min_rad) /
        //         (joint_max_rad - joint_min_rad) *
        //         (servo_max_deg - servo_min_deg) +
        //     servo_min_deg;

        // degree = clampDouble(degree, servo_min_deg, servo_max_deg);

        // if (sendServoDegree(channel, degree))
        // {
        //     last_command_rad_[channel] = radian;
        // }
        radian = clampDouble(radian, -1.57, 1.57);

        double degree = mapJointRadianToDegree(radian);

        if (sendServoDegree(channel, degree))
        {
            last_command_rad_[channel] = radian;
        }
    }

    double getMeasuredPositionRadian(int channel)
    {
        /*
         * Chưa có encoder feedback:
         * trả về command cuối cùng.
         */

        if (last_command_rad_.find(channel) == last_command_rad_.end())
        {
            return 0.0;
        }

        return last_command_rad_[channel];
    }

    void deactivate(int channel)
    {
        if (channel < 0 || channel > 15)
        {
            std::cerr << "Invalid servo channel: " << channel << std::endl;
            return;
        }

        std::ostringstream ss;

        ss << "{";
        ss << "\"cmd\":\"deactivate\",";
        ss << "\"channel\":" << channel;
        ss << "}\n";

        writeString(ss.str());
    }

    void setGripperPositionMeter(int channel, double position_m)
    {
        const double min_m = 0.0;
        const double max_m = 0.06;

        const double close_deg = 0.0;
        const double open_deg = 170.0;

        position_m = clampDouble(position_m, min_m, max_m);

        double ratio = (position_m - min_m) / (max_m - min_m);
        double degree = close_deg + ratio * (open_deg - close_deg);

        sendServoDegree(channel, degree);
    }

    bool sendPing()
    {
        std::string cmd = "{\"cmd\":\"ping\"}\n";
        return writeString(cmd);
    }

private:
    std::string port_;
    int baudrate_;
    int serial_fd_;

    std::mutex write_mutex_;

    std::map<int, double> last_command_rad_;
    std::map<int, double> last_command_deg_;

    double clampDouble(double value, double min_value, double max_value)
    {
        if (value < min_value)
        {
            return min_value;
        }

        if (value > max_value)
        {
            return max_value;
        }

        return value;
    }

    double radianToDegree(double radian)
    {
        return radian * 180.0 / M_PI;
    }

    speed_t baudrateToTermios(int baudrate)
    {
        switch (baudrate)
        {
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

        if (tcgetattr(serial_fd_, &tty) != 0)
        {
            std::cerr << "tcgetattr failed: " << strerror(errno) << std::endl;
            return false;
        }

        cfsetospeed(&tty, baudrateToTermios(baudrate));
        cfsetispeed(&tty, baudrateToTermios(baudrate));

        // Raw mode
        cfmakeraw(&tty);

        // 8-bit data
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;

        // Enable receiver, ignore modem control lines
        tty.c_cflag |= (CLOCAL | CREAD);

        // No parity
        tty.c_cflag &= ~(PARENB | PARODD);

        // 1 stop bit
        tty.c_cflag &= ~CSTOPB;

        // No hardware flow control
        tty.c_cflag &= ~CRTSCTS;

        // No software flow control
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);

        // Non-blocking-ish read config
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
        {
            std::cerr << "tcsetattr failed: " << strerror(errno) << std::endl;
            return false;
        }

        return true;
    }

    bool setDtrRts(bool dtr, bool rts)
    {
        if (serial_fd_ < 0)
        {
            std::cerr << "Serial port is not open, cannot set DTR/RTS." << std::endl;
            return false;
        }

        int status = 0;

        if (ioctl(serial_fd_, TIOCMGET, &status) != 0)
        {
            std::cerr << "Warning: failed to get modem status: "
                      << strerror(errno) << std::endl;
            return false;
        }

        if (dtr)
        {
            status |= TIOCM_DTR;
        }
        else
        {
            status &= ~TIOCM_DTR;
        }

        if (rts)
        {
            status |= TIOCM_RTS;
        }
        else
        {
            status &= ~TIOCM_RTS;
        }

        if (ioctl(serial_fd_, TIOCMSET, &status) != 0)
        {
            std::cerr << "Warning: failed to set DTR/RTS: "
                      << strerror(errno) << std::endl;
            return false;
        }

        return true;
    }

    bool sendServoDegree(int channel, double degree)
    {
        if (channel < 0 || channel > 15)
        {
            std::cerr << "Invalid servo channel: " << channel << std::endl;
            return false;
        }

        degree = clampDouble(degree, 0.0, 180.0);

        // Giảm spam serial:
        // Nếu góc thay đổi quá nhỏ thì không gửi.
        if (last_command_deg_.find(channel) != last_command_deg_.end())
        {
            double last_deg = last_command_deg_[channel];

            if (std::abs(last_deg - degree) < 0.5)
            {
                return true;
            }
        }

        std::ostringstream ss;

        ss << "{";
        ss << "\"cmd\":\"servo\",";
        ss << "\"channel\":" << channel << ",";
        ss << "\"degree\":" << degree;
        ss << "}\n";

        bool ok = writeString(ss.str());

        if (ok)
        {
            last_command_deg_[channel] = degree;
        }

        return ok;
    }

    bool writeString(const std::string &data)
    {
        std::lock_guard<std::mutex> lock(write_mutex_);

        if (serial_fd_ < 0)
        {
            std::cerr << "Serial port is not open." << std::endl;
            return false;
        }

        std::string log_data = data;
        if (!log_data.empty() && log_data.back() == '\n')
        {
            log_data.pop_back();
        }

        std::cout << "Serial TX: [" << log_data << "]" << std::endl;

        const char *buffer = data.c_str();
        size_t total_size = data.size();
        size_t total_written = 0;

        while (total_written < total_size)
        {
            ssize_t n = write(
                serial_fd_,
                buffer + total_written,
                total_size - total_written);

            if (n < 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }

                std::cerr << "Serial write failed: "
                          << strerror(errno) << std::endl;
                return false;
            }

            if (n == 0)
            {
                std::cerr << "Serial write returned 0 bytes." << std::endl;
                return false;
            }

            total_written += static_cast<size_t>(n);
        }

        if (tcdrain(serial_fd_) != 0)
        {
            std::cerr << "Warning: tcdrain failed: "
                      << strerror(errno) << std::endl;
            return false;
        }

        // Khoảng nghỉ rất nhỏ để ESP32 kịp xử lý dòng JSON.
        // Có thể giảm xuống 500 nếu muốn nhanh hơn.
        std::this_thread::sleep_for(std::chrono::microseconds(1000));

        return true;
    }
};

#endif