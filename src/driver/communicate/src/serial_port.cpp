#include "serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace pure_uav {

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& device, int baud) { //打开串口
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);  //可读可写，非阻塞
    if (fd_ < 0) {
        std::cout << "open failed: "
              << std::strerror(errno)
              << ", errno = " << errno
              << std::endl;
        return false;
    }

    termios tty{};  //保存串口参数的结构体
    if (tcgetattr(fd_, &tty) != 0) {   //将fd_对应的串口的参数拷贝进tty
        close();
        return false;
    }

    // 波特率映射
    speed_t speed;
    switch (baud) {
        case 57600:
            speed = B57600;
            break;
        case 115200:
            speed = B115200;
            break;
        case 230400:
            speed = B230400;
            break;
        case 460800:
            speed = B460800;
            break;
        case 921600:
            speed = B921600;
            break;
        default:
            close();  // 不支持的波特率，明确失败而非静默映射
            return false;
    }
    cfsetispeed(&tty, speed); //发送波特率
    cfsetospeed(&tty, speed);  //接收波特率

    // 8N1，无流控
    tty.c_cflag &= ~PARENB;  // 无校验位
    tty.c_cflag &= ~CSTOPB;  // 1 停止位
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      // 8 数据位
    tty.c_cflag &= ~CRTSCTS; // 关硬件流控
    tty.c_cflag |= CREAD | CLOCAL;

    // raw mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // 关软件流控
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    // 非阻塞：VMIN=0, VTIME=0
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        close();
        return false;
    }

    return true;
}

void SerialPort::close() {   //关闭串口
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

ssize_t SerialPort::read(uint8_t* buf, size_t len) {    //读取返回读取字节数
    return ::read(fd_, buf, len);
}

ssize_t SerialPort::write(const uint8_t* buf, size_t len) {  //发送返回发送字节数
    return ::write(fd_, buf, len);
}

void SerialPort::flush() {    
    if (fd_ >= 0) {
        tcflush(fd_, TCIFLUSH);
    }
}

}  // namespace pure_uav
