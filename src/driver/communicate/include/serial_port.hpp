#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <iostream>
#include <mutex>
#include <sys/types.h>

namespace pure_uav {

// 串口封装
class SerialPort {
public:
    SerialPort() = default;   //构造函数
    ~SerialPort();     //析构函数

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    
    bool open(const std::string& device, int baud);  //打开串口

    void close();   //关闭串口
    bool isOpen() const;    //判断串口是否打开

    // 非阻塞读：返回实际读取字节数；0 表示暂无数据；<0 表示错误。
    ssize_t read(uint8_t* buf, size_t len);

    // 【2026-10-29】等待可读并返回 poll 的 revents（用于发现 POLLHUP/POLLERR = 设备拔出）。
    // 返回 -1 表示 fd 无效/poll 失败。
    int pollIn(int timeout_ms, short* revents = nullptr);

    // 写：返回实际写入字节数。
    ssize_t write(const uint8_t* buf, size_t len);

    // 清空输入缓冲
    void flush();

private:
    void closeUnlocked();
    mutable std::mutex fd_mutex_;
    int fd_ = -1;
};

}  // namespace pure_uav
