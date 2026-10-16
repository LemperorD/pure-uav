#include "communicator.hpp"
#include <cstdio>
#include <unistd.h>

int main() {
    pure_uav::Communicator comm;
    comm.setSystemId(2);

    if (!comm.start("/dev/ttyACM0", 115200)) {  
        printf("打开串口失败\n");
        return 1;
    }

    printf("等待心跳...\n");
    while (!comm.isConnected()) usleep(100000);
    printf("已连接，发送切模式指令...\n");

    comm.sendSetMode(6);  

    for (int i = 0; i < 50; i++) {
        uint8_t ack = comm.lastAckResult();
        printf("ACK = %u\n", ack);
        if (ack != 255) break;  // 255 = 还没收到 ACK
        usleep(100000);
    }
    return 0;
}
