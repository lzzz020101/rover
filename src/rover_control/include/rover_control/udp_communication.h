#ifndef UDP_COMMUNICATION_H
#define UDP_COMMUNICATION_H

#include <iostream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cmath>

namespace rover_control
{
    class UdpCommunication
    {
    public:
        // 构造函数：传入目标IP，目标端口，本地监听端口
        UdpCommunication(std::string romote_ip, int remote_port, int local_port);
        ~UdpCommunication();

        // 定义下位机传回来的结构体
        struct StatusInformation
        {
            int state;
            float velocity;
            float accleration;
            float angularVelocity;
            float wheelVelocity_left;
            float wheelVelocity_right;
        };
        StatusInformation chasis_status;

        // 核心功能函数
        bool udpConnect();
        void udpClose();

        // 发送指令，运动控制节点调用的接口
        bool sendCommand(float velocity, float angularVelocity);

        // 接收并解码
        void chasisStatusDecoding(StatusInformation &status);

    private:
        bool udpSend(uint8_t *data, int sendSize);
        int udpReceive(uint8_t *buf, int bufsize);

        int server_sockfd;
        struct sockaddr_in local_addr;
        struct sockaddr_in remote_addr;
        socklen_t sin_size;

        std::string remote_ip_;
        int remote_port_;
        int local_port_;
    };
}; // rover_control

#endif // UDP_COMMUNICATION_H