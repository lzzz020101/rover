#include "rover_control/udp_communication.h"
#include <iostream>
#include <iomanip> // 用于 std::hex, std::dec 等格式化输出
#include <cmath>   // 用于 pow 函数

namespace rover_control
{
    // 构造函数
    UdpCommunication::UdpCommunication(std::string remote_ip, int remote_port, int local_port)
        : remote_ip_(remote_ip), remote_port_(remote_port), local_port_(local_port)
    {
        // 初始化本地地址
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = htonl(INADDR_ANY); // 监听所有网卡
        local_addr.sin_port = htons(local_port_);

        // 初始化目标地址
        memset(&remote_addr, 0, sizeof(remote_addr));
        remote_addr.sin_family = AF_INET;
        remote_addr.sin_addr.s_addr = inet_addr(remote_ip_.c_str());
        remote_addr.sin_port = htons(remote_port_);

        sin_size = sizeof(struct sockaddr_in);
    }

    // 析构函数
    UdpCommunication::~UdpCommunication()
    {
        udpClose();
    }

    // 建立连接
    bool UdpCommunication::udpConnect()
    {
        // 创建 Socket
        if ((server_sockfd = socket(PF_INET, SOCK_DGRAM, 0)) < 0)
        {
            std::cerr << "[UDP] Socket creation failed!" << std::endl;
            return false;
        }

        // 设置接收超时 (重要！防止阻塞卡死 ROS 节点)
        struct timeval read_timeout;
        read_timeout.tv_sec = 0;
        read_timeout.tv_usec = 10000; // 10ms 超时
        setsockopt(server_sockfd, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));

        // 绑定端口
        if (bind(server_sockfd, (struct sockaddr *)&local_addr, sizeof(struct sockaddr)) < 0)
        {
            std::cerr << "[UDP] Bind failed!" << std::endl;
            return false;
        }

        return true;
    }

    void UdpCommunication::udpClose()
    {
        close(server_sockfd);
    }

    // 发送数据
    bool UdpCommunication::udpSend(uint8_t *data, int sendSize)
    {
        int len = sendto(server_sockfd, data, sendSize, 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
        if (len < 0)
        {
            // std::cerr << "[UDP] Send failed!" << std::endl;
            return false;
        }
        return true;
    }

    // 接收数据
    int UdpCommunication::udpReceive(uint8_t *buf, int bufsize)
    {
        int len = recvfrom(server_sockfd, buf, bufsize, 0, (struct sockaddr *)&remote_addr, &sin_size);
        return len;
    }

    // 指令发送 (编码)
    bool UdpCommunication::sendCommand(float velocity, float angularVelocity)
    {
        uint8_t sendbuf[11];
        int velocity_c, angularVelocity_c;

        // 协议帧头
        sendbuf[0] = 0xFF;
        sendbuf[1] = 0x01;
        sendbuf[2] = 0x01;

        // 数据转换 (保留原协议：x1000 + 10000 偏移量)
        velocity_c = (int)(velocity * 1000) + 10000;
        angularVelocity_c = (int)(angularVelocity * 1000) + 10000;

        // 拆分字节 (保留原协议位移逻辑)
        // 线速度
        sendbuf[6] = velocity_c / (int)std::pow(16, 6);
        sendbuf[5] = velocity_c % (int)std::pow(16, 6) / (int)std::pow(16, 4);
        sendbuf[4] = velocity_c % (int)std::pow(16, 6) % (int)std::pow(16, 4) / (int)std::pow(16, 2);
        sendbuf[3] = velocity_c % (int)std::pow(16, 6) % (int)std::pow(16, 4) % (int)std::pow(16, 2);

        // 角速度
        sendbuf[10] = angularVelocity_c / (int)std::pow(16, 6);
        sendbuf[9] = angularVelocity_c % (int)std::pow(16, 6) / (int)std::pow(16, 4);
        sendbuf[8] = angularVelocity_c % (int)std::pow(16, 6) % (int)std::pow(16, 4) / (int)std::pow(16, 2);
        sendbuf[7] = angularVelocity_c % (int)std::pow(16, 6) % (int)std::pow(16, 4) % (int)std::pow(16, 2);

        return udpSend(sendbuf, 11);
    }

    // 状态接收 (解码 - 完整补全版)
    void UdpCommunication::chasisStatusDecoding(StatusInformation &status)
    {
        static int receiveid = 0; // 用于计数
        uint8_t recbuf[23];       // 协议规定包长 23
        int recSize = udpReceive(recbuf, 23);

        if (recSize > 0)
        {
            // 校验帧头 0xAA
            if ((int)recbuf[0] == 0xAA)
            {
                // 1. 状态位
                status.state = (int)recbuf[2];

                // 2. 线速度 (Bytes 3-6)
                status.velocity = ((int)recbuf[6] * (int)std::pow(16, 6) +
                                   (int)recbuf[5] * (int)std::pow(16, 4) +
                                   (int)recbuf[4] * (int)std::pow(16, 2) +
                                   (int)recbuf[3] - 10000) *
                                  0.001;

                // 3. 角速度 (Bytes 7-10)
                status.angularVelocity = ((int)recbuf[10] * (int)std::pow(16, 6) +
                                          (int)recbuf[9] * (int)std::pow(16, 4) +
                                          (int)recbuf[8] * (int)std::pow(16, 2) +
                                          (int)recbuf[7] - 10000) *
                                         0.001;

                // 4. 加速度 (Bytes 11-14)
                status.accleration = ((int)recbuf[14] * (int)std::pow(16, 6) +
                                      (int)recbuf[13] * (int)std::pow(16, 4) +
                                      (int)recbuf[12] * (int)std::pow(16, 2) +
                                      (int)recbuf[11] - 10000) *
                                     0.001;

                // 5. 左轮速度 (Bytes 15-18)
                status.wheelVelocity_left = ((int)recbuf[18] * (int)std::pow(16, 6) +
                                             (int)recbuf[17] * (int)std::pow(16, 4) +
                                             (int)recbuf[16] * (int)std::pow(16, 2) +
                                             (int)recbuf[15] - 10000) *
                                            0.001;

                // 6. 右轮速度 (Bytes 19-22)
                status.wheelVelocity_right = ((int)recbuf[22] * (int)std::pow(16, 6) +
                                              (int)recbuf[21] * (int)std::pow(16, 4) +
                                              (int)recbuf[20] * (int)std::pow(16, 2) +
                                              (int)recbuf[19] - 10000) *
                                             0.001;

                // 调试打印 (可选：为了不刷屏，可以设置每接收10次打印一次)
                /*
                if (receiveid % 10 == 0) {
                    std::cout << "--- Recv ID: " << receiveid << " ---" << std::endl;
                    std::cout << "Vel: " << status.velocity << " | Ang: " << status.angularVelocity << std::endl;
                    std::cout << "L: " << status.wheelVelocity_left << " | R: " << status.wheelVelocity_right << std::endl;
                }
                */
                receiveid++;
            }
            else
            {
                // std::cerr << "[UDP] Receive header error!" << std::endl;
                // 数据异常时清零，防止飞车
                status.state = -1;
                status.velocity = 0;
                status.angularVelocity = 0;
                status.accleration = 0;
                status.wheelVelocity_left = 0;
                status.wheelVelocity_right = 0;
            }
        }
    }
}