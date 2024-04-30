# Fixposition ROS Driver

Added the ability to accept RTCM3 topic messages and pass them in through the I/O interface or TCP(ROS1):
    If you want to pass in your own RTCM3 message , please provide the rostopic<std_msgs::ByteMultiArray> message for RTCM3 in the corresponding topic format, you can change the rtcm3_topic incoming data in the serial.yaml file:
    
        The following is an example of converting an RTCM3 message accepted by from TCP to the corresponding rostopic:
            #include <ros/ros.h>
            #include <std_msgs/ByteMultiArray.h>
            #include <sys/socket.h>
            #include <netinet/in.h>
            #include <arpa/inet.h>
            #include <unistd.h>
            #include <cstring>

            int main(int argc, char **argv) {
                ros::init(argc, argv, "rtcm3_receiver");
                ros::NodeHandle nh;

                ros::Publisher rtcm3_pub = nh.advertise<std_msgs::ByteMultiArray>("/rtcm3_data", 100);

                int sockfd;
                struct sockaddr_in addr;

                sockfd = socket(AF_INET, SOCK_STREAM, 0);
                if (sockfd < 0) {
                    ROS_ERROR("Failed to create socket");
                    return -1;
                }

                addr.sin_family = AF_INET;
                addr.sin_port = htons(23010);
                addr.sin_addr.s_addr = inet_addr("10.0.1.1");

                if (connect(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                    ROS_ERROR("Failed to connect to IP and port");
                    close(sockfd);
                    return -1;
                }

                char buffer[1024];
                std_msgs::ByteMultiArray rtcm3_msg;

                while (ros::ok()) {
                    memset(buffer, 0, sizeof(buffer));
                    ssize_t len = read(sockfd, buffer, sizeof(buffer) - 1);

                    if (len <= 0) {
                        ROS_ERROR("Failed to read from socket or connection closed");
                        break;
                    }

                    // 将接收到的数据填充到rtcm3_msg的data中
                    rtcm3_msg.data.clear();
                    rtcm3_msg.data.insert(rtcm3_msg.data.end(), buffer, buffer + len);

                    // 发布消息
                    rtcm3_pub.publish(rtcm3_msg);

                    // 处理ROS的回调函数
                    ros::spinOnce();
                }

                close(sockfd);
                return 0;
            }

        The following is an example of converting an RTCM3 message accepted by from UART to the corresponding rostopic:
            #include <ros/ros.h>
            #include <std_msgs/ByteMultiArray.h>
            #include <cstring>
            #include <fcntl.h> // for open
            #include <unistd.h> // for close
            #include <termios.h> // for termios

            int main(int argc, char **argv) {
                ros::init(argc, argv, "rtcm3_receiver");
                ros::NodeHandle nh;

                ros::Publisher rtcm3_pub = nh.advertise<std_msgs::ByteMultiArray>("/rtcm3_data", 100);

                int serial_fd;
                struct termios tty;

                // 打开串行端口
                serial_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
                if (serial_fd < 0) {
                    ROS_ERROR("Failed to open serial port");
                    return -1;
                }

                // 配置串行端口
                memset(&tty, 0, sizeof(tty));
                if (tcgetattr(serial_fd, &tty) != 0) {
                    ROS_ERROR("Failed to get serial attributes");
                    close(serial_fd);
                    return -1;
                }

                cfsetospeed(&tty, B115200); // 设置波特率为115200
                cfsetispeed(&tty, B115200);

                tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
                // 禁用IGNBRK忽略中断、IXON软件流控、ICRNL映射CR到NL等
                tty.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
                // 无输出处理
                tty.c_oflag = 0;
                // 禁用回显、回显NL、规范模式等
                tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
                // 设置读取阻塞
                tty.c_cc[VMIN] = 0;
                tty.c_cc[VTIME] = 10; // 1秒超时（10 deciseconds）

                if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
                    ROS_ERROR("Failed to set serial attributes");
                    close(serial_fd);
                    return -1;
                }

                char buffer[1024];
                std_msgs::ByteMultiArray rtcm3_msg;

                while (ros::ok()) {
                    memset(buffer, 0, sizeof(buffer));
                    ssize_t len = read(serial_fd, buffer, sizeof(buffer) - 1);

                    if (len <= 0) {
                        ROS_ERROR("Failed to read from serial port or connection closed");
                        break;
                    }

                    // 将接收到的数据填充到rtcm3_msg的data中
                    rtcm3_msg.data.clear();
                    rtcm3_msg.data.insert(rtcm3_msg.data.end(), buffer, buffer + len);

                    // 发布消息
                    rtcm3_pub.publish(rtcm3_msg);

                    // 处理ROS的回调函数
                    ros::spinOnce();
                }

                close(serial_fd);
                return 0;
            }
