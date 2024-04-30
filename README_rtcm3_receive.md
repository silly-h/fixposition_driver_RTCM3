# Fixposition ROS Driver

Added the ability to accept RTCM3 topic messages and pass them in through the I/O interface(ROS1):
    If you want to pass in your own RTCM3 message via I/0, please provide the rostopic<std_msgs::ByteMultiArray> message for RTCM3 in the corresponding topic format, and the following is an example of converting an RTCM3 message accepted by from TCP to the corresponding rostopic（You can change the rtcm3_topic incoming data in the serial.yaml file）:
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


