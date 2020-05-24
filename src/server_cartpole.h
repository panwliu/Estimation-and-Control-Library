#ifndef GYM_GAZEBO_GAZEBOCARTPOLE_H_
#define GYM_GAZEBO_GAZEBOCARTPOLE_H_

#include <iostream>
#include <cstring>
#include <unistd.h>     // provide sleep(), usleep()
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <thread>

#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/ServerConfig.hh>
#include <ignition/transport/Node.hh>

#include <ignition/msgs/float_v.pb.h>

namespace ecl {

class ServerCartpole
{
    public:
        ServerCartpole(int _port_self, int _port_remote);
        ~ServerCartpole()
        {
            close(fd_);
        }
        int recvFromRL();
        void sendCommands();
        void StateTopicCB(const ignition::msgs::Float_V &_msg);
        void pauseSim();
        void unpauseSim();

    private:
        std::unique_ptr<ignition::gazebo::Server> server_;

        std::string states_topic_;
        std::string commands_topic_;
        ignition::transport::Node node_;
        ignition::transport::Node::Publisher commands_pub_;
        ignition::msgs::Float_V commands_msg_;

        int fd_;
        struct sockaddr_in sockaddr_local_;
        struct sockaddr_in sockaddr_remote_;
        socklen_t sockaddr_remote_len_;
        int msg_len_ = 30;
        float commands_[30];

};

}

#endif