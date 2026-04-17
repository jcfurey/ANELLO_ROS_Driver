/********************************************************************************
 * File Name:   ethernet_interface.cpp
 * Description: definition for the ethernet_interface class.
 *
 * Author:      Austin Johnson
 * Date:        7/12/24
 *
 * License:     MIT License
 ********************************************************************************/

#include "../main_anello_ros_driver.h"
#include "ethernet_interface.h"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdexcept>

ethernet_interface::ethernet_interface(std::string remote_ip_address, int remote_port, int local_port)
{
    this->remote_ip_address = remote_ip_address;
    this->local_port = local_port;
    this->remote_port = remote_port;
    this->sockfd = -1;
}

ethernet_interface::~ethernet_interface()
{
    if (this->sockfd != -1)
    {
        close(this->sockfd);
        this->sockfd = -1;
    }
}

void ethernet_interface::init()
{
    if (this->sockfd != -1)
    {
        close(this->sockfd);
    }

    if ((this->sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        throw std::runtime_error("Ethernet socket creation failed: " +
                                 std::string(strerror(errno)));
    }

    memset(&(this->servaddr), 0, sizeof(this->servaddr));
    memset(&(this->cliaddr), 0, sizeof(this->cliaddr));

    this->servaddr.sin_family = AF_INET;
    this->servaddr.sin_addr.s_addr = INADDR_ANY;
    this->servaddr.sin_port = htons(this->local_port);

    if (bind(this->sockfd, (const struct sockaddr *)&(this->servaddr), sizeof(this->servaddr)) < 0)
    {
        close(this->sockfd);
        this->sockfd = -1;
        throw std::runtime_error("Ethernet bind failed on port " +
                                 std::to_string(this->local_port) + ": " +
                                 std::string(strerror(errno)));
    }

    memset(&(this->cliaddr), 0, sizeof(this->cliaddr));
    this->cliaddr.sin_family = AF_INET;
    this->cliaddr.sin_port = htons(this->remote_port);
    this->cliaddr.sin_addr.s_addr = inet_addr(this->remote_ip_address.c_str());

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100 * 1000; // 100 ms
    setsockopt(this->sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
}

void ethernet_interface::write_data(const char *buf, size_t buf_len)
{
    if (this->sockfd < 0) return;
    sendto(this->sockfd, buf, buf_len, 0,
           (const struct sockaddr *)&(this->cliaddr), sizeof(this->cliaddr));
}

size_t ethernet_interface::get_data(char *buf, size_t buf_len)
{
    if (this->sockfd < 0) return 0;
    socklen_t len = sizeof(this->cliaddr);
    int n = recvfrom(this->sockfd, buf, buf_len, 0,
                     (struct sockaddr *)&(this->cliaddr), &len);
    if (n < 0)
    {
        return 0;
    }
    buf[n] = '\0';
    return static_cast<size_t>(n);
}
