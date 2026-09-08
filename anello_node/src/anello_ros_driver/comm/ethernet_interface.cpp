/********************************************************************************
 * File Name:   ethernet_interface.cpp
 * Description: definition for the ethernet_interface class.
 *
 * Author:      Austin Johnson
 * Date:        7/12/24
 *
 * License:     MIT License
 ********************************************************************************/

#include "../logging.h"
#include "ethernet_interface.h"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <poll.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdexcept>

ethernet_interface::ethernet_interface(const std::string &remote_ip_address, int remote_port, int local_port)
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
    if (local_port<0 || local_port>65535 || remote_port<=0 || remote_port>65535)
        throw std::invalid_argument("UDP port out of range");
    if (this->sockfd != -1)
    {
        close(this->sockfd);
        this->sockfd=-1;
    }

    if ((this->sockfd = socket(AF_INET, SOCK_DGRAM|SOCK_CLOEXEC|SOCK_NONBLOCK, 0)) < 0)
    {
        throw std::runtime_error("Ethernet socket creation failed: " +
                                 std::string(strerror(errno)));
    }

    memset(&(this->servaddr), 0, sizeof(this->servaddr));
    memset(&(this->cliaddr), 0, sizeof(this->cliaddr));

    this->servaddr.sin_family = AF_INET;
    this->servaddr.sin_addr.s_addr = INADDR_ANY;
    this->servaddr.sin_port = htons(this->local_port);

    if (bind(this->sockfd, reinterpret_cast<const sockaddr *>(&this->servaddr), sizeof(this->servaddr)) < 0)
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
    if (this->cliaddr.sin_addr.s_addr == INADDR_NONE)
    {
        close(this->sockfd);
        this->sockfd = -1;
        throw std::runtime_error("Invalid remote IP address: " +
                                 this->remote_ip_address);
    }
}

bool ethernet_interface::write_data(const char *buf, size_t buf_len)
{
    if (this->sockfd < 0) return false;
    ssize_t sent;
    do { sent=sendto(this->sockfd, buf, buf_len, 0,
           reinterpret_cast<const sockaddr *>(&this->cliaddr), sizeof(this->cliaddr)); } while (sent<0 && errno==EINTR);
    return sent>=0 && static_cast<size_t>(sent)==buf_len;
}

size_t ethernet_interface::get_data(char *buf, size_t buf_len)
{
    if (this->sockfd < 0 || buf_len == 0) return 0;

    // Receive into a separate source address: cliaddr stays fixed at the
    // configured device endpoint so writes cannot be redirected by an
    // arbitrary sender, and datagrams from other hosts are dropped.
    struct sockaddr_in srcaddr;
    socklen_t len = sizeof(srcaddr);
    memset(&srcaddr, 0, sizeof(srcaddr));

    // buf_len - 1 leaves room for the NUL terminator below.
    int n = recvfrom(this->sockfd, buf, buf_len - 1, MSG_DONTWAIT|MSG_TRUNC,
                     reinterpret_cast<sockaddr *>(&srcaddr), &len);
    if (n < 0)
    {
        return 0;
    }
    if (static_cast<size_t>(n)>=buf_len) { ++truncated_; return 0; }
    if (srcaddr.sin_addr.s_addr != this->cliaddr.sin_addr.s_addr)
    {
        return 0;
    }
    buf[n] = '\0';
    return static_cast<size_t>(n);
}

size_t ethernet_interface::get_data(char *buf, size_t buf_len, int timeout_ms)
{
    if (this->sockfd < 0) return 0;

    pollfd pending{sockfd,POLLIN,0};
    if (poll(&pending,1,timeout_ms<0?0:timeout_ms)<=0) return 0;
    return this->get_data(buf, buf_len);
}
