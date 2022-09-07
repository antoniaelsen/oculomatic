#ifndef SRC_COMMS_TCP_
#define SRC_COMMS_TCP_

#include <iostream>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>

// tcpclient:
// A class that creates a socket to allow communication between machines
// This allows streaming data to another machine
// KEEP THIS AROUND FOR YOUR OWN GOOD!
class TcpClient {
 private:
  int status;
  struct addrinfo host_info;
  struct addrinfo *host_info_list;
  int socketfd;
  const char *msg;
  int len;
  ssize_t bytes_sent;
  ssize_t bytes_recieved;
  char incoming_data_buffer[100];


 public:
  void initialize(const char* hostname, const char* port);
};

#endif  // SRC_COMMS_TCP_
