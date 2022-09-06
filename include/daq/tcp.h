
// tcpclient:
// A class that creates a socket to allow communication between machines
// This allows streaming data to another machine
// KEEP THIS AROUND FOR YOUR OWN GOOD!
class tcpclient{
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
  void initialize(const char* hostname, const char* port){
    // need to block out memory and set to 0s
    memset(&host_info, 0, sizeof host_info);
    std::cout << "Setting up structs..." << std::endl;
    host_info.ai_family = AF_UNSPEC;
    host_info.ai_socktype = SOCK_STREAM;
    status = getaddrinfo(hostname, port, &host_info, &host_info_list);
    if (status != 0) std::cout << "getaddrinfo error" << gai_strerror(status);

    std::cout << "Creating a socket... " << std::endl;
    socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype, host_info_list->ai_protocol);
    if (socketfd == -1) std::cout << "Socket error";

    std::cout << "Connecting..." << std::endl;
    status = connect(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
    if (status == -1) std::cout << "Connect error";
  }
};