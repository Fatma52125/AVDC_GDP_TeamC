#include <UDP.h>

void UDP::begin(uint16_t new_port){
    printf("Initialising UDP...\n");

    //Copy port
    port=new_port;

    //Initialise
    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    int flags = fcntl(socket_fd, F_GETFL);
    flags |= O_NONBLOCK;
    fcntl(socket_fd, F_SETFL, flags);
    FD_ZERO(&read_fd_original);
    FD_ZERO(&read_fd);
    FD_SET(socket_fd,&read_fd_original);
    FD_SET(socket_fd,&read_fd);
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(port);
    server_address.sin_addr.s_addr = INADDR_ANY;
    bzero(&(server_address.sin_zero),8);
    bind(socket_fd,(struct sockaddr *)&server_address, sizeof(struct sockaddr));
    address_length = sizeof(struct sockaddr);
    int broadcastEnable=1;
    setsockopt(socket_fd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));

    printf("UDP Initialised!\n");
    init=1;

    sleep(1);
}

uint8_t UDP::read(){
    //Return if not initialised
    if(!init)return false;

    //Check if message available
    read_fd = read_fd_original;
    tv.tv_sec = 0;
    tv.tv_usec = 1;
    int receive = select(socket_fd+1, &read_fd, NULL, NULL, &tv);
        
    //Get bytes if available
    if (FD_ISSET(socket_fd, &read_fd)&&receive!=-1){ 
        FD_CLR(socket_fd, &read_fd);
        bytes_read = recvfrom(socket_fd,buffer_in,buflen,0,(struct sockaddr *)&client_address, &address_length);
        return true;
    }else return false;
}

void UDP::send(uint8_t msg[], uint16_t len, struct sockaddr_in to_address){    
    //Return if not initialised
    if(!init)return;

    //Write
    sendto(socket_fd,  msg, len , 0 , (struct sockaddr *) &to_address, address_length);
}