#pragma once
#include <Core.h>
class UDP{
    public:
        //Functions
        void begin(uint16_t new_port);
        uint8_t read();
        void send(uint8_t buffer[], uint16_t size, struct sockaddr_in to_address);
        
        //Variable
        uint8_t init=0;
        uint16_t port=51001;
        uint16_t bytes_read=0;
        static const uint16_t buflen=512;
        char buffer_in[buflen];  
        struct sockaddr_in server_address, client_address;
        
        //Messages
        
    private:
        //UDP Variables
        int socket_fd;
	    socklen_t address_length;
        fd_set read_fd;
        fd_set read_fd_original;
        fd_set read_console_fd;
        struct timeval tv;    
        
        //Messages
        char file_name[6]="UDP";
};