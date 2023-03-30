#pragma once
#include <Core.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#define USB_UART_MAX_BUS 16
#define USB_UART_STRING_BUF 64
#define USB_UART_MAX_READ_LEN 128
class USB_UART{
    public:
        //Functions
        int begin(uint8_t new_bus,uint32_t baudrate,float timeout_s, int canonical_en, int stop_bits, int parity_en);
        int close_bus();
        int flush();
        int write_bytes(uint8_t data[], size_t bytes);
        int read_bytes(uint8_t data[], size_t bytes);
        int read_bytes(size_t bytes);
        int bytes_available();
        
        //Variable
        uint8_t init=0;
        uint8_t bus=0;
        static const uint16_t buflen=1024;
        uint8_t buffer_in[buflen];
        int bytes_read;
        
        //Messages
        
    private:
        //Functions
        
        //Variables
        
        //Messages
        char file_name[6]="UART";
};