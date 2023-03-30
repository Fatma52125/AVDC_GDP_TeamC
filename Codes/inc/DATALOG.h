#pragma once
#include <stdint.h>
#include <stdio.h>

//Log Sizes
#define LOG_SIZE 73
#define LOG_BYTES LOG_SIZE*4

//Log backups
#define LOG_BACKUPS 5

class DATALOG{
    private:
        //Functions
        int is_mounted(char * dev_path);
        
        //Variables
        FILE *my_file;
        uint8_t counter=0;
        uint8_t limit=100;
        uint8_t index=0;
        float data[LOG_BYTES];
        uint8_t bin_file_created=0;
        uint8_t bin_file_open=0;
        uint32_t iter=0;
        
        //Messages
        
    public:
        //Functions
        void begin(uint8_t new_limit);
        void open_bin();
        void close_bin();
        void write_bin();
        void log(float buffer[], uint16_t size);
        void check_mount();
        void reset_counter();
        void backup_logs(const char base_file[40],uint8_t n, uint8_t type);
        void backup_parameters(uint8_t n);
        
        //Variables
        uint8_t mounted=0;
        uint8_t disable_logging=0;
        
        //Messages
        
};