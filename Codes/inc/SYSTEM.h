#pragma once
#include <stdint.h>
#include <stdio.h>
#include <Types.h>
class SYSTEM{
    public:
        //Variables
        uint8_t armed=0;
        uint8_t checks=0;
        float period=0;
        float period_eps=0.2;
        float exec_time=0;
        float Time=0;
        uint8_t limit=10;
        uint8_t counter=0;
        uint8_t toc_counter=0;
        uint8_t toc_func_counter=0;
        uint8_t sync_time_request=0;
        
        //Functions
        void begin(uint8_t new_limit);
        void update();
        void log_msg(const char msg[]);
        uint8_t arm_checks();
        void arm(reasons reason);
        void disarm(reasons reason);
        void run_checks(uint8_t flag);
        void print();
        void tic();
        float toc();
        uint8_t check_period();
        uint8_t check_exec_time();
        void sync_time();
        void wait(float seconds);
        void trigger_failsafe(failsafes failsafe);
        
        //Messages
        
    private:
        //Variables
        uint8_t file_created=0;
        uint8_t file_open=0;
        
        //Functions
        FILE *f;
};