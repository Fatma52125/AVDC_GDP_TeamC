#pragma once
#include <stdint.h>
#include <stdio.h>
#include <Types.h>
class BATTERY{
    public:
        //Functions
        void begin();
        void update();
        void load_failsafes();
        void save_failsafes(float parameters[]);
        uint8_t update_failsafes(float parameters[]);
        void print_failsafes();
        void check_failsafes();
        void reset_failsafes();
        
        //Variables
        uint8_t battery_monitor_type=0;
        float max_voltage=8.5;
        float min_voltage=6.5;
        float voltage_scaler=18.181;
        float voltage=0;
        float filtered_voltage=0;
        float current_scaler=36;
        float current=0;
        float filtered_current=0;
        float alpha=0.995;
        uint8_t status=0;
        uint8_t fs_set=1;
        static const uint8_t n_fs=2;
        failsafes fs[n_fs] = {Land_fs,Disarm_fs};
        float fs_lim[n_fs]={6.5f,3.0f};
        uint8_t fs_triggered[n_fs]={0,0};
        
        //Messages
        
    private:
        //Functions
        
        //Variables
        uint16_t fs_index=0;
        
        //Messages
        char file_name[10]="BATTERY";
    
};