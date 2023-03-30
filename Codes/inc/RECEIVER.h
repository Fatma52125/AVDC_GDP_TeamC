#pragma once
// #include <Core.h>
#include <stdint.h>
#include <Types.h>
class RECEIVER{
    public:
        //Functions
        void begin(uint8_t new_limit);
        void update(uint8_t flag);
        void print();
        void print_raw();
        void calibrate();
        void load_or_save_calibration(uint8_t save);
        uint8_t check_calibration();
        void print_calibration();
        void set_offsets_and_scales();
        void print_offsets_and_scales();
        void apply_dead_zones();
        void limit_outputs();
        void reset_channels();
        void update_failsafes(int16_t buffer[]);
        void load_or_save_failsafes(uint8_t save);
        void print_failsafes();
    
        //Variables
        static const uint8_t active_ch=8;
        static const uint8_t n_ch=10;
        static const uint8_t coms_ch=4;
        float channels[n_ch];
        float coms_channels[coms_ch];
        float ch_max[n_ch];
        float ch_min[n_ch];
        uint8_t receiver_cal=0;
        uint8_t flight_state=0;
        uint8_t flight_mode=0;
        uint8_t aux1_state=0;
        uint8_t aux2_state=0;
        uint8_t throttle_ch=2;
        uint8_t roll_ch=0;
        uint8_t pitch_ch=1;
        uint8_t yaw_ch=3;
        uint8_t enable_ch=4;
        uint8_t mode_ch=5;
        uint8_t aux1_ch=6;
        uint8_t aux2_ch=7;
        uint8_t state_change_counter=0;
        uint8_t state_change_limit=5;  
        uint8_t mode_change_counter=0;
        uint8_t mode_change_limit=5;  
        uint8_t aux1_change_counter=0;
        uint8_t aux1_change_limit=5;
        uint8_t aux2_change_counter=0;
        uint8_t aux2_change_limit=5;            
        uint8_t force_disarm=1;

        //Failsafe variables
        static const uint8_t n_fs=2;
        failsafes fs[n_fs]={Disarm_fs,Disarm_fs};
        uint8_t fs_triggered[n_fs]={0,0};
        float fs_lim[n_fs]={0.1f,5.0f};
        float timer=0;
        uint8_t active=0;
        float bounds_timer=0;
        float bounds_timer_limit=1;
        uint8_t bounds=0;
        failsafes bounds_fs=Disarm_fs;
        float bounds2_timer=0;
        float bounds2_timer_limit=1;
        uint8_t bounds2=0;
        failsafes bounds2_fs=Disarm_fs;

        //UART 
        UART uart;
        uint8_t uart_bus=5;
        uint32_t uart_baudrate=115200;
        float uart_timeout=0.1;

        //Message buffer
        uint8_t msg_size=30;
        uint8_t n_max=10;
        uint16_t index=0;
        static int16_t const buffer_size=400;
        unsigned char buffer[buffer_size+10];

        //iBUS Structure
        __attribute__((__packed__))struct{
            uint16_t channels[14];
            uint16_t checksum;
        }iBUS;
        
    private:
        //Functions
        void cut_buffer(uint16_t i);
        uint8_t validate_iBUS_checksum();
        void process_iBUS();
        uint16_t limit_ch(uint16_t ch);
        void offset_channels();
        void check_signals();
        void set_angle_ch(uint8_t ch,float range, float dead_zone);
        void set_throttle_ch(uint8_t ch,float range, float dead_zone);
        void set_flight_state();
        void set_flight_mode();
        void check_aux1();
        void check_aux2();
        void reset_failsafes();
        void check_failsafes();
    
    
        //Variables

        //Mode variables
        static const uint8_t n_modes=3;
        modes mode[n_modes]={Stabilise,Alt_Hold,Land};
        float mode_limits[n_modes]={300,600,1000};
    
        //Variable
        uint16_t limits_index=0;
        uint16_t modes_index=40;
        uint16_t failsafes_index=50;
        uint16_t failsafes_timers_index=60;
        uint16_t bounds_fs_index=70;
        uint16_t bounds_fs_timers_index=80;
        uint16_t bounds2_fs_index=90;
        uint16_t bounds2_fs_timers_index=100;
        uint8_t counter=0;
        uint8_t limit=5;
        uint8_t ch_counter=0;
        uint16_t raw_channels[n_ch];
        uint16_t upper_limits[n_ch];
        uint16_t lower_limits[n_ch];
        float offsets[n_ch];
        float scales[n_ch];
        float dead_zones[n_ch];
        uint16_t pulse_limit=2700;              //Anything above this will be considered synching pulse

        //Indexes
    
        //Messages
        char file_name[10]="RECEIVER";
};