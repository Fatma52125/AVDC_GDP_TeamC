#pragma once
#include <Core.h>
#include <UDP.h>
#include <UART.h>
#include <USB_UART.h>
class COMS{
    public:
        //Functions
        void begin(uint8_t new_limit);
        void send_data();
        void receive_data();
        void send_msgs();
        void send_msgs(const char msg[]);
        uint8_t any_message_available();
        void print_matrix(float mat[],uint8_t n_row, uint8_t n_cols, uint8_t row_major);
        void send_uav_data();
        void process_uav_msg(char buffer[]);
        void process_vicon();
        void process_jetson(char buffer[]);
        
        //Variable
        uint8_t disable_console=0;
        uint8_t disable_udp=0;
        uint8_t disable_uart=0;
        uint8_t counter=0;
        uint8_t limit=5;
        uint32_t timestep=0;
        uint32_t vicon_time_limit = 1;
        uint32_t gcs_time_limit = 1;

        //VICON Failsafes
        uint8_t vicon_active=0;
        failsafes vicon_fs= Land_fs;
        float vicon_fs_limit=2;
        uint8_t vicon_fs_triggered=0;
        float vicon_fs_stamp=0;
        void reset_vicon_fs();
        void update_vicon_fs();

        //GCS Failsafes
        uint8_t gcs_active=0;
        failsafes gcs_fs= Do_Nothing;
        float gcs_fs_limit=2;
        uint8_t gcs_fs_triggered=0;
        float gcs_fs_stamp=0;
        void reset_gcs_fs();
        void update_gcs_fs();
        float car_stamp = 0;
        
        //Messages
        
    private:
        //Functions
        void print_serial_int(int16_t var);
        void print_variable(float var[],uint8_t size, float scaler);
        void print_variable_int(int16_t var[],uint8_t size, float scaler);
        uint16_t calculate_checksum(uint8_t buf[],uint16_t size);
        void process_udp();
        uint8_t process_msg(char buffer_in1[],uint8_t &index_in1);
        void process_case();

        //General Message variables
        char header[5]="STR";
        char terminator[5]="END";
        uint8_t header_size=3;
        uint8_t terminator_size=3;

        //Jetson Nano Message variables
        char header_j[5] = "JST";
        uint8_t header_j_size = 3;
        uint32_t prev_jframe = 0;

        //Message Out Variables
        static const uint16_t buffer_out_size=256;
        char buffer_out[buffer_out_size];
        uint16_t index_out;
        int16_t chksm_out;
        uint8_t enable_send=1;

        //Message In Variables
        static const uint8_t int_buffer_in_size=20;
        struct __attribute__((__packed__)){
            uint8_t msg_id1;
            uint8_t msg_id2;
            uint8_t msg_id3;
            uint8_t msg_type;
            int16_t int_buffer_in[int_buffer_in_size];
            uint16_t chksm1;
            uint16_t chksm2;
        } msg_in;
        uint8_t income_total_size=48;
        static const uint8_t float_buffer_in_size=int_buffer_in_size*2;
        float float_buffer_in[float_buffer_in_size];

         //VICON Processing
        void process_vicon(char buffer[]);
        uint32_t prev_frame=0;
        float vicon_freq=120;
        uint8_t n_obj_max=10;
        uint8_t my_id=0;
        float vicon_stamp=0;
        float gcs_stamp = 0;
        uint8_t car_id=0;

        //Backoff variables
        float backoff_timer=0;
        float backoff_time=3;
        float receive_dt=0.02;

        //UART Variables
        USB_UART uart;
        // UART uart;
        uint8_t uart_bus=0;
        uint32_t uart_baudrate=57600;
        float uart_timeout=0.1;
        
        //UDP Variables
        UDP udp;
        uint16_t udp_port=51001;
        uint8_t udp_iter=20;
        struct sockaddr_in GCS_address, DEF_address,BROAD_address,VICON_address,Broadcast_address;
        
        //Messages
        char file_name[6]="COMS";
};