#pragma once
#include <stdint.h>
#include <PID.h>
#include <Types.h>

//Flying states
enum flying_states{
    Idle = 0,
    Flying = 1,
    Landing = 2,
    Crashed = 3
};
extern char flying_msgs[4][10];

class CONTROL{
    public:
        //PID Variables
        PID roll_rate_pid;
        PID pitch_rate_pid;
        PID yaw_rate_pid;
        PID roll_pid;
        PID pitch_pid;
        PID yaw_pid;
        PID x_pid;
        PID y_pid;
        PID z_pid;
        PID x_vel_pid;
        PID y_vel_pid;  
        PID z_acc_pid; 
        
        //Core Functions
        void begin(uint8_t new_limit);
        void update(uint8_t flag);  
        void reset();      
        void set_flying_state(flying_states new_flying_state); 
        void reset_LMNZ();
        void reset_rpy_refs();
        void reset_rate_refs();
        void reset_xyz_refs();
        void reset_xyz_vel_refs();
        void reset_xyz_acc_refs();
        uint8_t set_xyz_refs(float xyz[]);
        uint8_t set_yaw_ref(float xyz[]);
        uint8_t set_xyz_refs_wmaxvel(float xyz[],float max_vel);
        void get_xyz_refs();
        void get_alt_ref();
        void get_xyz_vel_refs();
        void get_xyz_acc_refs();
        void get_rate_refs();
        void get_rpy_refs();
        void get_throttle_ref();
        void get_direct_refs();
        void print(uint8_t flag);
        void print_xyz_refs();
        void print_xyz_vel_refs();
        void print_xyz_acc_refs();
        void print_rpy_refs();
        void print_rate_refs();
        void print_LMNZ();
        void set_mode(modes mode, reasons reason);
        void check_new_mode(modes mode, reasons reason);
        void check_flying_state();
        uint8_t mode_checks(modes new_mode,reasons reason);
        uint8_t check_feasible_arena(float xyz[3]);
        uint8_t check_feasible_xyz(float xyz[3]);
        uint8_t check_idle();
        uint8_t check_neutral_throttle();
        void set_home(float xyz[3]);

        //Main control functions
        void run_xyz_controllers();
        void run_alt_controller();
        void run_xyz_vel_controllers();
        void run_xy_acc_controllers();
        void run_z_acc_controllers();
        void run_rpy_controllers();
        void convert_euler_rates_to_body_rates();
        void run_rate_controllers();
        void output();
        void limit_LMNZ();
        void allocate();
        void limit_motors();
        void convert_acc_to_eulerangles();
        void set_yaw();

        //Main mode functions
        void direct_mode();
        void acro_mode();
        void stabilise_mode(); 
        void alt_hold_mode();
        void pos_hold_mode();
        void guided_mode();
        void mission_mode();
        void track_mode();
        void loiter_mode();
        void land_mode();
        void break_mode();
        void rtl_mode();

        //Main parameter functions
        void load_p1();
        void load_p2();
        void load_p3();
        void load_p4();
        void save_p1(float buffer[]);
        void save_p2(float buffer[]);
        void save_p3(float buffer[]);
        void save_p4(float buffer[]);
        uint8_t update_p1(float buffer[]);
        uint8_t update_p2(float buffer[]);
        uint8_t update_p3(float buffer[]);
        uint8_t update_p4(float buffer[]);
        uint8_t check_array(float buffer[],uint8_t N);
        void print_p1();
        void print_p2();
        void print_p3();
        void print_p4();

        //Variables
        modes mode=Stabilise;
        modes prev_mode=Stabilise;
        flying_states flying_state=Idle;
        float mode_timer=0;
        float idle_timer=0;
        float idle_timer_limit=1;
        uint8_t limit=10;
        uint8_t counter=0;
        float xyz_refs[3];
        float xyz_vel_refs[3];
        float xyz_acc_refs[3];
        float Fxy_ref[2];  
        float rpy_refs[3];
        float rate_refs[3];
        float LMNZ[4];  
        uint8_t defer_xyz_ref_update=0;
        uint8_t defer_xyz_for_yaw=0;
        uint8_t defer_alt_ref_update=0; 
        uint8_t defer_xyz_vel_ref_update=0;
        uint8_t defer_xyz_acc_ref_update=0;
        uint8_t defer_rpy_ref_update=0;
        uint8_t defer_rate_ref_update=0;
        uint8_t defer_throttle_update=0;
        float ground_threshold=0.2;
        float arena_xyz[6]={4,4,10,12,0,2.8};      //(x1,y1,x2,y2,z1,z2)

        // Check feasible variable
        float xyz_feasible[3];

        // Tracking_variables
        uint8_t track_alive = 0;
        float tracking_xyz[3];
        uint8_t car_xyz_available=0;
        float car_xyz[3];

        // Return home variables
        float xyz_home[3];
        uint8_t home_set=0;


        // Mission variables
        static const uint8_t max_mission_waypoints=5;
        float mission_xyz[max_mission_waypoints][3];
        // float mission_time[max_mission_waypoints];
        // float mission_timers[max_mission_waypoints];
        // float mission_velocity[max_mission_waypoints];
        uint8_t mission_state=0;
        uint8_t mission_waypoints=0;
        void reset_mission();
        void print_mission();

        void load_x_waypoint();
        void save_x_waypoint(float parameters[]);
        uint8_t update_x_waypoint(float parameters[]);
        void print_x_waypoint();

        void load_y_waypoint();
        void save_y_waypoint(float parameters[]);
        uint8_t update_y_waypoint(float parameters[]);
        void print_y_waypoint();

        void load_z_waypoint();
        void save_z_waypoint(float parameters[]);
        uint8_t update_z_waypoint(float parameters[]);
        void print_z_waypoint();

        // void reset_mission_timers();

        //Parameters p1
        struct __attribute__((__packed__)){
            float Zmax=800;
            float Zmin=80;
            float LMmax=300;
            float Nmax=300;
            float RC_to_rp=0.09;
            float RC_to_rp_rates=0.6;
            float RC_to_yaw=0.36;
            float RC_to_yaw_rates=0.2;
            float RC_to_alt=0.01;
            float RC_to_xy=0.01;
        }p1;

        //Parameters p2
        struct __attribute__((__packed__)){
            float neutral_throttle=500;
            float neutral_deadzone=200;
            float motors_max = 1000;
            float motors_min = 100; 
            float land_velocity=1;
            float ground_threshold = 0.2;
            float neutral=200;
            float thrust_to_weight=2;
            float alpha_min_gz=0.7;
            float rp_lim = 20;
        }p2;

        //Parameters p3
        struct __attribute__((__packed__)){
            uint8_t control_yaw=1;
            uint8_t pars[39];
        }p3;

        //Parameters p4
        struct __attribute__((__packed__)){
            float T_min=100;
            float T_max=1000;
            float alpha_m=1;
            float pars[7];
        }p4;
        
    private:
        //Parameters Index
        uint16_t index_p1=0;
        uint16_t index_p2=40;
        uint16_t index_p3=80;
        uint16_t index_p4=120;
        uint16_t index_x = 160;
        uint16_t index_y = 200;
        uint16_t index_z = 240;
        
        //Messages
        char file_name[10]="CONTROL";
};