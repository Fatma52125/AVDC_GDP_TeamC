#pragma once
#include <stdint.h>
class PID{
    public:
        //Variables
        uint8_t init=0;
        float ek,ek1,ek_dot,ek_dot1,ei_k,r1,y1;

        //Parameters        
        struct __attribute__((__packed__)){
            float Kp=1;
            float Kd=0;
            float Ki=0.00001;
            float Imax=0;
            float alpha_d=0;
            float alpha_r=0;
            float alpha_e=0;
            float alpha_i=0;
            float max_ek=1000;
            float max_ek_dot=1000;
        }p;
        
        //Functions
        void begin(const char msg[]);
        void begin(const char msg[],float parameters[]);
        uint8_t update_parameters(float parameters[]);
        void load_parameters();
        void save_parameters(float parameters[]);
        void print_parameters();
        uint8_t check_parameters();        
        float update(float y,float r, float delta_t, uint8_t integrate, uint8_t kick);
        float update_yaw(float y,float r, float delta_t, uint8_t integrate,uint8_t kick);
        float update_xyz(float yk,float rk, float delta_t, uint8_t integrate,uint8_t kick, float y_dot);
        void reset();
        
        //Messages
    
    private:
        //Variables
        float time_stamp=0;
        
        //Functions
        
        //Messages
        char name[10]="DEF";
};