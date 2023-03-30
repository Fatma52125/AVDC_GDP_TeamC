#pragma once
// #include <Core.h>
#include <stdint.h>
#define RCOUT_PRUSS_RAM_BASE 0x4a302000
#define RCOUT_PRUSS_CTRL_BASE 0x4a324000
#define RCOUT_PRUSS_IRAM_BASE 0x4a338000
#define PWM_CHAN_COUNT 12
class INPUTS{
    public:
        //Functions
        void begin(uint16_t freq);
        void disarm_motors();
        void zero_servos();
        void disarm();
        void write();
        void write_motors();
        void write_servos();
        void calibrate_motors();
        
        //Variables
        static const uint8_t max_inputs=8;
        static const uint8_t number_motors=6;
        static const uint8_t number_servos=2;
        float motors[max_inputs];
        float angles[max_inputs];
        
        //Messages
    
    private:
        //Functions
        void set_freq(uint32_t chmask, uint16_t freq_hz);
        uint8_t check_repeated_ch();
        void enable_ch(uint8_t ch);
        void write_ch(uint8_t ch, uint16_t period_us);
        
        //Variables
        static const uint32_t TICK_PER_US = 200;
        static const uint32_t TICK_PER_S = 200000000;
        struct pwm {
            volatile uint32_t channelenable;
            struct {
            volatile uint32_t time_high;
            volatile uint32_t time_t;
            } channel[PWM_CHAN_COUNT];
        };
        volatile struct pwm *pwm;
        uint16_t max_us_motors=2000;
        uint16_t min_us_motors=1000;
        uint16_t max_us_servos=2000;
        uint16_t min_us_servos=1000;
        uint16_t trim_us_servos=1000;
        uint16_t range_us_motors=max_us_motors-min_us_motors;
        uint16_t range_us_servos=max_us_servos-min_us_servos;
        uint8_t motor_channels[number_motors]={0,1,2,3,4,5};
        uint8_t servo_channels[number_servos]={6,7};
        uint8_t reverse_servos[number_servos]={0,0};
        
        //Messages
        
};
