#pragma once
#include <Core.h>

class SENSORS{
    public:
        //General Functions
        void begin(uint8_t new_limit);
        void read();
        void read_extra();
        void update(uint8_t flag);
        void update_extra(uint8_t flag);
        void print(uint8_t flag);
        void print_extra(uint8_t flag);
        void print_rpy();
        void print_pqr();
        void print_acc();
        void print_mag();
        void print_baro();
        
        //General variables
        uint8_t counter=0;
        uint8_t counter_extra=0;
        uint8_t limit=10;
        uint8_t rpy_meas_available=0;
        uint8_t xyz_meas_available=0;
        uint8_t xyz_vel_meas_available=0;
        float xyz_meas[3];
        float xyz_vel_meas[3];
        float rpy_meas[3];
        
        //Barometer
        float baro_alt=0;
        float baro_ref=0;
        rc_bmp_data_t baro;
        void calibrate_baro();

        //GPS 
        // GPS gps;
        
        //Parameters
        float alpha_pqr=0;
        float alpha_pqr_dot=0;
        float alpha_acc=0;
        float alpha_mag=0;
        float alpha_rpy_meas=0;
        float alpha_z=0;
        float alpha_z_ref=0;
        float alpha_xyz_meas=0;
        uint8_t enable_magnetometer=0;
        float magnetic_declination=0;
        
        //IMU
        rc_mpu_data_t imu;
        rc_mpu_config_t conf;
        float gyro[3];
        float accel[3];
        float mag[3];
        float gyro_offsets[3];
        float accel_scales[3];
        float accel_offsets[3];
        float mag_scales[3];
        float mag_offsets[3];
        float mag_scales2[9];
        float mag_offsets2[3];
        uint8_t accel_cal=0;
        uint8_t gyro_cal=0;
        uint8_t mag_cal=0;
        void correct_orientation();
        void calibrate_gyro(uint8_t N);
        void calibrate_level(uint8_t N);
        void calibrate_accel(uint8_t N);
        uint16_t run_accel_test(float max[],float min[],uint8_t N,uint16_t index);
        void calibrate_mag();
        void calibrate_mag2();
        void load_or_save_accel_cal(uint8_t save);
        void load_or_save_mag_cal(uint8_t save);
        void load_or_save_mag_cal2(uint8_t save);
        uint8_t check_accel_cal();
        uint8_t check_mag_cal();
        uint8_t check_mag_cal2();
        void print_accel_cal();
        void print_mag_cal();
        void print_mag_cal2();
        
        //Parameter functions
        void load_parameters();
        void save_parameters(float parameters[]);
        uint8_t update_parameters(float parameters[]);
        void print_parameters();
        
        //Messages
    
    private:
        //Internal Functions
        
        //Variables
        
        //Memory Index 
        uint16_t accel_index=0;
        uint16_t mag_index=24;
        uint16_t mag2_index=48;
        uint16_t p1_index=100;
        
        //Messages
        char file_name[10]="SENSORS";
        char acc_cal_name[10]="Acc_Cal";
        char mag_cal_name[10]="Mag_Cal";
};
