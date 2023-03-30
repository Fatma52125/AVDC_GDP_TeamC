#pragma once
#include <Types.h>
#include <stdint.h>
#include <Dense>
using namespace Eigen;

class MODEL{
    public:
    //Variables
    uint8_t limit=10;
    uint8_t counter=0;
    uint8_t override=0;
    Matrix<float,3,1>xyz;
    Matrix<float,3,1>xyz_vel;
    Matrix<float,3,1>wind_xyz_vel;
    Matrix<float,3,1>xyz_acc;
    Matrix<float,3,1>acc;
    Matrix<float,4,1>quat;
    Matrix<float,4,3>quat_rot;
    Matrix<float,3,1>rpy;
    Matrix<float,3,1>rpy_rad;
    Matrix<float,3,1>pqr;
    Matrix<float,3,1>pqr_rad;
    Matrix<float,3,3>dcm;

    //Parameters        
    struct __attribute__((__packed__)){
        float mass=1;
        float Ixx=1;
        float Iyy=1;
        float Izz=1;
        float kt=1; //Ct in model 
        float kq=1; //Cq in model 
        float lx=1;
        float ly=1;
        float Cd=0.05;
        float radius=1;
    }p;
    
    //Functions
    void begin(uint8_t new_limit);
    void reset();
    void simulate(uint8_t flag);
    void set_override(uint8_t flag);
    void update_wind(float xyz[]);
    void load_parameters();
    void save_parameters(float parameters[]);
    uint8_t update_parameters(float parameters[]);
    void print_parameters();
    void print(uint8_t flag);
    void print_xyz();
    void print_xyz_vel();
    void print_xyz_acc();
    void print_quat();
    void print_rpy();
    void print_pqr();
    
    //Messages
    
    private:
    //Variables
    
    //Functions
    
    //Messages
    char file_name[10]="MODEL";
};