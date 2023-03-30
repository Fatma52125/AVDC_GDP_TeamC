#pragma once
#include <stdint.h>
#include <Dense>
using namespace Eigen;

class NAV_EKF{
    public:   
    //General Functions
    void begin();
    void reset();    
    void load_parameters();
    void save_parameters(float parameters[]);
    uint8_t update_parameters(float parameters[]);
    void print_parameters();

    
    void load_Qatt();
    void save_Qatt(float parameters[]);
    uint8_t update_Qatt(float parameters[]);
    void print_Qatt();

    void load_Ratt();
    void save_Ratt(float parameters[]);
    uint8_t update_Ratt(float parameters[]);
    void print_Ratt();

    void load_Tpqr();
    void save_Tpqr(float parameters[]);
    uint8_t update_Tpqr(float parameters[]);
    void print_Tpqr();

    void load_Qxyz();
    void save_Qxyz(float parameters[]);
    uint8_t update_Qxyz(float parameters[]);
    void print_Qxyz();

    void load_Rxyz();
    void save_Rxyz(float parameters[]);
    uint8_t update_Rxyz(float parameters[]);
    void print_Rxyz();

    void load_Tacc();
    void save_Tacc(float parameters[]);
    uint8_t update_Tacc(float parameters[]);
    void print_Tacc();

    void load_Qz();
    void save_Qz(float parameters[]);
    uint8_t update_Qz(float parameters[]);
    void print_Qz();

    //Attitude Variables
    Matrix<float,3,3>dcm;
    Matrix<float,3,1>rpy_meas;
    Matrix<float,3,1>mag;
    Matrix<float,3,1>acc;
    Matrix<float,4,1>quat;
    Matrix<float,3,1>rpy;
    Matrix<float,3,1>rpy_rad;
    Matrix<float,3,1>pqr;
    Matrix<float,3,1>pqr_bias;
    Matrix<float,3,1>pqr_rad;
    Matrix<float,3,1>pqr_rad_bias;
    Matrix<float,7,7>Patt; //Change: augmented state
    Matrix<float,7,1>Qatt; //Change: augmented state 
    Matrix<float,3,1>Tpqr;
    Matrix<float,3,1>Racc;
    Matrix<float,3,1>Rrpy;
    Matrix<float,3,1>Rmag;
    // Matrix<float,3,1>acc_linear_hp;
    // Matrix<float,3,1>acc_linear_prev;
    float default_Patt_ini=1000;
    uint8_t att_lock=0;
    uint8_t att_ini=0;

    //Attitude functions
    void set_pqr(float vec3[3],float alpha);
    void set_acc(float vec3[3],float alpha);
    void set_mag(float vec3[3],float alpha);
    void set_rpy_meas(float vec3[3],float alpha);
    void propagate_att();
    void correct_att3(uint8_t type);
    void correct_acc();
    void correct_rpy();
    void correct_mag();
    void get_rpy();
    void reset_att();
    void check_att_covariance();
    
    //Position Variables
    Matrix<float,3,1>xyz_meas;
    Matrix<float,3,1>xyz_vel_meas;
    Matrix<float,1,1>z_meas;
    Matrix<float,3,1>xyz;
    Matrix<float,3,1>xyz_vel;
    Matrix<float,3,1>xyz_acc;
    Matrix<float,3,1>acc_bias;
    Matrix<float,9,9>Pxyz;
    Matrix<float,2,2>Pz;
    Matrix<float,9,1>Qxyz;
    Matrix<float,2,1>Qz;
    Matrix<float,3,1>Tacc;
    Matrix<float,3,1>Rxyz_meas;
    Matrix<float,3,1>Rxyz_vel_meas;
    Matrix<float,1,1>Rz_meas;
    float default_Pxyz_ini=1000;
    float default_Pz_ini=1000;
    uint8_t xyz_lock=0;
    uint8_t xyz_ini=0;

    //Position functions
    void set_xyz_meas(float vec3[3],float alpha);
    void set_xyz_vel_meas(float vec3[3],float alpha);
    void set_z_meas(float new_z[1],float alpha);


    void propagate_xyz();
    void propagate_z();
    void correct_xyz3(uint8_t type);
    void correct_xyz();
    void correct_xyz_vel();
    void correct_z();
    void reset_xyz();
    void check_xyz_covariance();
    
    private:
    //Att Index Variables
    uint16_t index_att_Q=0;
    uint16_t index_att_R=40;
    // uint16_t index_att_Racc=40;
    // uint16_t index_att_Rrpy=80;
    // uint16_t index_att_Rmag=120;
    uint16_t index_att_Tpqr=80;
    
    //XYZ Index Variables
    uint16_t index_xyz_Q=300;
    uint16_t index_xyz_R=340;
    uint16_t index_xyz_Tacc=380;
    uint16_t index_xyz_Qz=420;
    // uint16_t index_xyz_Rxyz=340;
    // uint16_t index_xyz_Rvel=380;
    // uint16_t index_xyz_Rz=420;
    
    //Messages
    char empty_set_msg[25]="Parameter Set Empty!\n";
    char file_name[10]="NAV_EKF";
};