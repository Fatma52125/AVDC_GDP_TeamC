#include <Definitions.h>

void NAV_EKF::begin(){
    coms.send_msgs("Initialising Navigation EKF...\n");
    load_parameters();
    reset();
    coms.send_msgs("Navigation EKF Initialised!\n");
}

void NAV_EKF::reset(){
    reset_att();
    reset_xyz();
}

void NAV_EKF::load_parameters(){
    load_Qatt();
    load_Ratt();
    load_Tpqr();
    load_Qxyz();
    load_Rxyz();
    load_Tacc();
    load_Qz();

}

void NAV_EKF::save_parameters(float parameters[]){
    
}

uint8_t NAV_EKF::update_parameters(float parameters[]){  
    return true;
}

void NAV_EKF::print_parameters(){
    
}

void NAV_EKF::load_Qatt(){    
    float parameters[10];
    save_read_param_float(file_name,index_att_Q,parameters,10,0);
    update_Qatt(parameters);
}

void NAV_EKF::save_Qatt(float parameters[]){
    if(update_Qatt(parameters))save_read_param_float(file_name,index_att_Q,parameters,10,1);
}

uint8_t NAV_EKF::update_Qatt(float parameters[]){  
    Map<Matrix<float,10,1>>pars(parameters);
    if(pars.norm()<eps){
        sprintf(msgs,"Qatt %s",empty_set_msg);
        coms.send_msgs();
        return false;
    }
    Qatt=pars.block<7,1>(0,0);
    print_Qatt();

    return true;
}

void NAV_EKF::print_Qatt(){
    coms.send_msgs("Qatt = \t");
    coms.print_matrix(&Qatt(0,0),1,7,0);
}


void NAV_EKF::load_Tpqr(){    
    float parameters[10];
    save_read_param_float(file_name,index_att_Tpqr,parameters,10,0);
    update_Tpqr(parameters);
}

void NAV_EKF::save_Tpqr(float parameters[]){
    if(update_Tpqr(parameters))save_read_param_float(file_name,index_att_Tpqr,parameters,10,1);
}

uint8_t NAV_EKF::update_Tpqr(float parameters[]){  
    Map<Matrix<float,10,1>>pars(parameters);
    if(pars.norm()<eps){
        sprintf(msgs,"Tpqr %s",empty_set_msg);
        coms.send_msgs();
        return false;
    }
    Tpqr=pars.block<3,1>(0,0);
    print_Tpqr();

    return true;
}

void NAV_EKF::print_Tpqr(){
    coms.send_msgs("Tpqr = \t");
    coms.print_matrix(&Tpqr(0,0),1,3,0);
}

void NAV_EKF::load_Ratt(){    
    float parameters[10];
    save_read_param_float(file_name,index_att_R,parameters,10,0);
    update_Ratt(parameters);
}

void NAV_EKF::save_Ratt(float parameters[]){
    if(update_Ratt(parameters))save_read_param_float(file_name,index_att_R,parameters,10,1);
}

uint8_t NAV_EKF::update_Ratt(float parameters[]){  
    Map<Matrix<float,10,1>>pars(parameters);
    if(pars.norm()<eps){
        sprintf(msgs,"Ratt %s",empty_set_msg);
        coms.send_msgs();
        return false;
    }
    Racc=pars.block<3,1>(0,0);
    Rrpy=pars.block<3,1>(3,0);
    Rmag=pars.block<3,1>(6,0);
    print_Ratt();

    return true;
}

void NAV_EKF::print_Ratt(){
    coms.send_msgs("Racc = \t");
    coms.print_matrix(&Racc(0,0),1,3,0);
    coms.send_msgs("Rrpy = \t");
    coms.print_matrix(&Rrpy(0,0),1,3,0);
    coms.send_msgs("Rmag = \t");
    coms.print_matrix(&Rmag(0,0),1,3,0);
}

void NAV_EKF::load_Qxyz(){    
    float parameters[10];
    save_read_param_float(file_name,index_xyz_Q,parameters,10,0);
    update_Qxyz(parameters);
}

void NAV_EKF::save_Qxyz(float parameters[]){
    if(update_Qxyz(parameters))save_read_param_float(file_name,index_xyz_Q,parameters,10,1);
}

uint8_t NAV_EKF::update_Qxyz(float parameters[]){  
    Map<Matrix<float,10,1>>pars(parameters);
    if(pars.norm()<eps){
        sprintf(msgs,"Qxyz %s",empty_set_msg);
        coms.send_msgs();
        return false;
    }
    Qxyz=pars.block<9,1>(0,0);
    print_Qxyz();

    return true;
}

void NAV_EKF::print_Qxyz(){
    coms.send_msgs("Qxyz = \t");
    coms.print_matrix(&Qxyz(0,0),1,9,0);
}

void NAV_EKF::load_Rxyz(){    
    float parameters[10];
    save_read_param_float(file_name,index_xyz_R,parameters,10,0);
    update_Rxyz(parameters);
}

void NAV_EKF::save_Rxyz(float parameters[]){
    if(update_Rxyz(parameters))save_read_param_float(file_name,index_xyz_R,parameters,10,1);
}

uint8_t NAV_EKF::update_Rxyz(float parameters[]){  
    Map<Matrix<float,10,1>>pars(parameters);
    if(pars.norm()<eps){
        sprintf(msgs,"Rxyz %s",empty_set_msg);
        coms.send_msgs();
        return false;
    }
    Rxyz_meas=pars.block<3,1>(0,0);
    Rxyz_vel_meas=pars.block<3,1>(3,0);
    Rz_meas=pars.block<1,1>(6,0);
    print_Rxyz();

    return true;
}

void NAV_EKF::print_Rxyz(){
    coms.send_msgs("Rxyz_meas = \t");
    coms.print_matrix(&Rxyz_meas(0,0),1,3,0);
    coms.send_msgs("Rxyz_vel_meas = \t");
    coms.print_matrix(&Rxyz_vel_meas(0,0),1,3,0);
    coms.send_msgs("Rz_meas = \t");
    coms.print_matrix(&Rz_meas(0,0),1,1,0);
}

void NAV_EKF::load_Tacc(){    
    float parameters[10];
    save_read_param_float(file_name,index_xyz_Tacc,parameters,10,0);
    update_Tacc(parameters);
}

void NAV_EKF::save_Tacc(float parameters[]){
    if(update_Tacc(parameters))save_read_param_float(file_name,index_xyz_Tacc,parameters,10,1);
}

uint8_t NAV_EKF::update_Tacc(float parameters[]){  
    Map<Matrix<float,10,1>>pars(parameters);
    if(pars.norm()<eps){
        sprintf(msgs,"Tacc %s",empty_set_msg);
        coms.send_msgs();
        return false;
    }
    Tacc=pars.block<3,1>(0,0);
    print_Tacc();

    return true;
}

void NAV_EKF::print_Tacc(){
    coms.send_msgs("Tacc = \t");
    coms.print_matrix(&Tacc(0,0),1,3,0);
}


void NAV_EKF::load_Qz(){    
    float parameters[10];
    save_read_param_float(file_name,index_xyz_Qz,parameters,10,0);
    update_Qz(parameters);
}

void NAV_EKF::save_Qz(float parameters[]){
    if(update_Qz(parameters))save_read_param_float(file_name,index_xyz_Qz,parameters,10,1);
}

uint8_t NAV_EKF::update_Qz(float parameters[]){  
    Map<Matrix<float,10,1>>pars(parameters);
    if(pars.norm()<eps){
        sprintf(msgs,"Qz %s",empty_set_msg);
        coms.send_msgs();
        return false;
    }
    Qz=pars.block<2,1>(0,0);
    print_Qz();

    return true;
}

void NAV_EKF::print_Qz(){
    coms.send_msgs("Qz = \t");
    coms.print_matrix(&Qz(0,0),1,2,0);
}