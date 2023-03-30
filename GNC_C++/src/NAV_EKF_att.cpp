#include <Definitions.h>

void NAV_EKF::reset_att(){
    quat.setZero();
    quat(0,0)=1;
    rpy.setZero();
    rpy_rad.setZero();
    pqr.setZero();
    pqr_rad.setZero();
    Patt.setIdentity();
    Patt*=default_Patt_ini;
    // acc_linear_hp.setZero();
    // acc_linear_prev.setZero();
    att_ini=1;
}

void NAV_EKF::set_pqr(float vec3[3],float alpha){
    //Map Input
    Map<Matrix<float,3,1>>new_pqr(vec3);

    //Copy Input
    pqr=alpha*pqr+(1-alpha)*(new_pqr-pqr_bias);

    // Trial to see how bias converges
    Matrix<float,3,1> added_bias;
    added_bias << 0, 0, 0;
    pqr += added_bias;

    //Convert to radians
    pqr_rad=pqr*deg_to_rad;
}

void NAV_EKF::set_acc(float vec3[3],float alpha){ 
    //Map Input
    Map<Matrix<float,3,1>>new_acc(vec3);

    //Copy Input
    acc=alpha*acc+(1-alpha)*new_acc;
}

void NAV_EKF::set_mag(float vec3[3],float alpha){  
    //Map Input
    Map<Matrix<float,3,1>>new_mag(vec3);

    //Copy Input
    mag=alpha*mag+(1-alpha)*new_mag;
}

void NAV_EKF::set_rpy_meas(float vec3[3],float alpha){
    //Map Input
    Matrix<float,3,1>new_rpy_meas;

    //RPY to quat
    float cr,cp,cy,sr,sp,sy;
    Matrix<float,4,1> quat2;
    cr = cos(vec3[0]/2.0f);
    cp = cos(vec3[1]/2.0f);
    cy = cos(vec3[2]/2.0f);
    sr = sin(vec3[0]/2.0f);
    sp = sin(vec3[1]/2.0f);
    sy = sin(vec3[2]/2.0f);
    quat2(0,0) = cr*cp*cy-sr*sp*sy;
    quat2(1,0) = sr*cp*cy+cr*sp*sy;
    quat2(2,0) = -sr*cp*sy+cr*sp*cy;
    quat2(3,0) = cr*cp*sy+sr*sp*cy;

    //From quat to RPY
    quat_to_rpy(&quat2(0,0),&new_rpy_meas(0,0));

    //From ENU to NED
    new_rpy_meas(1,0) = - new_rpy_meas(1,0);
    new_rpy_meas(2,0) = - new_rpy_meas(2,0);

    //Print RPY
    // cout << "RPY VICON = " << new_rpy_meas*rad_to_deg << endl;

    //Copy Input
    rpy_meas=alpha*rpy_meas+(1-alpha)*new_rpy_meas;
}

void NAV_EKF::propagate_att(){
    //Initialisation of covariances and relevant matrices
    if(!att_ini)reset_att();

    // Rates rot to quaternion
    Matrix<float,4,4> omega_b;
    omega_b << 0, -pqr_rad(0,0), -pqr_rad(1,0), -pqr_rad(2,0),
            pqr_rad(0,0), 0, pqr_rad(2,0), -pqr_rad(1,0),
            pqr_rad(1,0), -pqr_rad(2,0), 0, pqr_rad(0,0),
            pqr_rad(2,0), pqr_rad(1,0), -pqr_rad(0,0), 0;

    Matrix<float,4,4> Bw;
    Bw.setIdentity();
    Bw += (dt/2.0f)*omega_b;

    // STATE PROPAGATION
    quat = Bw * quat;
    quat /= quat.norm();

    //Convert rpy_rad to degrees
    quat_to_rpy(&quat(0,0),&rpy_rad(0,0),&rpy(0,0));

    // Quaternions to rate rot 
    Matrix<float,4,3> Bq;
    quat_to_rates_rot(&quat(0,0),&Bq(0,0));

    // COVARIANCE PROPAGATION
    Matrix<float,7,7> A;
    A.setIdentity();
    A.block<4,4>(0,0) = Bw;
    A.block<4,3>(0,4) = -0.5*dt*Bq;
    Matrix<float, 7,3> B;
    B.setZero();
    B.block<4,3>(0,0) = 0.5*dt*Bq;
    Patt = A * Patt * A.transpose() + B * Tpqr.asDiagonal() * B.transpose();
    Patt += Qatt.asDiagonal();
}

void NAV_EKF::correct_att3(uint8_t type){    

    //Sanity check
    if(!att_ini)return;
    
    //Define variables
    float q0,q1,q2,q3,q00,q11,q22,q33,q01,q02,q03,q12,q13,q23;
    q0=quat(0,0);
    q1=quat(1,0);
    q2=quat(2,0);
    q3=quat(3,0);
    q00=q0*q0;
    q11=q1*q1;
    q22=q2*q2;
    q33=q3*q3;
    q01=2*q0*q1;
    q02=2*q0*q2;
    q03=2*q0*q3;
    q12=2*q1*q2;
    q13=2*q1*q3;
    q23=2*q2*q3;
    
    //Select (0 - acc, 1 - rollpitch, 2 - mag, 3 - yaw)
    Matrix<float,3,1>hk,zk;//rpy_meas2;
    Matrix<float,3,7>Hk;
    Matrix<float,3,3>R;
    
    if(!type){
        //High pass filter 
        Matrix<float,3,3> DCM;
        quat_to_dcm(&quat(0,0),&DCM(0,0));
        Matrix<float,3,1> gravity;
        gravity << 0, 0, 1;
        float alpha_hf = 0.86;
        float K_hf = 1 / (1 - alpha_hf);
 
        // Measurement from the accelerometer
        Matrix<float,3,1>acc2;
        // acc2 = acc/acc.norm();

        Matrix<float,3,1> acc_linear_k,acc_linear_hp,acc_linear_prev; 
        acc_linear_k= DCM.transpose() * acc2 - gravity;
        acc_linear_hp = alpha_hf * acc_linear_hp + K_hf * (acc_linear_k - acc_linear_prev);
        acc_linear_prev = acc_linear_k;
        Matrix<float,3,1>acc_hf;
        acc_hf = DCM * acc_linear_hp;
        acc2 -= acc_hf;

        // // Measurement from the accelerometer
        // Matrix<float,3,1> acc2;
        acc2 = acc/acc.norm();
        zk= acc2;

        // Measurement equation
        hk<<q13-q02,
            q01+q23,
            q00-q11-q22+q33;

        // Jacobian
        Hk<<-2*q2, 2*q3, -2*q0, 2*q1, 0, 0, 0,
            2*q1, 2*q0, 2*q3, 2*q2, 0, 0, 0,
            2*q0,-2*q1,-2*q2, 2*q3, 0, 0, 0;

        // Measurement noise
        R = Racc.asDiagonal();
    }else if(type==1){
        // rpy_meas2=rpy_meas;
        // rpy_meas2(2,0)=rpy_rad(2,0);

        // Meauserement from the VICON
        rpy_to_dcm(&rpy_meas(0,0),&dcm(0,0));
        zk<<0,
            0,
            1;
        zk=dcm*zk;

        // Measurement equation
        hk<<q13 - q02,
            q01 + q23,
            q00 - q11 - q22 + q33;

        // Jacobian
        Hk<<-2*q2, 2*q3, -2*q0, 2*q1, 0, 0, 0,
            2*q1, 2*q0, 2*q3, 2*q2, 0, 0, 0,
            2*q0,-2*q1,-2*q2, 2*q3, 0, 0, 0;

        //Measurement noise
        R=Rrpy.asDiagonal();
    }else if(type==2){
        // Meauserement from the magnetometer
        mag /= mag.norm();
        zk= mag;

        // Measurement equation
        hk<<q12 + q03,
            q00 - q11 + q22 - q33,
            q23 - q01;
        
        // Jacobian
        Hk<<2*q3, 2*q2, 2*q1, 2*q0, 0, 0, 0,
            2*q0,-2*q1, 2*q2,-2*q3, 0, 0, 0,
            -2*q1,-2*q0,2*q3, 2*q2, 0, 0, 0;

        // Measurement noise
        R=Rmag.asDiagonal();
    }else if(type==3){        
        // rpy_meas2=rpy_meas;
        // rpy_meas2.block<2,1>(0,0)=rpy_rad.block<2,1>(0,0);

        // Meauserement from the VICON
        rpy_to_dcm(&rpy_meas(0,0),&dcm(0,0));
        zk<<0,
            1,
            0;
        zk=dcm*zk;

        // Measurement equation
        hk<<q12 + q03,
            q00 - q11 + q22 - q33,
            q23 - q01;
        
        // Jacobian
        Hk<<2*q3, 2*q2, 2*q1, 2*q0, 0, 0, 0,
            2*q0,-2*q1, 2*q2,-2*q3, 0, 0, 0,
            -2*q1,-2*q0,2*q3, 2*q2, 0, 0, 0;

        // Measurement noise 
        R = Rrpy.asDiagonal();
    }    

    //Calculation of KF
    Matrix<float,7,3> KF,PH;
    Matrix<float,3,3> S;
    PH = Patt*Hk.transpose();
    S = Hk*PH + R;
    KF = PH*S.inverse();

    //Innovation of the state
    Matrix<float,7,1> x;
    x = KF*(zk - hk);

    // UPDATE of the state and covariance
    if(type<2){
        x(3,0)=0;
    }else {
        x(1,0)=0;
        x(2,0)=0;
    }

    quat += x.block<4,1>(0,0); 
    pqr_rad_bias+=x.block<3,1>(4,0);
    pqr_bias = rad_to_deg*pqr_rad_bias;
    
    //Correct covariance
    Patt -= KF*Hk*Patt;

    //Normalise quat
    quat /= quat.norm(); 

    //Convert rpy_rad to degrees
    quat_to_rpy(&quat(0,0),&rpy_rad(0,0),&rpy(0,0));
}

void NAV_EKF::correct_acc(){
    correct_att3(0);
}

void NAV_EKF::correct_rpy(){
    correct_att3(1);
    correct_att3(3);
}

void NAV_EKF::correct_mag(){
    correct_att3(2);
}

void NAV_EKF::check_att_covariance(){
    //Sanity check 
    if(!att_ini)return;
    
    //Check norm 
    float norm=Patt.norm();
    float val;
    if(norm>default_Patt_ini){
        val=default_Patt_ini/norm;
        Patt*=val;
    }
    
    //Make symmetric!
    Matrix<float,7,7>P2;
    P2=Patt+Patt.transpose();
    Patt=0.5f*P2;
}

void NAV_EKF::get_rpy(){
    quat_to_rpy(&quat(0,0),&rpy_rad(0,0),&rpy(0,0));
}
