#include <Definitions.h>

void NAV_EKF::reset_xyz(){
    xyz.setZero();
    xyz_vel.setZero();
    xyz_acc.setZero();
    Pxyz.setIdentity();
    Pz.setIdentity();
    Pxyz*=default_Pxyz_ini;
    Pz*=default_Pz_ini;
    xyz_ini=1;
}

void NAV_EKF::set_xyz_meas(float vec3[3],float alpha){
    //Map input
    Map<Matrix<float,3,1>>new_xyz_meas(vec3);
    
    //Copy Input
    xyz_meas=alpha*xyz_meas+(1-alpha)*new_xyz_meas;
}

void NAV_EKF::set_xyz_vel_meas(float vec3[3],float alpha){    
    //Map input
    Map<Matrix<float,3,1>>new_xyz_vel_meas(vec3);

    //Copy Input
    xyz_vel_meas=alpha*xyz_vel_meas+(1-alpha)*new_xyz_vel_meas;
}

void NAV_EKF::set_z_meas(float vec1[1],float alpha){         
    //Map input
    Map<Matrix<float,1,1>>new_z_meas(vec1);
    
    //Copy Input
    z_meas=alpha*z_meas+(1-alpha)*new_z_meas;
}

void NAV_EKF::propagate_xyz(){
    //Initialisation of covariance and relevant matrices
    if(!xyz_ini)reset_xyz();

    //Copy acc
    Matrix<float,3,1>acc2;
    acc2=-acc;

    //DCM with the current quaternions
    quat_to_dcm(&quat(0,0),&dcm(0,0));

    // Removal of external acceleration
    xyz_acc = dcm.transpose()*(acc2-acc_bias);
    xyz_acc(2) += g0;

    //From NED to ENU, x and y components
    xyz_acc(1,0)=-xyz_acc(1,0);
    xyz_acc(2,0)=-xyz_acc(2,0);


    //Propagation of the states
    xyz += dt*xyz_vel + dt*dt/2.0f*xyz_acc;
    xyz_vel += dt*xyz_acc;

    // Propagation of the covariance
    // Matrix<float,3,3> I;
    Matrix<float,3,1>Dt;
    Dt.setConstant(dt);
    Matrix<float,9,9> A;
    A.setIdentity();
    A.block<3, 3>(0,3) = Dt.asDiagonal();
    A.block<3,3>(0,6) = -dt/2.0f*Dt.asDiagonal();
    A.block<3,3>(3,6) = -1* Dt.asDiagonal();
    Matrix<float,9,3> B;
    B.setZero();
    B.block<3,3>(0,0) = dt/2.0f*Dt.asDiagonal();;
    B.block<3,3>(3,0) = Dt.asDiagonal();
    Pxyz = A * Pxyz * A.transpose() + B *Tacc.asDiagonal() * B.transpose();
    Pxyz += Qxyz.asDiagonal();

}

void NAV_EKF::propagate_z(){
    //Initialisation of covariance and relevant matrices
    // if(!xyz_ini){
    //     reset_xyz();
    //     xyz_ini=1;
    // }
    //Copy acceleration changing from NED to ENU
    Matrix<float,3,1>acc2;
    acc2=-acc;

    //DCM with the current quaternions
    quat_to_dcm(&quat(0,0),&dcm(0,0));

    // Removal of external acceleration
    xyz_acc = dcm.transpose()*(acc2-acc_bias);
    xyz_acc(2) += g0;

    //From NED to ENU, z components
    xyz_acc(2,0)=-xyz_acc(2,0);

    //Propagation of the states
    xyz(2) += dt*xyz_vel(2) + dt*dt/2.0f*xyz_acc(2);
    xyz_vel(2) += dt*xyz_acc(2);


    //Propagation of the covariance
    Matrix<float,2,2> A;
    A.setIdentity();
    A(0,1) = dt;
    Matrix<float,2,1> B;
    B.setZero();
    B(0,0) = dt*dt/2.0f;
    B(1,0) = dt;
    
    Pz = A * Pz * A.transpose() + B *Tacc(2,0) * B.transpose();
    Pz += Qz.asDiagonal();

}

void NAV_EKF::correct_xyz3(uint8_t type){
    //Sanity check
    if(!xyz_ini)return;

    //Correction phase of EKF

    //Definition of the variables
    Matrix<float,3,1> zk,hk;//rpy_meas2;
    Matrix<float,3,9> Hk;
    Matrix<float,3,3> R;
    hk.setZero();
    zk.setZero();
    // Hk.setZero();
    // R=Rxyz_meas.asDiagonal();
    if(!type){
        //Position from the VICON
        zk = xyz_meas;

        //Measurement equation
        hk = xyz;

        //Jacobian matrix
        Hk = Hk.Zero();
        Hk.block<3,3>(0,0).setIdentity();

        //Measurement noise
        R = Rxyz_meas.asDiagonal();
    }else if(type==1){
        //Velocity from the VICON
        zk = xyz_vel_meas;

        //Measurement equation
        hk = xyz_vel;

        //Jacobian matrix
        Hk = Hk.Zero();
        Hk.block<3,3>(0,3).setIdentity();

        //Measurement noise
        R = Rxyz_vel_meas.asDiagonal();
    }
    //Calculation of KF
    Matrix<float, 9,3> KF,PH;
    Matrix<float, 3,3> S;
    PH = Pxyz*Hk.transpose();
    S = Hk*PH + R;
    KF = PH*S.inverse();

    //Innovation of the state
    Matrix<float,9,1> x;
    x = KF*(zk - hk);

    //Update of the states
    if(!type){
        xyz += x.block<3,1>(0,0);        
        xyz_vel += x.block<3,1>(3,0);
        x.block<2,1>(7,0) = -x.block<2,1>(7,0);
        acc_bias += dcm*x.block<3,1>(6,0);
    }else if(type==1){
        xyz += x.block<3,1>(0,0);        
        xyz_vel += x.block<3,1>(3,0);
        x.block<2,1>(7,0) = -x.block<2,1>(7,0);
        acc_bias += dcm*x.block<3,1>(6,0);
    }

    //UPDATE of the Covariance
    Pxyz -= KF*Hk*Pxyz;    
}

void NAV_EKF::correct_xyz_vel(){   
    correct_xyz3(1); 
}

void NAV_EKF::correct_xyz(){  
    correct_xyz3(0);
}

void NAV_EKF::correct_z(){  
    //Sanity check
    if(!xyz_ini)return;   
    float zk,hk;
    Matrix<float,1,2> Hk;
    Matrix<float,1,1> R;
    //Correction phase of EKF (z axis only) 
    zk = z_meas(0,0);

    //Measurement equation
    hk = xyz(2,0);

    //Jacobian
    Hk = Hk.Zero();
    Hk(0,0)  = 1;

    //Noise Measurement
    R = Rz_meas;

    //Calculation of KF
    Matrix<float, 2,1> KF,PH;
    Matrix<float,1 ,1> S;
    PH = Pz*Hk.transpose();
    S = Hk*PH + R;
    KF = PH*S.inverse();

    //Innovation of the state
    Matrix<float,2,1> x;
    x = KF*(zk-hk);

    //Update of the state
    xyz(2) += x(0);
    xyz_vel(2) += x(1);

    //Update of the Covariance
    Pz -= KF*Hk*Pz; 
}

void NAV_EKF::check_xyz_covariance(){      
    //Sanity check
    if(!xyz_ini)return;

    //Check norm 
    float norm=Pxyz.norm();
    float val;
    if(norm>default_Pxyz_ini){
        val=default_Pxyz_ini/norm;
        Pxyz*=val;
    }
    
    //Make symmetric!
    Matrix<float,9,9>P2;
    P2=Pxyz+Pxyz.transpose();
    Pxyz=0.5f*P2;
}