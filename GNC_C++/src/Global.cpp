#include <Types.h>
#include <stdint.h>
#include <Dense>
using namespace Eigen;

uint8_t in_range(float y[], float max, float min){
    if(y[0]>max){
        y[0]=max;
        return false;
    }
    if(y[0]<min){
        y[0]=min;
        return false;
    }
    return true;
}

void quat_to_dcm(float vec4[4],float vec9[9]){
    //Map inputs
    Map<Matrix<float,4,1>>quat(vec4);
    Map<Matrix<float,3,3>>dcm(vec9);
    
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
    
    //Compute output
    dcm<<q00+q11-q22-q33,q12+q03,q13-q02,
        q12-q03,q00-q11+q22-q33,q23+q01,
        q13+q02,q23-q01,q00-q11-q22+q33;
}

void rpy_to_dcm(float vec3[3],float vec9[9]){
    //Map inputs
    Map<Matrix<float,3,1>>rpy_rad(vec3);
    Map<Matrix<float,3,3>>dcm(vec9);
    
    //Cosines and sines
    float roll,pitch,yaw,cr,cp,cy,sr,sp,sy;
    roll=rpy_rad(0,0);
    pitch=rpy_rad(1,0);
    yaw=rpy_rad(2,0);
    cr=cos(roll);
    cp=cos(pitch);
    cy=cos(yaw);
    sr=sin(roll);
    sp=sin(pitch);
    sy=sin(yaw);
    
    //Define DCM
    dcm<<cp*cy,cp*sy,-sp,
        -cr*sy+sr*sp*cy,cr*cy+sr*sp*sy,sr*cp,
        sr*sy+cr*sp*cy,-sr*cy+cr*sp*sy,cr*cp;
}

void quat_to_rpy(float vec4[4],float vec3[3]){
    //Map inputs
    Map<Matrix<float,4,1>>quat(vec4);
    Map<Matrix<float,3,1>>rpy_rad(vec3);
    
    //Define variables
    float q0,q1,q2,q3,q00,q11,q22,q33;
    q0=quat(0,0);
    q1=quat(1,0);
    q2=quat(2,0);
    q3=quat(3,0);
    q00=q0*q0;
    q11=q1*q1;
    q22=q2*q2;
    q33=q3*q3;
    
    //Calculate
    float aSinInput=2.0f * (q0 * q2 - q1 * q3);
    if(aSinInput>1.00000000f)aSinInput=1.000000000f;
    if(aSinInput<-1.00000000f)aSinInput=-1.000000000f;
    rpy_rad(0,0)=atan2(2.0f * (q0 * q1 + q2 * q3), q00 - q11 - q22 + q33);
    rpy_rad(1,0)=asin(2.0f * (q0 * q2 - q1 * q3)); 
    rpy_rad(2,0)=atan2(2.0f * (q1 * q2 + q0 * q3), q00 + q11 - q22 - q33); 
}

void quat_to_rpy(float vec4[4],float vec3_rad[3],float vec3[3]){
    //Run normal function
    quat_to_rpy(vec4,vec3_rad);
    
    //Map inputs
    Map<Matrix<float,3,1>>rpy_rad(vec3_rad);
    Map<Matrix<float,3,1>>rpy(vec3);
    
    //Scale rpy
    rpy=rpy_rad*rad_to_deg;
}

void quat_to_rates_rot(float vec4[4],float vec12[12]){
    //Map inputs
    Map<Matrix<float,4,1>>quat(vec4);
    Map<Matrix<float,4,3>>Bk(vec12);
    
    //Declar variables
    float q0,q1,q2,q3;
    q0=quat(0,0);
    q1=quat(1,0);
    q2=quat(2,0);
    q3=quat(3,0);
    
    //Get rotation matrix
    Bk<<-q1,-q2,-q3,
    q0,-q3,q2,
    q3,q0,-q1,
    -q2,q1,q0;
}