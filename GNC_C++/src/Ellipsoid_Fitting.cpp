#include <Definitions.h>

void Correct_DataPoint(float XYZ[3], float off[3], float rotM[9]){
    Map<Matrix<float,3,1>>offset(off);
    Map<Matrix<float,3,3>>rotation(rotM);
    Map<Matrix<float,3,1>>xyz(XYZ);
    xyz-=offset;
    xyz=rotation*xyz;
}

void Recursive_Ellipsoid_Fit(float P[81], float theta[9], float XYZ[3], uint8_t ini, uint8_t rotation){
    Map<Matrix<float,9,9>>Pk(P);
    Map<Matrix<float,9,1>>Theta(theta);
    Map<Matrix<float,3,1>>xyz(XYZ);
    if(ini){
        Pk.setIdentity();
        Pk=1000*Pk;
        Theta.setZero();
        Theta.block<3,1>(0,0).setConstant(1);
    }else{
        //Fill input vector Psi
        float x,y,z;
        x=xyz(0,0);
        y=xyz(1,0);
        z=xyz(2,0);
        Matrix<float,9,1>Psi;
        Psi<<x*x,
            y*y,
            z*z,
            2*x*y,
            2*x*z,
            2*y*z,
            2*x,
            2*y,
            2*z;
        if(!rotation)Psi.block<3,1>(3,0).setZero();
        
        //Calculate error
        Matrix<float,1,1>error;
        error=-Psi.transpose()*Theta;
        error(0,0)+=1;
        
        //Calculate correction gain
        Matrix<float,1,1>S;
        Matrix<float,9,1>temp91;
        Matrix<float,9,1>Kk;
        temp91=Pk*Psi;
        S=Psi.transpose()*temp91;
        S(0,0)+=1;
        Kk=temp91*S.inverse();
        
        //Calculate correction
        Theta+=Kk*error;
        
        //Correct covariance
        Pk-=Kk*temp91.transpose();

        //Make symmetric
        Matrix<float,9,9>Pk2;
        Pk2=(Pk+Pk.transpose())/2.0f;
        Pk=Pk2;
    }
}


void Calculate_Fit(float Theta[9], float ref_radius, float off[3], float rotM[9]){
    Map<Matrix<float,9,1>>v(Theta);
    Map<Matrix<float,3,1>>offsets(off);
    Map<Matrix<float,3,3>>rotation(rotM);
    
    //Extract coefficients
    float a,b,c,d,e,f,g,h,i;
    a=v(0,0);
    b=v(1,0);
    c=v(2,0);
    d=v(3,0);
    e=v(4,0);
    f=v(5,0);
    g=v(6,0);
    h=v(7,0);
    i=v(8,0);
    
    //Calculate matrices
    Matrix<float,4,4>A4;
    Matrix<float,3,3>A3;
    Matrix<float,3,1>v_ghi;
    A4<<a,d,e,g,
        d,b,f,h,
        e,f,c,i,
        g,h,i,-1;
    A3<<a,d,e,
        d,b,f,
        e,f,c;
    v_ghi<<g,
            h,
            i;
            
    //Calculate offsets
    offsets=-A3.inverse()*v_ghi;
    float ox,oy,oz;
    ox=offsets(0,0);
    oy=offsets(1,0);
    oz=offsets(2,0);
    
    //Rotate backwards
    Matrix<float,4,4>T;
    Matrix<float,4,4>B4;
    Matrix<float,3,3>B3;
    T<<1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        ox,oy,oz,1;
    A4=A4*T.transpose();
    B4=T*A4;
    B3=-B4.block<3,3>(0,0)/B4(3,3);
    
    // Get Rotation Matrix
    SelfAdjointEigenSolver<Matrix3f> eigensolver(B3);
    if (eigensolver.info() == Success){
        rotation=eigensolver.eigenvectors();
        Matrix<float,3,1>ev;
        ev=eigensolver.eigenvalues();
        ev=ev.array().abs();
        ev=ev.array().inverse();
        ev=ev.array().sqrt();
        ev=ev.array().inverse();
        ev=ref_radius*ev;
        rotation=rotation*ev.asDiagonal();
    }else{
        rotation.setIdentity();
    }

    //Put largest in diagonal
    Matrix<float,3,3>rotation2;
    rotation2=rotation; 
    for(uint8_t i=0;i<3;i++){
        for(uint8_t j=0;j<3;j++){
            if(abs(rotation(i,j))>abs(rotation(i,i)))rotation2.block<3,1>(0,i)=rotation.block<3,1>(0,j);
        }
    }
    rotation=rotation2;
}

