#include <Definitions.h>

void MODEL::begin(uint8_t new_limit){
    coms.send_msgs("Initialising Model...\n");
    limit=new_limit;
    load_parameters();
    override=0;
    coms.send_msgs("Model Initialised!\n");
}

void MODEL::reset(){
    //Reset states
    xyz.setZero();
    xyz(0,0)=2;
    xyz(1,0)=2;
    xyz_vel.setZero();
    wind_xyz_vel.setZero();
    xyz_acc.setZero();
    quat.setZero();
    quat(0,0)=1;
    rpy.setZero();
    rpy_rad.setZero();
    pqr.setZero();
    pqr_rad.setZero();
}

void MODEL::simulate(uint8_t flag){   
    //Simulate if overriding
    if(override){
        //Define DCM
        quat_to_dcm(&quat(0,0),&dcm(0,0));

        //Get current acc from thrust vector in NED
        float Z=-control.LMNZ[3]*p.kt;
        xyz_acc.setZero();
        xyz_acc(2,0)=Z;
        xyz_acc=dcm.transpose()*xyz_acc;

        //Add gravity (in NED)
        xyz_acc(2,0)+=g0;

        //Rotate Acc (NED to ENU/NWU)
        xyz_acc.block<2,1>(1,0)=-xyz_acc.block<2,1>(1,0);

        //Add drag
        Matrix<float,3,1>relative_xyz_vel=xyz_vel-wind_xyz_vel;
        xyz_acc-=relative_xyz_vel*relative_xyz_vel.norm()*p.Cd;

        //XYZ and XYZ vel
        xyz+=xyz_vel*dt+xyz_acc*dt*dt/2.0f;
        xyz_vel+=dt*xyz_acc;

        //Detect ground (reset velocities and z if less than 0)
        if(xyz(2,0)<0){
            xyz(2,0)=0;
            xyz_vel(2,0)=0;
            xyz_acc(2,0)=0;
        }

        //Get Quat Rotation Matrix
        quat_to_rates_rot(&quat(0,0),&quat_rot(0,0));

        //Propagate quaternion
        float dt2=dt/2.0f;
        quat+=dt2*quat_rot*pqr_rad;
        
        //Normalise
        quat=quat/quat.norm();
        
        //Convert to quat to rpy_ref and rpy
        quat_to_rpy(&quat(0,0),&rpy_rad(0,0),&rpy(0,0));

        //Simulate PQR radians
        pqr(0,0)+=dt*(p.kt*p.ly/p.Ixx*control.LMNZ[0]);
        pqr(1,0)+=dt*(p.kt*p.lx/p.Iyy*control.LMNZ[1]);
        pqr(2,0)+=dt*(p.kq/p.Izz*control.LMNZ[2]);
        
        //Convert pqr to radians
        pqr_rad=pqr*deg_to_rad;

        //Copy values to NAV EKF        
        nav_ekf.xyz=xyz;
        nav_ekf.xyz_vel=xyz_vel;
        nav_ekf.xyz_acc=xyz_acc;
        nav_ekf.rpy_rad=rpy_rad;
        nav_ekf.rpy=rpy;
        nav_ekf.pqr_rad=pqr_rad;
        nav_ekf.pqr=pqr;
        nav_ekf.xyz_lock=1;
        nav_ekf.att_lock=1;
        
        //Print
        print(flag);
    }    
}

void MODEL::set_override(uint8_t flag){
    override=flag;
    if(flag)coms.send_msgs("Model Override Enabled!\n");
    else coms.send_msgs("Model Override Disabled!\n");
}

void MODEL::update_wind(float xyz[]){
    Map<Matrix<float,3,1>>new_wind_xyz_vel(xyz);
    wind_xyz_vel=new_wind_xyz_vel;
}

void MODEL::print(uint8_t flag){
    if(flag){
        if(counter>=limit){
            counter=0;
            switch(flag){
                case 1:
                print_xyz();
                break;

                case 2:
                print_xyz_vel();
                break;
                
                case 3:
                print_xyz_acc();
                break;

                case 4:
                print_quat();
                break;

                case 5:
                print_rpy();
                break;

                case 6:
                print_pqr();
                break;

                case 7:
                print_xyz();
                print_xyz_vel();
                break;

                case 8:
                print_xyz();
                print_xyz_vel();
                print_rpy();
                break;

                case 9:
                print_xyz();
                print_xyz_vel();
                print_rpy();
                print_pqr();
                break;

                default:
                break;
            }
        }else counter++;
    }else counter=0;    
}

void MODEL::print_xyz(){
    coms.send_msgs("XYZ = \t");
    coms.print_matrix(&xyz(0,0),1,3,0);
}

void MODEL::print_xyz_vel(){
    coms.send_msgs("XYZ Vel = \t");
    coms.print_matrix(&xyz_vel(0,0),1,3,0);
}

void MODEL::print_xyz_acc(){
    coms.send_msgs("XYZ Acc = \t");
    coms.print_matrix(&xyz_acc(0,0),1,3,0);
}

void MODEL::print_quat(){
    coms.send_msgs("Quat = \t");
    coms.print_matrix(&quat(0,0),1,4,0);
}

void MODEL::print_rpy(){
    coms.send_msgs("RPY = \t");
    coms.print_matrix(&rpy(0,0),1,3,0);
}

void MODEL::print_pqr(){
    coms.send_msgs("PQR = \t");
    coms.print_matrix(&pqr(0,0),1,3,0);
}

void MODEL::load_parameters(){
    float parameters[10];
    save_read_param_float(file_name,0,parameters,10,0);
    update_parameters(parameters);
}

void MODEL::save_parameters(float parameters[]){
    if(update_parameters(parameters))save_read_param_float(file_name,0,parameters,10,1);
}

uint8_t MODEL::update_parameters(float parameters[]){
    //Check parameters not empty
    float norm=0;
    for(uint8_t i=0;i<10;i++)norm+=fabs(parameters[i]);
    if(norm<eps){
        coms.send_msgs("Model Error: Empty Parameter Set!\n");
        return false;
    }else coms.send_msgs("Model: Parameters Updated!\n");
    
    //Copy parameters
    memcpy(&p,parameters,40);    
    
    //Print
    print_parameters();

    //Exit successfully
    return true;
}

void MODEL::print_parameters(){
    sprintf(msgs,"Mass = %f\n",p.mass);
    coms.send_msgs();
    sprintf(msgs,"Ixx = %f\n",p.Ixx);
    coms.send_msgs();
    sprintf(msgs,"Iyy = %f\n",p.Iyy);
    coms.send_msgs();
    sprintf(msgs,"Izz = %f\n",p.Izz);
    coms.send_msgs();
    sprintf(msgs,"kt = %f\n",p.kt);
    coms.send_msgs();
    sprintf(msgs,"kq = %f\n",p.kq);
    coms.send_msgs();
    sprintf(msgs,"lx = %f\n",p.lx);
    coms.send_msgs();
    sprintf(msgs,"ly = %f\n",p.ly);
    coms.send_msgs();
    sprintf(msgs,"Cd = %f\n",p.Cd);
    coms.send_msgs();
    sprintf(msgs,"Radius = %f\n",p.radius);
    coms.send_msgs();
}