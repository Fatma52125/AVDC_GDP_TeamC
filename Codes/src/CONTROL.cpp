#include <Definitions.h>

char modes_msgs[15][10]={"Acro","Stabilise","Alt_Hold","Pos_Hold","Guided","Mission","Track","Land","RTL"};
char flying_msgs[4][10]={"Idle","Flying","Landing","Crashed"};

void CONTROL::begin(uint8_t new_limit){
    //Initialisation message
    coms.send_msgs("Initialising Controllers...\n");
    
    //Update printing limit
    limit=new_limit;

    //Initialise PID controllers
    roll_rate_pid.begin("R_RATE");
    pitch_rate_pid.begin("P_RATE");
    yaw_rate_pid.begin("Y_RATE");
    roll_pid.begin("ROLL");
    pitch_pid.begin("PITCH");
    yaw_pid.begin("YAW");
    x_pid.begin("X");
    y_pid.begin("Y");
    z_pid.begin("Z");
    x_vel_pid.begin("X_vel");
    y_vel_pid.begin("Y_vel");
    z_acc_pid.begin("ACC_Z");

    //Load internal parameters
    load_p1();
    load_p2();
    load_p3();
    load_p4();

    //Load waypoints
    load_x_waypoint();
    load_y_waypoint();
    load_z_waypoint();

    //Reset
    reset();
    
    //Initialised and sleep
    coms.send_msgs("Controllers Initialised!\n");
    sleep(1);
}

void CONTROL::update(uint8_t flag){
    //Run Mode
    // direct_mode();
    // if(receiver.channels[receiver.mode_ch]<200)acro_mode();
    // else if(receiver.channels[receiver.mode_ch]<800)stabilise_mode();
    // else alt_hold_mode();

    switch(mode){
        case Acro:
            acro_mode();
        break;
        case Stabilise:
            stabilise_mode();
        break;
        case Alt_Hold:
            alt_hold_mode();
        break;
        case Guided:
            guided_mode();
        break;
        case Pos_Hold:
            pos_hold_mode();
        break;
        case Mission:
            mission_mode();
        break;
        case Track:
            track_mode();
        break;
        case Land:
            land_mode();
        break;
        case RTL:
            rtl_mode();
        break;

        default: 
            reset();
        break;
    }
    check_flying_state();



    //Print if requested
    // print(flag);
}

void CONTROL::set_flying_state(flying_states new_flying_state){
    if(new_flying_state!=flying_state){
        if(flying_state==Idle)idle_timer=sys.Time;
        flying_state=new_flying_state;
        sprintf(msgs,"Flying State: %s @ %.1f\n",flying_msgs[flying_state],sys.Time);
        coms.send_msgs();
    }
}

void CONTROL::check_flying_state(){
    //Flying check
    if(sys.armed&&LMNZ[3]>p1.Zmin&&flying_state!=Flying&&flying_state!=Landing){
        coms.send_msgs();
        set_flying_state(Flying);
        idle_timer=sys.Time;
        return;
    }

    //Idle check
    float timer=sys.Time-idle_timer;
    if(timer>idle_timer_limit){
        set_flying_state(Idle);
        return;
    }else{
        if(sys.armed){
            // if(LMNZ[3]>Zmin||nav_ekf.xyz(2,0)>ground_threshold||mode==Flip||mode==Double_Flip)idle_timer=sys.Time;
            if(LMNZ[3]>p1.Zmin||nav_ekf.xyz(2,0)>ground_threshold)idle_timer=sys.Time;
        }
    }
}

void CONTROL::set_mode(modes new_mode,reasons reason){
    if(mode!=new_mode){
        if(mode_checks(new_mode,reason)){
            prev_mode=mode;
            mode=new_mode;
            mode_timer=sys.Time;
            // cout << "xyz_lock" << nav_ekf.xyz_lock << endl;
            sprintf(msgs,"Mode %s to %s @ %.1f: Reason %s\n",modes_msgs[prev_mode],modes_msgs[mode],sys.Time,reasons_msgs[reason]);
            coms.send_msgs();
        }
    }
}

uint8_t CONTROL::mode_checks(modes new_mode,reasons reason){
    //Flag for reporting
    uint8_t flag=0;
    
    //Run Checks and set variables as required
    if(flying_state==Crashed){
        coms.send_msgs("UAV Crashed!\n");
        sys.disarm(Control);
        return false;
    }

    switch(new_mode){
        //As long as we have RC
        case Stabilise: 
        // if(!receiver.active&&!coms.override_rc)flag=1;
        if(!receiver.active || receiver.channels[receiver.throttle_ch] > 100)flag=1;
        break;

        //As long as we have RC signals
        case Acro:
        if(!receiver.active)flag=1;
        break;

        //As long as we have RC signals or GCS is active or Failsafe was triggered
        case Alt_Hold:
        if(receiver.active||coms.gcs_active||reason==Failsafe)xyz_refs[2]=nav_ekf.xyz(2,0);
        else flag=1;
        break;

        //Land checks
        case Land:
        if(!receiver.active&&!coms.gcs_active&&reason!=Failsafe)flag=1;
        else memcpy(xyz_refs,&nav_ekf.xyz(0,0),12);
        break;

        //Mission
        case Mission:
        if(flying_state==Flying&&nav_ekf.xyz_lock&&(receiver.active||coms.gcs_active)){
            if(!set_xyz_refs(&nav_ekf.xyz(0,0)))flag=1;
            mission_state=0;
        }
        else flag=1;
        break;

        //Track Car
        case Track:        
        if(sys.armed&&flying_state==Flying&&nav_ekf.xyz_lock&&mode==Pos_Hold)mode_timer=sys.Time;
        else flag=1;
        break;

        //As long as we have lock and gcs is active
        // case Guided:
        // if(!nav_ekf.xyz_lock||!coms.gcs_active)flag=1;
        // else if(!set_xyz_ref(&nav_ekf.xyz(0,0)))flag=1;
        // break;

        //As long as we have xyz_lock and RC signals or GCS is active
        // xyz_lock don't have it
        case Pos_Hold:
        if(nav_ekf.xyz_lock&&(receiver.active||coms.gcs_active||reason==Failsafe)){
            xyz_refs[0]=nav_ekf.xyz(0,0);
            xyz_refs[1]=nav_ekf.xyz(1,0);
            xyz_refs[2]=nav_ekf.xyz(2,0);
            if(!set_xyz_refs(&nav_ekf.xyz(0,0)))flag=1;
        }else flag=1;
        break;

        //RTL checks
        case RTL:
        if(!nav_ekf.xyz_lock)flag=1;
        else {
            if(home_set){
                if(!check_feasible_arena(xyz_home))flag=1;
            }
            else flag=1;
        }
        break;

        default:
        break;
    }
    //Emit log msg if unable to swap
    if(flag){
        sprintf(msgs,"Unable: Mode %s from %s\n",modes_msgs[new_mode],modes_msgs[mode]);
        coms.send_msgs();
        return false;
    }else return true;
    
}

uint8_t CONTROL::check_feasible_arena(float xyz[3]){    
    //Extract
    float x,y,z;
    x=xyz[0];
    y=xyz[1];
    z=xyz[2];
    
    //Arena limits
    float x1,y1,x2,y2,z1,z2;
    x1=arena_xyz[0];
    y1=arena_xyz[1];
    x2=arena_xyz[2];
    y2=arena_xyz[3];
    z1=arena_xyz[4];
    z2=arena_xyz[5];
    
    // //Checks for outdoors
    // uint8_t accept=0;
    // if(z>z1&&z<z2&&x>x1&&x<x2&&y>y1&&y<y2)accept=1;

    //Checks for indoors
    uint8_t accept=0;
    if(z>z1-eps&&z<z2+eps){
        //Horizontal Section check
        if(x>-eps&&x<x2+eps&&y>-eps&&y<y1+eps)accept=1;
        
        //Vertical Section check
        if(x>-eps&&x<x1+eps&&y>-eps&&y<y2+eps)accept=1;
    }
    
    //Return accept
    return accept;
}

uint8_t CONTROL::check_feasible_xyz(float xyz[3]){    

    
    //Extract
    float x,y,z;
    x=xyz[0];
    y=xyz[1];
    z=xyz[2];

    //Arena limits
    float x1,y1,x2,y2,z1,z2;
    x1=arena_xyz[0];
    y1=arena_xyz[1];
    x2=arena_xyz[2];
    y2=arena_xyz[3];
    z1=arena_xyz[4];
    z2=arena_xyz[5];
    
    // //Checks for outdoors
    // uint8_t accept=0;
    // if(z>z1&&z<z2&&x>x1&&x<x2&&y>y1&&y<y2)accept=1;

    //Checks for indoors
    uint8_t accept=0;
    if(z>z1-eps&&z<z2+eps){
        //Horizontal Section check
        if(x>-eps&&x<x2+eps&&y>-eps&&y<y1+eps){
            if(nav_ekf.xyz(1,0)>-eps&&nav_ekf.xyz(1,0)<y1+eps){
                accept = 1;
            }else{
                xyz[0] = 2;
                xyz[1] = 2;
                accept = 1;
            }
        }
        //Vertical Section check
        if(x>-eps&&x<x1+eps&&y>-eps&&y<y2+eps){
            if(nav_ekf.xyz(0,0)>-eps && nav_ekf.xyz(0,0)<x1+eps){
                accept = 1;
            }else{
                xyz[0] = 2;
                xyz[1] = 2;
                accept = 1;
            }
        }
    }

    //Return accept
    return accept;
}

void CONTROL::reset(){
    //Reset main variables
    reset_LMNZ();
    reset_rpy_refs();
    reset_rate_refs();
    reset_xyz_refs();
    reset_xyz_vel_refs();
    reset_xyz_acc_refs();

    //Reset controllers
    roll_rate_pid.reset();
    pitch_rate_pid.reset();
    yaw_rate_pid.reset();
    roll_pid.reset();
    pitch_pid.reset();
    yaw_pid.reset();
    x_pid.reset();
    y_pid.reset();
    z_pid.reset();
    z_acc_pid.reset();
}

void CONTROL::set_home(float xyz[3]){
    //Check if feasible xyz_home
    if(check_feasible_xyz(xyz)){
        home_set=1;
        memcpy(xyz_home,xyz,12);
        sprintf(msgs,"Home Set @ %.1f\n",sys.Time);
        coms.send_msgs();
    }else {
        sprintf(msgs,"Refused Home @ %.1f\n",sys.Time);
        coms.send_msgs();
    }
}

void CONTROL::reset_xyz_refs(){
    for(uint8_t i=0;i<3;i++)xyz_refs[i]=0;
}

void CONTROL::reset_xyz_vel_refs(){
    for(uint8_t i=0;i<3;i++)xyz_vel_refs[i]=0;
}

void CONTROL::reset_xyz_acc_refs(){
    for(uint8_t i=0;i<3;i++)xyz_acc_refs[i]=0;
}

void CONTROL::reset_rpy_refs(){
    for(uint8_t i=0;i<3;i++)rpy_refs[i]=0;
}

void CONTROL::reset_rate_refs(){
    for(uint8_t i=0;i<3;i++)rate_refs[i]=0;
}

void CONTROL::reset_LMNZ(){
    for(uint8_t i=0;i<4;i++)LMNZ[i]=0;
}

void CONTROL::print(uint8_t flag){
    //Print if requested
    if(flag){
        if(counter>=limit){
            counter=0;
            switch(flag){
                case 1:
                print_xyz_refs();
                break;

                case 2:
                print_xyz_vel_refs();
                break;

                case 3: 
                print_xyz_acc_refs();
                break;

                case 4:
                print_rpy_refs();
                break;

                case 5:
                print_rate_refs();
                break;

                case 6:
                print_LMNZ();
                break;

                default:
                break;
            }
        }else counter++;
    }else counter=0;
}

void CONTROL::print_xyz_refs(){
    coms.send_msgs("XYZ Refs = \t");
    coms.print_matrix(xyz_refs,1,3,0);
}

void CONTROL::print_xyz_vel_refs(){
    coms.send_msgs("XYZ Vel Refs = \t");
    coms.print_matrix(xyz_vel_refs,1,3,0);
}

void CONTROL::print_xyz_acc_refs(){
    coms.send_msgs("XYZ Acc Refs = \t");
    coms.print_matrix(xyz_acc_refs,1,3,0);
}

void CONTROL::print_rpy_refs(){
    coms.send_msgs("RPY Refs = \t");
    coms.print_matrix(rpy_refs,1,3,0);
}

void CONTROL::print_rate_refs(){
    coms.send_msgs("Rate Refs = \t");
    coms.print_matrix(rate_refs,1,3,0);
}

void CONTROL::print_LMNZ(){
    coms.send_msgs("LMNZ = \t");
    coms.print_matrix(LMNZ,1,4,0);
}

uint8_t CONTROL::set_xyz_refs(float xyz[]){
    return set_xyz_refs_wmaxvel(xyz,50);
    
}

uint8_t CONTROL:: set_yaw_ref(float xyz[]){
    
    Map<Matrix<float,3,1>>xyz_refs_k(xyz);

    float yk[3],rk[3],d[3],delta_yaw, max_vel;

    yk[0]= xyz_refs[0];
    yk[1]= xyz_refs[1];

    rk[0] = xyz_refs_k(0,0);
    rk[1] = xyz_refs_k(1,0);

    // cout << "XYZ_refs_k" << xyz_refs_k << endl;

    d[0] = rk[0] - yk[0];       
    d[1] = rk[1] - yk[1];

    // rpy_refs[2] =  - rad_to_deg*atan2(d[1],d[0]);
    // delta_yaw=(nav_ekf.rpy(2)-rpy_refs[2]);
    // sprintf(msgs,"Yaw ref 1%f\n",rpy_refs[2]);
    // coms.send_msgs();
    // sprintf(msgs,"YAW_ CALCULUS %f\n",- rad_to_deg*atan2(d[1],d[0]));
    // coms.send_msgs();

    delta_yaw =  - rad_to_deg*atan2(d[1],d[0]) - rpy_refs[2];

    max_vel = 15;

    float max_delta=max_vel*dt;

    if(delta_yaw>180)delta_yaw-=360;                         //Limit error to max
    if(delta_yaw<-180)delta_yaw+=360;

    if(delta_yaw > 10)defer_xyz_for_yaw = 1;
    if(delta_yaw < -10)defer_xyz_for_yaw = 1;


    if(delta_yaw>max_delta)delta_yaw=max_delta;
    if(delta_yaw<-max_delta)delta_yaw=-max_delta;


    
    rpy_refs[2] = delta_yaw + rpy_refs[2];
    return true;
    // if(check_feasible_xyz(xyz)){
    //     // Matrix<float,3,1>delta_yaw;
    //     Map<Matrix<float,3,1>>xyz_refs_k(xyz);

    //     float yk[3],rk[3],d[3],delta_yaw, max_vel;

    //     yk[0]= xyz_refs[0];
    //     yk[1]= xyz_refs[1];

    //     rk[0] = xyz_refs_k(0,0);
    //     rk[1] = xyz_refs_k(1,0);

    //     // cout << "XYZ_refs_k" << xyz_refs_k << endl;

    //     d[0] = rk[0] - yk[0];       
    //     d[1] = rk[1] - yk[1];

    //     // rpy_refs[2] =  - rad_to_deg*atan2(d[1],d[0]);
    //     // delta_yaw=(nav_ekf.rpy(2)-rpy_refs[2]);
    //     // sprintf(msgs,"Yaw ref 1%f\n",rpy_refs[2]);
    //     // coms.send_msgs();
    //     // sprintf(msgs,"YAW_ CALCULUS %f\n",- rad_to_deg*atan2(d[1],d[0]));
    //     // coms.send_msgs();

    //     delta_yaw =  - rad_to_deg*atan2(d[1],d[0]) - rpy_refs[2];

    //     max_vel = 15;

    //     float max_delta=max_vel*dt;

    //     if(delta_yaw>180)delta_yaw-=360;                         //Limit error to max
    //     if(delta_yaw<-180)delta_yaw+=360;

    //     if(delta_yaw > 10)defer_xyz_for_yaw = 1;
    //     if(delta_yaw < -10)defer_xyz_for_yaw = 1;


    //     if(delta_yaw>max_delta)delta_yaw=max_delta;
    //     if(delta_yaw<-max_delta)delta_yaw=-max_delta;


        
    //     rpy_refs[2] = delta_yaw + rpy_refs[2];

    //     // sprintf(msgs,"Yaw ref 2 %f  & Delta_yaw %f\n",rpy_refs[2],delta_yaw);
    //     // coms.send_msgs();

    //     return true;
    // }else {
    //     sprintf(msgs,"Refused xyz_ref @ %.1f\n",sys.Time);
    //     coms.send_msgs();
    //     return false;
    // }
    
}

uint8_t CONTROL::set_xyz_refs_wmaxvel(float xyz[],float max_vel){
    // Matrix<float,3,1>delta_xyz;
    // Map<Matrix<float,3,1>>xyz_ref_k(xyz_refs);
    // Map<Matrix<float,3,1>>new_xyz_ref_k(xyz);
    // float max_delta_xyz=max_vel*dt;
    // delta_xyz=new_xyz_ref_k-xyz_ref_k;
    // float delta_norm=delta_xyz.norm();
    // if(delta_norm>max_delta_xyz&&delta_norm>eps){
    //     delta_xyz=delta_xyz/delta_norm*max_delta_xyz;
    // }
    // xyz_ref_k+=delta_xyz;
        //Defer checks
    if(defer_xyz_ref_update){
        defer_xyz_ref_update=0;
        return true;
    }

    // // //Copy to target xyz ref    
    // // memmove(target_xyz_refs,xyz,12);
    // // target_max_vel=max_vel;

    //Check if feasible xyz ref
    if(check_feasible_xyz(xyz)){
        Matrix<float,3,1>delta_xyz;
        Map<Matrix<float,3,1>>xyz_k(xyz);
        Map<Matrix<float,3,1>>xyz_refs_k(xyz_refs);
        delta_xyz=(xyz_k-xyz_refs_k);
        // max_vel = 0.1;
        float max_delta=max_vel*dt;
        for(uint8_t i=0;i<3;i++){
            if(delta_xyz(i,0)>max_delta)delta_xyz(i,0)=max_delta;
            if(delta_xyz(i,0)<-max_delta)delta_xyz(i,0)=-max_delta;
        }
        // sprintf(msgs,"Delta_xyz @ %f\n",delta_xyz);
        // coms.send_msgs();
        xyz_refs_k+=delta_xyz;
        return true;
    }else {
        sprintf(msgs,"Refused xyz_ref @ %.1f\n",sys.Time);
        coms.send_msgs();
        return false;
    }

    

    // check_feasible_xyz
}

void CONTROL::get_xyz_refs(){
    //Check if defer requested
    if(defer_xyz_ref_update){
        defer_xyz_ref_update=0;
        return;
    }
    
    //Update xyz refs
    float new_xyz_refs[3];
    new_xyz_refs[0]=xyz_refs[0];
    new_xyz_refs[1]=xyz_refs[1];
    new_xyz_refs[2]=xyz_refs[2];
    
    //Z axis
    float max=500+p2.neutral;
    float min=500-p2.neutral;
    if(!defer_alt_ref_update){
        if(receiver.channels[receiver.throttle_ch]>max){
            xyz_vel_refs[2]=(receiver.channels[receiver.throttle_ch]-max)*p1.RC_to_alt;
            new_xyz_refs[2]+=dt*xyz_vel_refs[2];
        }
        if(receiver.channels[receiver.throttle_ch]<min){
            xyz_vel_refs[2]=(receiver.channels[receiver.throttle_ch]-min)*p1.RC_to_alt;
            new_xyz_refs[2]+=dt*xyz_vel_refs[2];
        }
    }else defer_alt_ref_update=0;
    
    //XY Axis
    // float vel_ref_xy[2]={0,0};
    // max=p2.neutral;
    // min=-p2.neutral;
    // if(receiver.channels[receiver.pitch_ch]>max)vel_ref_xy[0]=-(receiver.channels[receiver.pitch_ch]-max)*p1.RC_to_xy;
    // if(receiver.channels[receiver.pitch_ch]<min)vel_ref_xy[0]=-(receiver.channels[receiver.pitch_ch]-min)*p1.RC_to_xy;
    // if(receiver.channels[receiver.roll_ch]>max)vel_ref_xy[1]=-(receiver.channels[receiver.roll_ch]-max)*p1.RC_to_xy;
    // if(receiver.channels[receiver.roll_ch]<min)vel_ref_xy[1]=-(receiver.channels[receiver.roll_ch]-min)*p1.RC_to_xy;
    // float yaw=nav_ekf.rpy_rad(2,0);
    // xyz_vel_refs[0]=(cos(yaw)*vel_ref_xy[0]+sin(yaw)*vel_ref_xy[1]);
    // new_xyz_refs[0]+=dt*xyz_vel_refs[0];
    // xyz_vel_refs[1]=(-sin(yaw)*vel_ref_xy[0]+cos(yaw)*vel_ref_xy[1]);
    // new_xyz_refs[1]+=dt*xyz_vel_refs[1];
        
    set_xyz_refs(new_xyz_refs);
}

void CONTROL::get_alt_ref(){
    //Check if defer requested
    if(defer_alt_ref_update){
        defer_alt_ref_update=0;
        return;
    }
    
    // //Update alt ref (Version 1)
    // xyz_refs[2]=receiver.channels[receiver.throttle_ch]*p1.RC_to_alt;

    //Update alt ref (Version 2)
    float value=receiver.channels[receiver.throttle_ch]-p2.neutral_throttle;
    // cout << "Receiver throttle" << receiver.throttle_ch << endl;
    if(value>p2.neutral_deadzone)value=value-p2.neutral_deadzone;
    else if(value<-p2.neutral_deadzone)value=value+p2.neutral_deadzone;
    else value=0;
    xyz_vel_refs[2]=value*p1.RC_to_alt;
    xyz_refs[2]+=dt*xyz_vel_refs[2];
    if(xyz_refs[2]<0)xyz_refs[2]=0;

    if(xyz_refs[2]>3.5)xyz_refs[2]=3.5;
}

void CONTROL::get_xyz_vel_refs(){    
    //Check if defer requested
    if(defer_xyz_vel_ref_update){
        defer_xyz_vel_ref_update=0;
        return;
    }
    
    //Update xyz vel refs
}

void CONTROL::get_xyz_acc_refs(){
    //Check if defer requested
    if(defer_xyz_acc_ref_update){
        defer_xyz_acc_ref_update=0;
        return;
    }
    
    //Update xyz acc refs
}

void CONTROL::get_rpy_refs(){
    //Check if defer requested
    if(defer_rpy_ref_update){
        defer_rpy_ref_update=0;
        return;
    }
    
    //Update rpy refs
    rpy_refs[0]=receiver.channels[receiver.roll_ch]*p1.RC_to_rp;
    rpy_refs[1]=receiver.channels[receiver.pitch_ch]*p1.RC_to_rp;
    rpy_refs[2]=receiver.channels[receiver.yaw_ch]*p1.RC_to_yaw;
    // cout <<"Receiver refs= " << rpy_refs << endl;
    // cout <<"Receiver p1 = " << p1.RC_to_rp << endl;
}

void CONTROL::get_rate_refs(){
    //Check if defer requested
    if(defer_rate_ref_update){
        defer_rate_ref_update=0;
        return;
    }
    
    //Update rate refs
    rate_refs[0]=receiver.channels[receiver.roll_ch]*p1.RC_to_rp_rates;
    rate_refs[1]=receiver.channels[receiver.pitch_ch]*p1.RC_to_rp_rates;
    rate_refs[2]=receiver.channels[receiver.yaw_ch]*p1.RC_to_yaw_rates;
    // cout <<"rate_refs_yaw = " <<rate_refs[2] << endl;
    // cout <<"rate_refs_roll = " <<rate_refs[0] << endl;
}

void CONTROL::get_throttle_ref(){
    //Check if defer requested
    if(defer_throttle_update){
        defer_throttle_update=0;
        return;
    }
    
    //Update throttle
    LMNZ[3]=receiver.channels[receiver.throttle_ch];
}

void CONTROL::get_direct_refs(){
    LMNZ[0]=receiver.channels[receiver.roll_ch];
    LMNZ[1]=receiver.channels[receiver.pitch_ch];
    LMNZ[2]=receiver.channels[receiver.yaw_ch];
    LMNZ[3]=receiver.channels[receiver.throttle_ch];
}

//Main functions
void CONTROL::run_xyz_controllers(){
    //Declare variables.
    float yk,rk, y_dot;

    //Run x controller
    yk=nav_ekf.xyz(0,0);
    rk=xyz_refs[0];
    y_dot = nav_ekf.xyz_vel(0,0);
    xyz_acc_refs[0]=x_pid.update_xyz(yk,rk,dt,1,0,y_dot);
    // cout << "Accel x=" << xyz_acc_refs[0] << endl;

    //Run y controller
    yk=nav_ekf.xyz(1,0);
    rk=xyz_refs[1];
    y_dot = nav_ekf.xyz_vel(1,0);
    xyz_acc_refs[1]=y_pid.update_xyz(yk,rk,dt,1,0,y_dot);
    
}

void CONTROL::run_alt_controller(){
    //Run PID controller
    float yk=nav_ekf.xyz(2,0);
    float rk=xyz_refs[2];
    // LMNZ[3]=g0/model.p.kt+z_pid.update(yk,rk,dt,0,0)/model.p.kt; // + model.p.Cd*nav_ekf.xyz_vel(2,0);cd De
    xyz_acc_refs[2] = z_pid.update(yk,rk,dt,1,0);
    // //Tilt compensation
    // LMNZ[3]/=cos(nav_ekf.rpy_rad(0,0))*cos(nav_ekf.rpy_rad(1,0));    
}

void CONTROL::run_xyz_vel_controllers(){
    //Declare variables.
    float yk,rk;

    //Run roll controller
    yk=nav_ekf.xyz_vel(0,0);
    rk=xyz_vel_refs[0];
    rpy_refs[0]=x_vel_pid.update(yk,rk,dt,0,0);

    //Run pitch controller
    yk=nav_ekf.rpy(1,0);
    rk=xyz_vel_refs[1];
    rpy_refs[1]=y_vel_pid.update(yk,rk,dt,0,0);
}

void CONTROL::run_xy_acc_controllers(){

}

void CONTROL::run_z_acc_controllers(){
    //Declare variables.
    float yk,rk;

    //Run z acceleration controller
    yk=nav_ekf.xyz_acc(2,0);
    rk=xyz_acc_refs[2];
    LMNZ[3] = (g0 + z_acc_pid.update(yk,rk,dt,1,0) + model.p.Cd*nav_ekf.xyz_vel(2,0))/model.p.kt; 
    
    //Tilt compensation
    LMNZ[3]/=cos(nav_ekf.rpy_rad(0,0))*cos(nav_ekf.rpy_rad(1,0));  
}


void CONTROL::convert_acc_to_eulerangles(){
    //Define limits
    float max_g=p2.thrust_to_weight*g0;
    float min_gz=-p2.alpha_min_gz*g0;

    //Get requested acceleration
    Matrix<float,3,1>acc;
    acc(0,0)=xyz_acc_refs[0];
    acc(1,0)=xyz_acc_refs[1];
    acc(2,0)=xyz_acc_refs[2];  

    //Limit accelerations
    if(acc(2,0)<min_gz)acc(2,0)=min_gz;

    //Add gravity
    acc(2,0)+=g0;
    
    //Add drag
    float vel_norm=nav_ekf.xyz_vel.norm();
    acc+=nav_ekf.xyz_vel*vel_norm*model.p.Cd*0.1; 

    //Limitation with the maximum g's considered
    float acc_norm=acc.norm();
    if(acc_norm>max_g){
        acc*=max_g/acc_norm;
        acc_norm=max_g;
    }

    //Convert to roll and pitch 
    float yaw=-nav_ekf.rpy_rad(2,0); // ENU - NED conversions
    float cy,sy;
    Matrix<float,2,2>temp22;
    Matrix<float,2,1>temp21;
    cy=cos(yaw);
    sy=sin(yaw);
    temp22<<sy,cy,
            -cy,sy;
    temp21=acc.block<2,1>(0,0);
    temp21/=acc_norm;
    temp21=temp22.inverse()*temp21;
    
    //Check to apply ENU to NED compensation
    rpy_refs[0]=asin(temp21(0,0));
    rpy_refs[1]=-asin(temp21(1,0)/cos(rpy_refs[0]));
    // cout << "ROLL REFS=" << rpy_refs[0] << endl;

    //Convert to degrees
    rpy_refs[0]*=rad_to_deg;
    rpy_refs[1]*=rad_to_deg;

    //Limit angles to 20 degrees.
    if(rpy_refs[0] > p2.rp_lim) rpy_refs[0] = p2.rp_lim;
    if(rpy_refs[0] < -p2.rp_lim) rpy_refs[0] = -p2.rp_lim;

    if(rpy_refs[1] > p2.rp_lim) rpy_refs[1] = p2.rp_lim;
    if(rpy_refs[1] < -p2.rp_lim) rpy_refs[1] = -p2.rp_lim;
}

void CONTROL:: set_yaw(){
    float yk[3],rk[3],d[3];

    yk[0]= nav_ekf.xyz(0);
    yk[1]= nav_ekf.xyz(1);

    rk[0] = xyz_refs[0];
    rk[1] = xyz_refs[1];

    d[0] = rk[0] - yk[0];
    d[1] = rk[1] - yk[1];

    if(d[0] != 0){
    rpy_refs[2] = 180 - atan2(d[1],d[0])*rad_to_deg;
    }
}

void CONTROL::run_rpy_controllers(){
    //Declare variables.
    float yk,rk;

    //Run roll controller
    yk=nav_ekf.rpy(0,0);
    rk=rpy_refs[0];
    rate_refs[0]=roll_pid.update(yk,rk,dt,0,0);

    //Run pitch controller
    yk=nav_ekf.rpy(1,0);
    rk=rpy_refs[1];
    rate_refs[1]=pitch_pid.update(yk,rk,dt,0,0);

    //Run yaw controller
    if(p3.control_yaw){
        yk=nav_ekf.rpy(2,0);
        rk=rpy_refs[2];
        rate_refs[2]=yaw_pid.update_yaw(yk,rk,dt,0,1);
    }    
}

void CONTROL::convert_euler_rates_to_body_rates(){
    //Store temporal variables
    for(uint8_t i=0;i<3;i++)temp[i]=rate_refs[i];
    
    //Calculate roll/pitch
    float roll,pitch;
    roll=nav_ekf.rpy_rad(0,0);
    pitch=nav_ekf.rpy_rad(1,0);
    float cr,cp,sr,sp;
    cr=cos(roll);
    cp=cos(pitch);
    sr=sin(roll);
    sp=sin(pitch);

    //Convert references
    rate_refs[0]=1*temp[0]-sp*temp[2];
    rate_refs[1]=cr*temp[1]+sr*cp*temp[2];
    rate_refs[2]=-sr*temp[1]+cr*cp*temp[2];
}

void CONTROL::run_rate_controllers(){  
    //Declare variables.
    float yk,rk;

    //Run roll rate controller
    yk=nav_ekf.pqr(0,0);
    rk=rate_refs[0];
    LMNZ[0]=roll_rate_pid.update(yk,rk,dt,1,0);

    //Run pitch rate controller    
    yk=nav_ekf.pqr(1,0);
    rk=rate_refs[1];
    LMNZ[1]=pitch_rate_pid.update(yk,rk,dt,1,0);

    //Run yaw rate controller
    yk=nav_ekf.pqr(2,0);
    rk=rate_refs[2];
    LMNZ[2]=yaw_rate_pid.update(yk,rk,dt,1,0);
}

void CONTROL::output(){
    limit_LMNZ();
    allocate();
    limit_motors();
    // print_LMNZ();

    if(!model.override) inputs.write_motors();
}

uint8_t CONTROL::check_idle(){
    if(flying_state==Idle){
        set_xyz_refs(&nav_ekf.xyz(0,0));
        reset_rpy_refs();
        reset_rate_refs();
        if(!check_neutral_throttle()){
            set_flying_state(Flying);
            idle_timer=sys.Time;
        }else{
            reset_LMNZ();
            output();
            return true;
        }
    }
    return false;
}

uint8_t CONTROL::check_neutral_throttle(){
        get_throttle_ref();
        uint8_t flag=1;
        if((mode==Alt_Hold||mode==Pos_Hold)&&LMNZ[3]>500+p2.neutral)flag=0;
        // if((mode==Acro||mode==Stabilise||mode==Direct)&&LMNZ[3]>neutral)flag=0;
        if((mode==Acro||mode==Stabilise)&&LMNZ[3]>p2.neutral)flag=0;
        reset_LMNZ();
        return flag;
}

void CONTROL::limit_LMNZ(){
    // limit Z
    if(LMNZ[3]>p1.Zmax) LMNZ[3] = p1.Zmax;
    if(LMNZ[3]<p1.Zmin) LMNZ[3] = p1.Zmin;

    //limit Lmax
    float LMmax;
    if(LMNZ[3] - p1.Zmin > p1.LMmax) LMmax = p1.LMmax;
    else LMmax = LMNZ[3] - p1.Zmin;

    //limit Nmax
    float Nmax;
    if(LMNZ[3] - p1.Zmin > p1.Nmax) Nmax = p1.Nmax;
    else Nmax = LMNZ[3] - p1.Zmin;

    //limit L [-LMmax +LMmax]
    if(LMNZ[0]>LMmax) LMNZ[0] = LMmax;
    if(LMNZ[0]<-LMmax) LMNZ[0]=-LMmax;

    //limit M [-LMmax +LMmax]
    if(LMNZ[1]>LMmax) LMNZ[1] = LMmax;
    if(LMNZ[1]<-LMmax) LMNZ[1]=-LMmax;

    //limit N [-Nmax + Nmax]
    if(LMNZ[2]>Nmax) LMNZ[2] = Nmax;
    if(LMNZ[2]<-Nmax) LMNZ[2]=-Nmax;
}

void CONTROL::allocate(){
    float L,M,N,Z;
    L = LMNZ[0];
    M = LMNZ[1];
    N = LMNZ[2];
    Z = LMNZ[3];

    inputs.motors[0] = -L+M+N+Z;
    inputs.motors[1] = +L+M-N+Z;
    inputs.motors[2] = +L-M+N+Z;
    inputs.motors[3] = -L-M-N+Z;
    // inputs.motors[0]=500;
}

void CONTROL::limit_motors(){
    for( int i = 0; i < 4; i++){
        if(inputs.motors[i] > p2.motors_max) inputs.motors[i] = p2.motors_max;
        if(inputs.motors[i] < p2.motors_min) inputs.motors[i] = p2.motors_min;
    }

   

}

//Mode functions
void CONTROL::direct_mode(){
    get_direct_refs();
    output();
}

void CONTROL::acro_mode(){
    if(check_idle())return;
    get_rate_refs();
    get_throttle_ref();
    run_rate_controllers();
    output();
}

void CONTROL::stabilise_mode(){
    if(check_idle())return;
    get_rpy_refs();
    get_rate_refs();
    get_throttle_ref();
    run_rpy_controllers();
    convert_euler_rates_to_body_rates();
    defer_rate_ref_update=1;
    defer_throttle_update=1;
    acro_mode();
}

void CONTROL::alt_hold_mode(){
    if(check_idle())return;
    // Get references
    get_alt_ref();
 
    // Controller Matlab PID for z + P for accelerations
    run_alt_controller();
    run_z_acc_controllers();

    // As we defer throttle update it means is not updated
    defer_throttle_update=1;
    stabilise_mode();
}

void CONTROL::pos_hold_mode(){
    if(check_idle())return;

    // Get references
    get_xyz_refs();
    // print_xyz_refs();

    // Controller Matlab PID for xyz
    run_xyz_controllers();

    run_alt_controller();
    run_z_acc_controllers();

    convert_acc_to_eulerangles();

    //Run roll and pitch controllers
    run_rpy_controllers();
    convert_euler_rates_to_body_rates();
    run_rate_controllers();

    //Issue output
    output();  

}

void CONTROL::mission_mode(){
    if(check_idle())return;
    if(!car_xyz_available){
        float mission_xyz_refs[3];
        mission_xyz_refs[0] = mission_xyz[mission_state][0];
        mission_xyz_refs[1] = mission_xyz[mission_state][1];
        mission_xyz_refs[2] = mission_xyz[mission_state][2];

        set_yaw_ref(mission_xyz[mission_state]);
        // sprintf(msgs,"Defer xyz %d\n",defer_xyz_for_yaw);
        // coms.send_msgs();
        if(defer_xyz_for_yaw){
            defer_xyz_for_yaw=0;
        }else{
            set_xyz_refs_wmaxvel(mission_xyz_refs,0.8);
        }
        // sprintf(msgs,"Refs XYZ %f\n",mission_xyz[mission_state][0]);
        // coms.send_msgs();
        defer_xyz_ref_update=1;
        pos_hold_mode();

        //Check completion of waypoint
        Map<Matrix<float,3,1>>xyz_ref_k(mission_xyz[mission_state]);
        // sprintf(msgs,"Mission State(%d): %f,%f,%f\n",mission_state,mission_xyz[mission_state][0],mission_xyz[mission_state][1],mission_xyz[mission_state][2]);
        // coms.send_msgs();
        if((xyz_ref_k-nav_ekf.xyz).norm()<0.8){
            // if((xyz_ref_k[2]-nav_ekf.xyz(2,0))<0.3&&(xyz_ref_k[2]-nav_ekf.xyz(2,0)>-0.3)){
            if(mission_state < (max_mission_waypoints-1))mission_state++;
            else {
                mission_state=0;
                // reset_mission_timers();
                }
            // }           
        }    
        // sprintf(msgs,"car should be zero %d\n",car_xyz_available);
        // coms.send_msgs();
    }else{
        set_yaw_ref(car_xyz);
        // sprintf(msgs,"car available %d\n",control.car_xyz_available);
        // coms.send_msgs("Car tracking");
        if(defer_xyz_for_yaw){
            defer_xyz_for_yaw=0;
        }else{
    //         Map<Matrix<float,3,1>>xyz_refs_k(tracking_xyz);
    //         Matrix<float,3,1>xyz_track;

            float yk[3],rk[3];

            yk[0]= xyz_refs[0];
            yk[1]= xyz_refs[1];

            rk[0] = car_xyz[0];
            rk[1] = car_xyz[1];

    //         // cout << "XYZ_refs_k" << xyz_refs_k << endl;

            Matrix<float,2,1> d;
            d(0,0) = rk[0] - yk[0];
            d(1,0) = rk[1] - yk[1];

            // Maintain a distance of 2 meters from the target 
            float d_proj_normalized[2], drone_projected[2], xyz_track[3];
            float desired_distance = 2;
            float d_proj_norm=d.norm();
            if(d_proj_norm > 0.01){
                d_proj_normalized[0] = d(0,0)/d_proj_norm;
                d_proj_normalized[1] = d(1,0)/d_proj_norm;
                
                drone_projected[0] = car_xyz[0] - d_proj_normalized[0] * desired_distance; 
                drone_projected[1] = car_xyz[1] - d_proj_normalized[1] * desired_distance;

            }else{
                drone_projected[0] = car_xyz[0]; 
                drone_projected[1] = car_xyz[1];
            }
            // Update the drone position            
            xyz_track[0] = drone_projected[0];
            xyz_track[1] = drone_projected[1];
            xyz_track[2] = 2; //Altitude for tracking

            // sprintf(msgs,"car= %f \n %f \n %f \n",xyz_track[0],xyz_track[1],xyz_track[2]);
            // coms.send_msgs();

            set_xyz_refs_wmaxvel(xyz_track,0.8);
        }
    
        defer_xyz_ref_update=1;
        pos_hold_mode(); 
        if(sys.Time-coms.car_stamp > 1)control.car_xyz_available=0;
    }
}


void CONTROL::guided_mode(){
    if(check_idle())return;

    float mission_xyz_refs[3];
    mission_xyz_refs[0] = mission_xyz[mission_state][0];
    mission_xyz_refs[1] = mission_xyz[mission_state][1];
    mission_xyz_refs[2] = mission_xyz[mission_state][2];

    set_yaw_ref(mission_xyz[mission_state]);
    // sprintf(msgs,"Defer xyz %d\n",defer_xyz_for_yaw);
    // coms.send_msgs();
    if(defer_xyz_for_yaw){
        defer_xyz_for_yaw=0;
    }else{
        set_xyz_refs_wmaxvel(mission_xyz_refs,0.8);
    }
    // sprintf(msgs,"Refs XYZ %f\n",mission_xyz[mission_state][0]);
    // coms.send_msgs();
    defer_xyz_ref_update=1;
    pos_hold_mode();

    //Check completion of waypoint
    Map<Matrix<float,3,1>>xyz_ref_k(mission_xyz[mission_state]);
    // sprintf(msgs,"Mission State(%d): %f,%f,%f\n",mission_state,mission_xyz[mission_state][0],mission_xyz[mission_state][1],mission_xyz[mission_state][2]);
    // coms.send_msgs();
    if((xyz_ref_k-nav_ekf.xyz).norm()<0.8){
        // if((xyz_ref_k[2]-nav_ekf.xyz(2,0))<0.3&&(xyz_ref_k[2]-nav_ekf.xyz(2,0)>-0.3)){
        if(mission_state < (max_mission_waypoints-1))mission_state++;
        else {
            mission_state=0;
            // reset_mission_timers();
            }
        // }           
    }
}

void CONTROL::reset_mission(){
    mission_state = 0;
}

void CONTROL::track_mode(){
    // Track mode for testing
    if(check_idle())return;
    // // If tracking -> tracking_mode=1
    if(car_xyz_available){
    //     // Map<Matrix<float,3,1>>car_xyz(tracking_xyz);
        // sprintf(msgs,"Car_x = %f, Car_y = %f,Car_z = %f \n Drone_x = %f, Drone_y = %f",car_xyz[0],car_xyz[1],car_xyz[2],nav_ekf.xyz(0,0),nav_ekf.xyz(1,0));
        // coms.send_msgs();
        set_yaw_ref(car_xyz);
        if(defer_xyz_for_yaw){
            defer_xyz_for_yaw=0;
        }else{
    //         Map<Matrix<float,3,1>>xyz_refs_k(tracking_xyz);
    //         Matrix<float,3,1>xyz_track;

            float yk[3],rk[3];

            yk[0]= xyz_refs[0];
            yk[1]= xyz_refs[1];

            rk[0] = car_xyz[0];
            rk[1] = car_xyz[1];

    //         // cout << "XYZ_refs_k" << xyz_refs_k << endl;

            Matrix<float,2,1> d;
            d(0,0) = rk[0] - yk[0];
            d(1,0) = rk[1] - yk[1];

            // Maintain a distance of 2 meters from the target 
            float d_proj_normalized[2], drone_projected[2], xyz_track[3];
            float desired_distance = 2;
            float d_proj_norm=d.norm();
            if(d_proj_norm > 0.01){
                d_proj_normalized[0] = d(0,0)/d_proj_norm;
                d_proj_normalized[1] = d(1,0)/d_proj_norm;
                
                drone_projected[0] = car_xyz[0] - d_proj_normalized[0] * desired_distance; 
                drone_projected[1] = car_xyz[1] - d_proj_normalized[1] * desired_distance;

            }else{
                drone_projected[0] = car_xyz[0]; 
                drone_projected[1] = car_xyz[1];
            }
            // Update the drone position            
            xyz_track[0] = drone_projected[0];
            xyz_track[1] = drone_projected[1];
            xyz_track[2] = 1; //Altitude for tracking

            set_xyz_refs_wmaxvel(xyz_track,0.8);
        }
    }else{
   
    }
  
    defer_xyz_ref_update=1;
    pos_hold_mode();
} 
void CONTROL::loiter_mode(){

}

void CONTROL::land_mode(){
    if(check_idle())return;

    if(xyz_refs[2]>2.8)xyz_refs[2]=2.8;

    xyz_refs[0] = nav_ekf.xyz(0,0);
    xyz_refs[1] = nav_ekf.xyz(1,0);
    xyz_refs[2] -= dt*p2.land_velocity; //deberíamos cambiarlo

    if(nav_ekf.xyz_lock){
        defer_xyz_ref_update=1;
        pos_hold_mode();
    }else{
        defer_alt_ref_update=1;
        alt_hold_mode();
    }
    // if(nav_ekf.xyz(2,0)<p2.ground_threshold)sys.disarm(Control);
}

void CONTROL::break_mode(){

}

void CONTROL::rtl_mode(){
    if(check_idle())return;
    xyz_home[2] = 1;
    float xyz_home_refs[3];
    xyz_home_refs[0] = xyz_home[0];
    xyz_home_refs[1] = xyz_home[1];
    xyz_home_refs[2] = xyz_home[2];
    set_xyz_refs_wmaxvel(xyz_home_refs,0.8);
    defer_xyz_ref_update=1;
    pos_hold_mode();
    Map<Matrix<float,3,1>>xyz_ref_k(xyz_home);
    if((nav_ekf.xyz-xyz_ref_k).norm()<0.5){        
        defer_xyz_ref_update=1;
        set_mode(Land,Control);
    }
}

//Parameter functions
void CONTROL::load_p1(){
    float buffer[10];
    save_read_param_float(file_name,index_p1,buffer,10,0);
    update_p1(buffer);
}

void CONTROL::load_p2(){
    float buffer[10];
    save_read_param_float(file_name,index_p2,buffer,10,0);
    update_p2(buffer);
}

void CONTROL::load_p3(){
    float buffer[10];
    save_read_param_float(file_name,index_p3,buffer,10,0);
    update_p3(buffer);
}

void CONTROL::load_p4(){
    float buffer[10];
    save_read_param_float(file_name,index_p4,buffer,10,0);
    update_p4(buffer);
}

void CONTROL::save_p1(float buffer[]){
    if(update_p1(buffer))save_read_param_float(file_name,index_p1,buffer,10,1);
}

void CONTROL::save_p2(float buffer[]){
    if(update_p2(buffer))save_read_param_float(file_name,index_p2,buffer,10,1);
}

void CONTROL::save_p3(float buffer[]){
    if(update_p3(buffer))save_read_param_float(file_name,index_p3,buffer,10,1);
}

void CONTROL::save_p4(float buffer[]){
    if(update_p4(buffer))save_read_param_float(file_name,index_p4,buffer,10,1);
}

uint8_t CONTROL::update_p1(float buffer[]){
    if(check_array(buffer,10))return false;
    memcpy(&p1,buffer,40);
    print_p1();
    return true;
}

uint8_t CONTROL::update_p2(float buffer[]){
    if(check_array(buffer,10))return false;
    memcpy(&p2,buffer,40);
    print_p2();
    return true;
}

uint8_t CONTROL::update_p3(float buffer[]){
    // if(check_array(buffer,10))return false;
    memcpy(&p3,buffer,40);
    print_p3();
    return true;
}

uint8_t CONTROL::update_p4(float buffer[]){
    if(check_array(buffer,10))return false;
    memcpy(&p4,buffer,40);
    print_p4();
    return true;
}

uint8_t CONTROL::check_array(float buffer[],uint8_t N){
    float norm=0;
    for(uint8_t i=0;i<N;i++)norm+=fabs(buffer[i]);
    if(norm>eps)return false;
    else return true;
}

void CONTROL::print_p1(){
    coms.send_msgs("Control P1 Set!\n");
    sprintf(msgs,"Zmax = %f\n",p1.Zmax);
    coms.send_msgs();
    sprintf(msgs,"Zmin = %f\n",p1.Zmin);
    coms.send_msgs();
    sprintf(msgs,"LMmax = %f\n",p1.LMmax);
    coms.send_msgs();
    sprintf(msgs,"Nmax = %f\n",p1.Nmax);
    coms.send_msgs();
    sprintf(msgs,"RC to RP = %f\n",p1.RC_to_rp);
    coms.send_msgs();
    sprintf(msgs,"RC to RP Rate = %f\n",p1.RC_to_rp_rates);
    coms.send_msgs();
    sprintf(msgs,"RC to Yaw = %f\n",p1.RC_to_yaw);
    coms.send_msgs();
    sprintf(msgs,"RC to Yaw Rate = %f\n",p1.RC_to_yaw_rates);
    coms.send_msgs();
    sprintf(msgs,"RC to Alt = %f\n",p1.RC_to_alt);
    coms.send_msgs();
    sprintf(msgs,"RC to XY = %f\n",p1.RC_to_xy);
    coms.send_msgs();
}

void CONTROL::print_p2(){
    coms.send_msgs("Control P2 Set!\n");    
    sprintf(msgs,"Neutral Throttle = %f\n",p2.neutral_throttle);
    coms.send_msgs(); 
    sprintf(msgs,"Neutral DeadZone = %f\n",p2.neutral_deadzone);
    coms.send_msgs();
    sprintf(msgs,"Motors Max = %f\n",p2.motors_max);
    coms.send_msgs();
    sprintf(msgs,"Motors Min = %f\n",p2.motors_min);
    coms.send_msgs();
    sprintf(msgs,"Land Velocity = %f\n",p2.land_velocity);
    coms.send_msgs();
    sprintf(msgs,"Ground Threshold = %f\n",p2.ground_threshold);
    coms.send_msgs();
    sprintf(msgs,"Neutral= %f\n",p2.neutral);
    coms.send_msgs();
    sprintf(msgs,"Thrust To Weight= %f\n",p2.thrust_to_weight);
    coms.send_msgs();
    sprintf(msgs,"Alpha min= %f\n",p2.alpha_min_gz);
    coms.send_msgs();
    sprintf(msgs,"Roll_Pitch Lim= %f\n",p2.rp_lim);
    coms.send_msgs();
}

void CONTROL::print_p3(){
    coms.send_msgs("Control P3 Set!\n");
    sprintf(msgs,"Control Yaw = %d\n",p3.control_yaw);
    coms.send_msgs();
}

void CONTROL::print_p4(){
    coms.send_msgs("Control P4 Set!\n");
    sprintf(msgs,"T_min = %f\n",p4.T_min);
    coms.send_msgs();
    sprintf(msgs,"T_max = %f\n",p4.T_max);
    coms.send_msgs();
    sprintf(msgs,"alpha_m = %f \n",p4.alpha_m);
    coms.send_msgs();
}

// X Waypoint
void CONTROL::load_x_waypoint(){
    float parameters[10];
    save_read_param_float(file_name,index_x,parameters,10,0);
    update_x_waypoint(parameters);
}

void CONTROL::save_x_waypoint(float parameters[]){
    if(update_x_waypoint(parameters))save_read_param_float(file_name,index_x,parameters,10,1);
}

uint8_t CONTROL::update_x_waypoint(float parameters[]){

    for(uint8_t i = 0; i < max_mission_waypoints; i++){
        mission_xyz[i][0] = parameters[i];
        cout << "tutti waypoint = " << parameters[i] << endl;
    }
    print_x_waypoint();
}

void CONTROL::print_x_waypoint(){
    coms.send_msgs("X_Waypoint updated!\n");
    // coms.print_matrix(&mission_xyz[0][0],5,1,0);
    // cout << mission_xyz[][0] << endl;
}


// Y Waypoint
void CONTROL::load_y_waypoint(){
    float parameters[10];
    save_read_param_float(file_name,index_y,parameters,10,0);
    update_y_waypoint(parameters);
}

void CONTROL::save_y_waypoint(float parameters[]){
    if(update_y_waypoint(parameters))save_read_param_float(file_name,index_y,parameters,10,1);
}

uint8_t CONTROL::update_y_waypoint(float parameters[]){

    for(uint8_t i = 0; i < max_mission_waypoints; i++){
        mission_xyz[i][1] = parameters[i];
        cout << "mission_xyz[1] = " << mission_xyz[i][1] << endl;
    }
    print_y_waypoint();
}

void CONTROL::print_y_waypoint(){
    coms.send_msgs("Y_Waypoint updated!\n");
    // coms.print_matrix(&mission_xyz[0][1],1,10,0);
}

// Z Waypoint
void CONTROL::load_z_waypoint(){
    float parameters[10];
    save_read_param_float(file_name,index_z,parameters,10,0);
    update_z_waypoint(parameters);
}

void CONTROL::save_z_waypoint(float parameters[]){
    if(update_z_waypoint(parameters))save_read_param_float(file_name,index_z,parameters,10,1);
}

uint8_t CONTROL::update_z_waypoint(float parameters[]){

    for(uint8_t i = 0; i < max_mission_waypoints; i++){
        mission_xyz[i][2] = parameters[i];
        cout << "mission_xyz[2] = " << mission_xyz[i][2] << endl;
    }
    print_z_waypoint();
}

void CONTROL::print_z_waypoint(){
    coms.send_msgs("Z_Waypoint updated!\n");
    // coms.print_matrix(&mission_xyz[0][2],1,10,0);
}
