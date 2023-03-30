#include <Definitions.h>

void SENSORS::begin(uint8_t new_limit){
    coms.send_msgs("Initializing SENSORS..\n");   
    limit=new_limit;
    
    //Load parameters
    // load_parameters();
    
    //Initialise IMU
    conf = rc_mpu_default_config();
    conf.enable_magnetometer=enable_magnetometer;
    conf.i2c_bus=2;
    conf.accel_dlpf=ACCEL_DLPF_5;
    conf.gyro_dlpf=GYRO_DLPF_10;
    // conf.gyro_dlpf=GYRO_DLPF_20;
    coms.send_msgs("Initializing IMU...\n");
    if(rc_mpu_initialize(&imu, conf)){
        coms.send_msgs("rc_mpu_initialize failed\n");
    }
    calibrate_gyro(200);
    load_or_save_accel_cal(0);
    load_or_save_mag_cal(0);
    load_or_save_mag_cal2(0);
    coms.send_msgs("IMU Initialized!\n");
    
    //Initialise barometer
    coms.send_msgs("Initializing barometer...\n");
    if(rc_bmp_init(BMP_OVERSAMPLE_16,BMP_FILTER_8)<0){
        coms.send_msgs("rc_bmp_init failed\n");
    }
    calibrate_baro();               //Calibrate barometer zero
    coms.send_msgs("Barometer Initialized!\n");
    
    //Initialise GPS
    // gps.begin(50);
    
    coms.send_msgs("SENSORS Initialized!\n");
}


void SENSORS::read(){
    //Read IMU(One-Shot)
    rc_mpu_read_gyro(&imu);
    rc_mpu_read_accel(&imu);
    if(enable_magnetometer)rc_mpu_read_mag(&imu);
    
    //Store IMU data
    for(uint8_t i=0;i<3;i++){
        gyro[i]=(float)imu.gyro[i]-gyro_offsets[i];
        accel[i]=(float)accel_scales[i]*(imu.accel[i]-accel_offsets[i]);
        if(enable_magnetometer)mag[i]=(float)imu.mag[i];
    }
    if(enable_magnetometer)Correct_DataPoint(mag,mag_offsets2,mag_scales2);
    
    //Correct Orientation
    correct_orientation();                      //Correct orientation
}

void SENSORS::read_extra(){
    //Read barometer
    rc_bmp_read(&baro);
    baro_alt=baro.alt_m-baro_ref;

    //Read LiDAR rangefinder
    
    //Read GPS
    // gps.update(0);
    
    //Read airspeed sensor

}

void SENSORS::update(uint8_t flag){
    //Return if model override enable
    if(model.override)return;

    //Read measurements (including orientation corrections)
    read();                                     //Read sensors available with predictable timing (IMU)
    
    //Set measurements (apply filters)
    nav_ekf.set_pqr(gyro,alpha_pqr);
    nav_ekf.set_acc(accel,alpha_acc);
    if(rpy_meas_available)nav_ekf.set_rpy_meas(rpy_meas,alpha_rpy_meas);
    if(enable_magnetometer)nav_ekf.set_mag(mag,alpha_mag);
    
    //Propagate attitude
    nav_ekf.propagate_att();               //Run Att_EKF propagation step
    
    // //Apply corrections
    nav_ekf.correct_acc();
    if(rpy_meas_available)nav_ekf.correct_rpy();
    if(enable_magnetometer)nav_ekf.correct_mag();
    // cout << "pos" << nav_ekf.rpy << endl;

    //Apply corrections
    // if(coms.vicon_active){
    //     nav_ekf.correct_acc();
    //     if(rpy_meas_available)nav_ekf.correct_rpy();                   //Correct with VICON for rpy
    // }else{
    //     nav_ekf.correct_acc();
    //     if(enable_magnetometer)nav_ekf.correct_mag();                //Correct with magnetometer if desired
    // }    
    
    //Propagate xyz
    if(coms.vicon_active){    //||gps.active){
        nav_ekf.propagate_xyz();
    }
    else nav_ekf.propagate_z();
    
    //Check Att_EKF outputs for "NaN"
    if(isnan(nav_ekf.quat(0,0)))sys.disarm(Sensors);
    // else nav_ekf.get_rpy();
    
    //Check Pos_EKF outputs for "NaN"
    if(isnan(nav_ekf.xyz(2,0)))sys.disarm(Sensors);
    
    //Print if requested
    print(flag);
    
    //Reset RYP meas flag
    rpy_meas_available=0;
}

void SENSORS::update_extra(uint8_t flag){
    //Return if model override enable
    if(model.override)return;
    //Read extra sensors
    read_extra();
    
    //Set measurements
    if(xyz_meas_available)nav_ekf.set_xyz_meas(xyz_meas,alpha_xyz_meas);
    if(xyz_vel_meas_available)nav_ekf.set_xyz_vel_meas(xyz_vel_meas,alpha_xyz_meas);
    nav_ekf.set_z_meas(&baro_alt,alpha_z);

    //Run Pos_EKF steps
    if(coms.vicon_active){
        if(xyz_meas_available){
            nav_ekf.xyz_lock=1;
            nav_ekf.correct_xyz();
            // baro_ref-=(1-alpha_baro_ref)*(xyz_meas[2]-baro_alt);        //Tracking xyz_meas zero
            baro_ref-=(1)*(xyz_meas[2]-baro_alt);
        }
        if(xyz_vel_meas_available)nav_ekf.correct_xyz_vel();
        // else nav_ekf.correct_z();
    }else nav_ekf.correct_z();
    // if(xyz_meas_available){
    //     nav_ekf.xyz_lock=1;
    //     nav_ekf.correct_xyz();
    //     // cout << "pos" << nav_ekf.xyz << endl;
    // }
    if(xyz_vel_meas_available)nav_ekf.correct_xyz_vel();
    
    //Check covariances
    nav_ekf.check_att_covariance();
    nav_ekf.check_xyz_covariance();
    
    //Print if requested
    print_extra(flag);
    
    //Reset xyz_meas_available
    xyz_meas_available=0;
    xyz_vel_meas_available=0;
}

void SENSORS::print(uint8_t flag){
    if(flag){
        if(counter>=limit){
            counter=0;
            switch(flag){  
                case 1:
                print_rpy();
                break;

                case 2:
                print_pqr();
                break;

                case 3:
                print_acc();
                break;

                case 4:
                print_mag();
                break;

                case 5:
                print_rpy();
                print_pqr();
                break;

                case 6:
                print_rpy();
                print_pqr();
                print_acc();
                break;

                case 7:
                print_rpy();
                print_pqr();
                print_acc();
                print_mag();
                break;

                default:
                break; 
            }
        }else counter++;
    }else counter=0;
}


void SENSORS::print_rpy(){
    coms.send_msgs("RPY = \t");
    coms.print_matrix(&nav_ekf.rpy(0,0),1,3,0);
}

void SENSORS::print_pqr(){
    coms.send_msgs("PQR = \t");
    coms.print_matrix(&nav_ekf.pqr(0,0),1,3,0);
}

void SENSORS::print_acc(){
    coms.send_msgs("Acc = \t");
    coms.print_matrix(&nav_ekf.acc(0,0),1,3,0);
}

void SENSORS::print_mag(){
    coms.send_msgs("Mag = \t");
    coms.print_matrix(&nav_ekf.mag(0,0),1,3,0);
}

void SENSORS::print_extra(uint8_t flag){
    if(flag){
        if(counter_extra>=limit){
            counter_extra=0;
            switch(flag){
                case 1:
                print_baro();
                break;

                //Default
                default:
                break;
            }
        }else counter_extra++;
    }else counter_extra=0;
}

void SENSORS::print_baro(){
    coms.send_msgs("Baro alt = \t");
    coms.print_matrix(&baro_alt,1,1,0);
}

void SENSORS::calibrate_baro(){
    baro_ref=0;
    int N=100;
    for(uint8_t i=0;i<N;i++){
        rc_bmp_read(&baro);
        baro_ref+=baro.alt_m;
        rc_usleep(20000);
    }
    baro_ref=baro_ref/(float)N;
}

void SENSORS::correct_orientation(){
    //Rotate gyro in degs
    temp[0]=-gyro[1];
    temp[1]=-gyro[0];
    temp[2]=-gyro[2];
    for(uint8_t i=0;i<3;i++)gyro[i]=temp[i];
    
    //Rotate accel in m/s^2
    temp[0]=accel[1];
    temp[1]=accel[0];
    temp[2]=accel[2];
    for(uint8_t i=0;i<3;i++)accel[i]=temp[i];
    
    //Rotate mag in uT
    if(enable_magnetometer){
        temp[0]=-mag[1];
        temp[1]=-mag[0];
        temp[2]=-mag[2];
        for(uint8_t i=0;i<3;i++)mag[i]=temp[i];
    }
}

void SENSORS::calibrate_gyro(uint8_t N){
    uint8_t i,j;
    
    gyro_cal=0;
    while(!gyro_cal){
        //Calibrating message
        coms.send_msgs("Calibrating gyro...\n");
        
        //Reset gyro offsets
        for(i=0;i<3;i++)gyro_offsets[i]=0;
        
        //Take N samples to calibrate
        for(j=0;j<N;j++){
            rc_mpu_read_gyro(&imu);
            for(i=0;i<3;i++)gyro_offsets[i]+=(float)imu.gyro[i];
            rc_usleep(20000);
        }
        
        //Divide over N samples
        for(i=0;i<3;i++)gyro_offsets[i]/=(float)N;
        gyro_cal=1;
        
        //Check gyro calibration
        for(i=0;i<3;i++){
            if(fabs(gyro_offsets[i])>5)gyro_cal=0;
        }
    }
    
    //Termination message
    coms.send_msgs("Gyro calibrated!\n");
}

void SENSORS::calibrate_level(uint8_t N){
    //Calibrating message
    coms.send_msgs("Calibrating level...\n");

    //Run test
    float bias[3]={0,0,0};
    for(uint8_t n=0;n<N;n++){
        rc_mpu_read_accel(&imu);
        for(uint8_t i=0;i<3;i++)bias[i]+=(float)accel_scales[i]*(imu.accel[i]-accel_offsets[i]);
        rc_usleep(20000);
    }
    
    //Calculate new bias
    for(uint8_t i=0;i<3;i++){
        bias[i]/=(float)N;
        if(i==2)bias[i]-=g0;
        accel_offsets[i]+=bias[i]/accel_scales[i];
    }    
    
    //Save
    load_or_save_accel_cal(1);

    //Termination message
    coms.send_msgs("Level Calibrated!\n");
}

void SENSORS::calibrate_accel(uint8_t N){
    coms.send_msgs("Starting accel calibration...\n");
    
    //Index for bin file
    uint16_t index=0;
    
    //Reset accel offsets and scales and set max mins
    float max[3];
    float min[3];
    for(uint8_t i=0;i<3;i++){
        accel_offsets[i]=0;
        accel_scales[i]=1;
        max[i]=0;
        min[i]=0;
    }
    
    //Prompt user inputs
    coms.send_msgs("Place your vehicle level, and give input...\n");
    while(!coms.any_message_available())usleep(20000);
    index=run_accel_test(max,min,N,index);
    
    coms.send_msgs("Upside down. Give input...\n");
    while(!coms.any_message_available())usleep(20000);
    index=run_accel_test(max,min,N,index);
    
    coms.send_msgs("Right side. Give input...\n");
    while(!coms.any_message_available())usleep(20000);
    index=run_accel_test(max,min,N,index);
    
    coms.send_msgs("Left side. Give input...\n");
    while(!coms.any_message_available())usleep(20000);
    index=run_accel_test(max,min,N,index);
    
    coms.send_msgs("Front side. Give input...\n");
    while(!coms.any_message_available())usleep(20000);
    index=run_accel_test(max,min,N,index);
    
    coms.send_msgs("Back side. Give input...\n");
    while(!coms.any_message_available())usleep(20000);
    run_accel_test(max,min,N,index);
    
    //Average samples
    for(uint8_t i=0;i<3;i++){
        max[i]/=(float)N;
        min[i]/=(float)N;
    }
    
    //Calculate accel calibration
    float deltas,g0=9.80665;
    for(uint8_t i=0;i<3;i++){
        accel_offsets[i]=(max[i]+min[i])/2.0f;
        deltas=(max[i]-min[i])/2.0f;
        accel_scales[i]=g0/deltas;
    }
    
    //Save
    load_or_save_accel_cal(1);
}

uint16_t SENSORS::run_accel_test(float max[],float min[],uint8_t N,uint16_t index){
    float min_g=7;
    for(uint8_t n=0;n<N;n++){
		rc_mpu_read_accel(&imu);
		for(uint8_t i=0;i<3;i++){
		    if(imu.accel[i]>min_g)max[i]+=imu.accel[i];
		    if(imu.accel[i]<-min_g)min[i]+=imu.accel[i];
		    accel[i]=(float)imu.accel[i];
		}
        save_read_param_float(acc_cal_name,index,accel,3,1);
        index+=12;
		usleep(20000);
    }
    return index;
}

void SENSORS::calibrate_mag(){
    coms.send_msgs("Starting mag calibration...\n");
    //Index for bin file
    uint16_t index=0;
    
    //Reset mag offsets and scales and set max mins
    float max[3];
    float min[3];
    for(uint8_t i=0;i<3;i++){
        mag_offsets[i]=0;
        mag_scales[i]=1;
        max[i]=-10000;
        min[i]=10000;
    }
    
    //Prompt user inputs
    coms.send_msgs("Move board in all orientations and give input when ready.\n");
    while(!coms.any_message_available()){
		rc_mpu_read_mag(&imu);
        for(uint8_t i=0;i<3;i++){
            if(imu.mag[i]>max[i])max[i]=imu.mag[i];
            if(imu.mag[i]<min[i])min[i]=imu.mag[i];
            mag[i]=(float)imu.mag[i];
        }
        save_read_param_float(mag_cal_name,index,mag,3,1);
        index+=12;
        usleep(20000);
    }
    
    //Calculate offsets
    float deltas[3];
    for(uint8_t i=0;i<3;i++){
        mag_offsets[i]=(max[i]+min[i])/2.0f;
        deltas[i]=(max[i]-min[i])/2.0f;
    }
    
    //Calculate avg delta
    float avg_delta=0;
    for(uint8_t i=0;i<3;i++)avg_delta+=deltas[i];
    avg_delta=avg_delta/(float)3.0f;
    
    //Get scales
    for(uint8_t i=0;i<3;i++)mag_scales[i]=avg_delta/deltas[i];
    
    //Save calibration
    load_or_save_mag_cal(1);
}

void SENSORS::calibrate_mag2(){
    coms.send_msgs("Starting mag calibration...\n");
    
    //Index for bin file
    uint16_t index=0;
    
    //Reset mag offsets and scales and set max mins
    for(uint8_t i=0;i<3;i++)mag_offsets2[i]=0;
    for(uint8_t i=0;i<9;i++)mag_scales2[i]=0;
    
    //Initialise fitting variables
    float P_k[81],Theta_k[9];
    Recursive_Ellipsoid_Fit(P_k,Theta_k,mag,1,1);
    
    //Prompt user inputs
    coms.send_msgs("Move board in all orientations and give input when ready.\n");
    uint8_t cal_counter=0;
    uint8_t cal_limit=10;

    while(!coms.any_message_available()){
		rc_mpu_read_mag(&imu);
		for(uint8_t i=0;i<3;i++)mag[i]=(float)imu.mag[i];
        save_read_param_float(mag_cal_name,index,mag,3,1);
        index+=12;
        Recursive_Ellipsoid_Fit(P_k,Theta_k,mag,0,0);
        
        if(cal_counter>cal_limit){
            cal_counter=0;
            coms.send_msgs("Theta_k = \t");
            coms.print_matrix(Theta_k,1,9,0);
        }
        cal_counter++;
        usleep(20000);
    }
    
    //Calculate mag calibration
    coms.send_msgs("Calculating fit...\n");
    Calculate_Fit(Theta_k,100,mag_offsets2,mag_scales2);
    coms.send_msgs("Offsets = \t");
    coms.print_matrix(mag_offsets2,1,3,0);
    coms.send_msgs("Rotation = \t");
    coms.print_matrix(mag_scales2,3,3,0);
    sleep(1);
    
    //Save
    coms.send_msgs("Storing calibration...\n");
    load_or_save_mag_cal2(1);
}

uint8_t SENSORS::check_accel_cal(){
    //Check values
    uint8_t flag=0;
    for(uint8_t i=0;i<3;i++){
        if(fabs(accel_offsets[i])>1||accel_offsets[i]==0)flag=1;
        if(fabs(accel_scales[i]-1)>0.2)flag=1;
    }
    
    //Reset if flag positive
    if(flag){
        coms.send_msgs("Accelerometer calibration problem!\n");
        for(uint8_t i=0;i<3;i++){
            accel_offsets[i]=0;
            accel_scales[i]=1;
        }
        accel_cal=0;
    }else accel_cal=1;
    
    //Print accelerometer calibration
    print_accel_cal();
    
    return flag;
}

uint8_t SENSORS::check_mag_cal(){
    //Check values
    uint8_t flag=0;
    for(uint8_t i=0;i<3;i++){
        if(fabs(mag_offsets[i])>200||mag_offsets[i]==0)flag=1;
        if(fabs(mag_scales[i]-1)>0.5||mag_scales[i]==1)flag=1;
    }
    
    //Reset if flag positive
    if(flag){
        coms.send_msgs("Magnetometer calibration problem!\n");
        for(uint8_t i=0;i<3;i++){
            mag_offsets[i]=0;
            mag_scales[i]=1;
        }
        mag_cal=0;
    }else mag_cal=1;
    
    //Print magnetometer calibration
    print_mag_cal();
    
    return flag;
}

uint8_t SENSORS::check_mag_cal2(){
    //Check values
    uint8_t flag=0;
    for(uint8_t i=0;i<3;i++){
        if(fabs(mag_offsets2[i])>200||mag_offsets2[i]==0)flag=1;
        // if(fabs(mag_scales2[3*i+i])<0.1)flag=1;                     //Check diagonal
    }
    
    //Reset if flag positive
    flag=1;
    if(flag){
        coms.send_msgs("Magnetometer calibration problem!\n");
        for(uint8_t i=0;i<3;i++)mag_offsets2[i]=0;
        for(uint8_t i=0;i<9;i++)mag_scales2[i]=0;
        for(uint8_t i=0;i<3;i++)mag_scales2[3*i+i]=1;
        mag_cal=0;
    }else mag_cal=1;

    //Print scales2
    coms.send_msgs("Mag Scales 2 = \t");
    coms.print_matrix(mag_scales2,1,9,0);
    
    //Print magnetometer calibration
    print_mag_cal2();
    
    return flag;
}

void SENSORS::load_or_save_accel_cal(uint8_t save){
    float parameters[6];
    if(!save){
        save_read_param_float(file_name,accel_index,parameters,6,0);
        memcpy(accel_offsets,parameters,sizeof(accel_offsets));
        memcpy(accel_scales,parameters+3,sizeof(accel_scales));
    }
    uint8_t flag=check_accel_cal();
    if(flag||save){
        memcpy(parameters,accel_offsets,sizeof(accel_offsets));
        memcpy(parameters+3,accel_scales,sizeof(accel_scales));
        save_read_param_float(file_name,accel_index,parameters,6,1);
    }
}

void SENSORS::load_or_save_mag_cal(uint8_t save){
    float parameters[6];
    if(!save){
        save_read_param_float(file_name,mag_index,parameters,6,0);
        memcpy(mag_offsets,parameters,sizeof(mag_offsets));
        memcpy(mag_scales,parameters+3,sizeof(mag_scales));
    }
    uint8_t flag=check_mag_cal();
    if(flag||save){
        memcpy(parameters,mag_offsets,sizeof(mag_offsets));
        memcpy(parameters+3,mag_scales,sizeof(mag_scales));
        save_read_param_float(file_name,mag_index,parameters,6,1);
    }
}

void SENSORS::load_or_save_mag_cal2(uint8_t save){
    float parameters[12];
    if(!save){
        save_read_param_float(file_name,mag2_index,parameters,12,0);
        memcpy(mag_offsets2,parameters,sizeof(mag_offsets2));
        memcpy(mag_scales2,parameters+3,sizeof(mag_scales2));
    }
    uint8_t flag=check_mag_cal2();
    if(flag||save){
        memcpy(parameters,mag_offsets2,sizeof(mag_offsets2));
        memcpy(parameters+3,mag_scales2,sizeof(mag_scales2));
        save_read_param_float(file_name,mag2_index,parameters,12,1);
    }
}

void SENSORS::print_accel_cal(){
    coms.send_msgs("Accel Calibration..\n");
    coms.send_msgs("Scale | Offset\n");
    for(uint8_t i=0;i<3;i++){
        sprintf(msgs,"%f | %f\n",accel_scales[i],accel_offsets[i]);
        coms.send_msgs();
    }
}

void SENSORS::print_mag_cal(){
    coms.send_msgs("Mag Calibration..\n");
    coms.send_msgs("Scale | Offset\n");
    for(uint8_t i=0;i<3;i++){
        sprintf(msgs,"%f | %f\n",mag_scales[i],mag_offsets[i]);
        coms.send_msgs();
    }
}

void SENSORS::print_mag_cal2(){
    coms.send_msgs("Mag Calibration..\n");
    coms.send_msgs("Rotation\n");
    for(uint8_t i=0;i<9;i++){
        sprintf(msgs,"%f \n",mag_scales2[i]);
        coms.send_msgs();
    }
    coms.send_msgs("Offsets\n");
    for(uint8_t i=0;i<3;i++){
        sprintf(msgs,"%f \n",mag_offsets2[i]);
        coms.send_msgs();
    }
}

void SENSORS::load_parameters(){
    float parameters[10];
    save_read_param_float(file_name,p1_index,parameters,10,0);
    update_parameters(parameters);
}

void SENSORS::save_parameters(float parameters[]){
    uint8_t flag=update_parameters(parameters);
    if(flag)save_read_param_float(file_name,p1_index,parameters,10,1);
}

uint8_t SENSORS::update_parameters(float parameters[]){
    Map<Matrix<float,10,1>>pars(parameters);
    if(pars.norm()<eps||pars.norm()>10000){
        coms.send_msgs("SENSORS: Problem updating parameters!\n");
        return false;
    }
    alpha_pqr=parameters[0];
    alpha_pqr_dot=parameters[1];
    alpha_acc=parameters[2];
    alpha_mag=parameters[3];
    alpha_rpy_meas=parameters[4];
    alpha_z=parameters[5];
    alpha_z_ref=parameters[6];
    alpha_xyz_meas=parameters[7];
    enable_magnetometer=(uint8_t)round(parameters[8]);
    magnetic_declination=parameters[9];
    
    print_parameters();
    return true;
}

void SENSORS::print_parameters(){
    sprintf(msgs,"Alpha PQR = %.1f\n",alpha_pqr);
    coms.send_msgs();
    sprintf(msgs,"Alpha PQR Dot = %.1f\n",alpha_pqr_dot);
    coms.send_msgs();
    sprintf(msgs,"Alpha Acc = %.1f\n",alpha_acc);
    coms.send_msgs();
    sprintf(msgs,"Alpha Mag = %.1f\n",alpha_mag);
    coms.send_msgs();
    sprintf(msgs,"Alpha RPY Meas = %.1f\n",alpha_rpy_meas);
    coms.send_msgs();
    sprintf(msgs,"Alpha Z = %.1f\n",alpha_z);
    coms.send_msgs();
    sprintf(msgs,"Alpha Z Ref = %.1f\n",alpha_z_ref);
    coms.send_msgs();
    sprintf(msgs,"Alpha XYZ Meas = %.1f\n",alpha_xyz_meas);
    coms.send_msgs();
    sprintf(msgs,"Enable Magnetometer = %d\n",enable_magnetometer);
    coms.send_msgs();
    sprintf(msgs,"Magnetic Declination = %.1f\n",magnetic_declination);
    coms.send_msgs();
}