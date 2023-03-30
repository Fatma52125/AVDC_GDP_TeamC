#include <Definitions.h>

//IP Address and Port Definitions
#define GCS_ADDRESS "192.168.0.219"
#define VICON_ADDRESS "192.168.0.218"
#define BROADCAST_ADDRESS "192.168.0.255"
#define DEF_ADDRESS "192.168.7.1"
#define BROAD_ADDRESS "192.168.7.255"

//Msg buffer
char msgs[100];

//Temporal float array
float temp[10];

//Structure for UAV msg
char uav_header[5]="UAV";
uint8_t enable_uav_stream=0;
struct __attribute__((__packed__)){
    char header[3];
    uint8_t my_id=0;
    float states[6+9];
    float Time;
} uav_out;
struct __attribute__((__packed__)){
    char header[3];
    uint8_t my_id;
    float states[6+9];
    float Time;
} uav_in;
uint8_t uav_msg_size=4+4+(6)*4;

void COMS::begin(uint8_t new_limit){
    //Setting limit of send
    limit=new_limit;
    
    //Initialise UDP
    udp.begin(udp_port);
    
    //Set GCS address
    memset((char *) &GCS_address, 0, sizeof(GCS_address));
	GCS_address.sin_family = AF_INET;
	GCS_address.sin_port = htons(udp_port);
	inet_aton(GCS_ADDRESS , &GCS_address.sin_addr);
    
    //Set VICON address
    memset((char *) &VICON_address, 0, sizeof(VICON_address));
	VICON_address.sin_family = AF_INET;
	VICON_address.sin_port = htons(udp_port);
	inet_aton(VICON_ADDRESS , &VICON_address.sin_addr);
	
	//Set Broadcast address
    memset((char *) &Broadcast_address, 0, sizeof(Broadcast_address));
	Broadcast_address.sin_family = AF_INET;
	Broadcast_address.sin_port = htons(udp_port);
	inet_aton(BROADCAST_ADDRESS , &Broadcast_address.sin_addr);  

    //Set DEF address
    memset((char *) &DEF_address, 0, sizeof(DEF_address));
	DEF_address.sin_family = AF_INET;
	DEF_address.sin_port = htons(udp_port);
	inet_aton(DEF_ADDRESS , &DEF_address.sin_addr);

    //Set BROAD address
    memset((char *) &BROAD_address, 0, sizeof(BROAD_address));
	BROAD_address.sin_family = AF_INET;
	BROAD_address.sin_port = htons(udp_port);
	inet_aton(BROAD_ADDRESS , &BROAD_address.sin_addr);  

    //Initialise USB UART
    uart.begin(uart_bus,uart_baudrate,uart_timeout,0,1,0);
    
    //Short stop
    sleep(1);
}

void COMS::send_data(){  
    //Enable GCS data send
    if(enable_send&&(!disable_udp||!disable_uart)){
        if(counter>=limit){
            //Reset counter
            counter=0;
            timestep++;
            
            //Reset indexes
            index_out=0;    
            chksm_out=0;
    
            //Print header  
            memcpy(buffer_out,header,header_size);
            index_out+=header_size;
            
            //Print variables
            print_variable(&nav_ekf.xyz(0,0),3,100);
            print_variable(&nav_ekf.xyz_vel(0,0),3,100);
            print_variable(&nav_ekf.xyz_acc(0,0),3,100);
            print_variable(&nav_ekf.quat(0,0),4,100);
            print_variable(&nav_ekf.rpy(0,0),3,100);
            print_variable(&nav_ekf.pqr(0,0),3,10);
            print_variable(control.rpy_refs,3,10);
            print_variable(control.xyz_refs,3,10);
            // print_variable(control.rate_refs,3,10);
            print_variable(control.LMNZ,4,1);
            print_variable(receiver.channels,8,1);

            //Copy timestep
            memcpy(buffer_out+index_out,&timestep,4);
            index_out+=4;
    
            //Print checksum            
            uint16_t chksm_out2=calculate_checksum((uint8_t*)(buffer_out+header_size),index_out-header_size);
            print_variable_int((int16_t*)&chksm_out2,1,1);

            //Print terminator
            memcpy(buffer_out+index_out,terminator,terminator_size);
            index_out+=terminator_size;
            
            //Write UDP buffer to GCS
            if(!disable_udp){
                udp.send((uint8_t *)buffer_out, index_out, GCS_address);
                udp.send((uint8_t *)buffer_out, index_out, BROAD_address);
            }
            //Write to UART
            if(!disable_uart)uart.write_bytes((uint8_t *)buffer_out, index_out);
        }else counter++;
    }
}

void COMS::send_uav_data(){
    //Copy header info
    memcpy(uav_out.header,uav_header,3);
    
    //Copy States 
    memcpy(uav_out.states,&model.xyz,12);
    memcpy(uav_out.states+3,&model.xyz_vel,12);
    uav_out.Time=sys.Time;
    
    //Send through UDP
    if(!disable_udp)udp.send((uint8_t *)&uav_out, uav_msg_size, Broadcast_address);

    //Display message
    send_msgs("Sent UAV Data!\n");
}

void COMS::process_uav_msg(char buffer[]){    
    // memcpy(&uav_in,buffer,uav_msg_size);
    // // if(!strncmp(uav_in.header,uav_header,3)){
    // if(!strncmp(uav_in.header,uav_header,3)&&uav_in.my_id!=uav_out.my_id){
    //     //Display Data Received
    //     sprintf(msgs,"Received UAV %d States @ %f:\t",uav_in.my_id,uav_in.Time);
    //     send_msgs();
    //     print_matrix(uav_in.states,1,6,0);
    // }
}

uint16_t COMS::calculate_checksum(uint8_t buf[],uint16_t size){
    uint16_t s1=0;
    uint16_t s2=0;
    for(uint16_t i=0;i<size;i++){
        s1=(s1+buf[i])%255;
        s2=(s2+s1)%255;
    }
    return (s2<<8)|s1;
}

void COMS::print_serial_int(int16_t var){
    memcpy(buffer_out+index_out,&var,2);
    index_out+=2;
}

void COMS::print_variable(float var[],uint8_t size, float scaler){
    int16_t data;
    for(uint8_t i=0;i<size;i++){
        data=(int16_t)(var[i]*scaler);
        print_serial_int(data);
    }
}

void COMS::print_variable_int(int16_t var[],uint8_t size, float scaler){
    int16_t data;
    for(uint8_t i=0;i<size;i++){
        data=(int16_t)(var[i]*scaler);
        print_serial_int(data);
    }
}

void COMS::print_matrix(float mat[],uint8_t n_rows, uint8_t n_cols,uint8_t row_major){
    float value;
    for(uint8_t i=0;i<n_rows;i++){
        for(uint8_t j=0;j<n_cols;j++){
            if(row_major)value=mat[i*n_cols+j];
            else value=mat[j*n_rows+i];
            sprintf(msgs,"%f \t",value);
            send_msgs();
        }
        send_msgs("\n");
    }
}

void COMS::send_msgs(){
    send_msgs(msgs);
}

void COMS::send_msgs(const char msg[]){
    //Get string size
    size_t len = strlen(msg);
    
    //Send message through UDP to GCS
    if(!disable_udp){
        udp.send((uint8_t *)msg,len,GCS_address);
        udp.send((uint8_t *)msg,len,BROAD_address);
    }
    //Send message through UART to GCS
    if(!disable_uart)uart.write_bytes((uint8_t*)msg,len);
    
    //Print message in console
    if(!disable_console)printf("%s",msg);
    
    //Log msg
    if(!datalog.disable_logging)sys.log_msg(msg);
}

uint8_t COMS::any_message_available(void){    
    //Check UDP
    if(udp.read())return true;

    //Check UART
    if(uart.bytes_available()){
        uart.flush();
        return true;
    }
    
    //No message available
    return false;
}

void COMS::receive_data(){
    // failsafe_update();
    if(backoff_timer>0)backoff_timer-=receive_dt;

    //Update Failsafe
    update_vicon_fs();
    update_gcs_fs();

    //Process udp incoming messages   
    process_udp();
}

// void COMS::failsafe_update(){
//     if(sys.Time-vicon_stamp>vicon_time_limit){
        
//         }
    
//     if(sys.Time-gcs_stamp>gcs_time_limit){
        
//         }
// }
void COMS::process_udp(){
    //Read UDP and process if message was received (Needs to be in a for-loop)
    for(uint8_t i=0;i<udp_iter;i++){
        //Read UDP and process if message available
        if(udp.read()){            
            //Get number of bytes read
            uint8_t bytes_read2=(uint8_t)udp.bytes_read;
            
            //Check if message coming from GCS and process as required
            if(udp.client_address.sin_addr.s_addr==GCS_address.sin_addr.s_addr||udp.client_address.sin_addr.s_addr==DEF_address.sin_addr.s_addr){                
                bytes_read2-=header_size;
                uint8_t status=process_msg(udp.buffer_in+header_size,bytes_read2);  
                if(status==1)process_case();  
            }else if(udp.client_address.sin_addr.s_addr==VICON_address.sin_addr.s_addr){

                //Process vicon message
                process_vicon(udp.buffer_in +7);
            
            }else{
                // Check if message coming from UAV
                process_uav_msg(udp.buffer_in);
            }
    
        }else break;            
    }
}

void COMS::process_vicon(char buffer[]){
//Read frame
    uint32_t frame=0;
    float t=0;
    memcpy(&frame,buffer,4);
    if(frame<=prev_frame)return;
    else {
        t=(frame-prev_frame)/120.0f;
        prev_frame=frame;
    }

    //Read number of objects
    uint8_t n_obj=buffer[4];
    if(n_obj>n_obj_max)return;

    //Loop through objects
    uint16_t offset=5;
    uint8_t id=0;
    int16_t xyz[3];
    int16_t rpy[3];
    float value=0;
    // printf("N_objs = %d\n", n_obj);
    for(uint8_t n=0;n<n_obj;n++){
        //My ID
        id=buffer[offset];
        offset++;

        //XYZ
        memcpy(xyz,buffer+offset,6);
        offset+=6;

        //RPY
        memcpy(rpy,buffer+offset,6);
        offset+=6;

        //Check ID
        my_id=0;
        car_id = 1;
        // printf("ID = %d\n",id);
        if(id==my_id){
            for(uint8_t i=0;i<3;i++){
                value=(float)xyz[i]/1000.0f;
                sensors.xyz_vel_meas[i]=(value - sensors.xyz_meas[i])/t;
                sensors.xyz_meas[i]=value;
                value=(float)rpy[i]*0.000174532925199433; //measurement to radians 
                sensors.rpy_meas[i]=value;
            }
            // send_msgs("Vicon Data Received!\n XYZ = ");
            // print_matrix(sensors.xyz_meas,1,3,0);
            sensors.xyz_meas_available=1;
            sensors.xyz_vel_meas_available=1;
            sensors.rpy_meas_available=1;
            reset_vicon_fs();
        }else{
               
        }
        if(id==car_id){
            for(uint8_t i=0;i<3;i++){
                value=(float)xyz[i]/1000.0f;
                // control.xyz_vel_meas[i]=(value - sensors.xyz_meas[i])/t;
                control.car_xyz[i]=value;
                // value=(float)rpy[i]*0.000174532925199433; //measurement to radians 
                // sensors.rpy_meas[i]=value;
            }
            // send_msgs("Vicon Data Received!\n XYZ = ");
            // print_matrix(sensors.xyz_meas,1,3,0);
            control.car_xyz_available=1;
            car_stamp = sys.Time;
            
            // sprintf(msgs,"Car_x = %f, Car_y = %f,Car_z = %f \n",control.car_xyz[0],control.car_xyz[1],control.car_xyz[2]);
            // coms.send_msgs();
    
            // reset_vicon_fs();
        }else{
               
        }
    }
}

uint8_t COMS::process_msg(char buffer_in1[],uint8_t &index_in1){
    //Fill structure
    memcpy(&msg_in,buffer_in1,income_total_size);

    //Check msg ids match
    if(msg_in.msg_id1==msg_in.msg_id2&&msg_in.msg_id1==msg_in.msg_id3){
        //Special message type
        if(msg_in.msg_type==2){
            return true;
        }

        //Process checksum
        uint16_t chksm_in=calculate_checksum((uint8_t*)msg_in.int_buffer_in,40);
        
        //Accept message if checksum matches
        if(chksm_in==msg_in.chksm1&&chksm_in==msg_in.chksm2){
            if(msg_in.msg_type==0)memcpy(float_buffer_in,msg_in.int_buffer_in,float_buffer_in_size);
            reset_gcs_fs();
            return true;
        }
    }
    return false;
}

void COMS::process_case(){
    if(!sys.armed){
        uint8_t defer_wait=0;
        switch(msg_in.msg_id1){
            //Acknowledgement
            case 0:
            send_msgs("Acknowledged!\n");
            sys.sync_time_request=1;
            defer_wait=1;
            break;

            //Enable/disable model override
            case 23:
            model.set_override((uint8_t)msg_in.int_buffer_in[0]);  
            break;

            //Calibrate gyro
            case 50: 
            sensors.calibrate_gyro(200);
            break;

            //Calibrate level 
            case 51:
            sensors.calibrate_level(200);
            break;

            //Calibrate accel
            case 52:
            sensors.calibrate_accel(200);
            break;
            
            //Update roll_rate_pid Gains
            case 60:
            control.roll_rate_pid.save_parameters(float_buffer_in);
            break;

            //Update pitch_rate_pid Gains
            case 61:
            control.pitch_rate_pid.save_parameters(float_buffer_in);
            break;

            //Update yaw_rate_pid Gains
            case 62:
            control.yaw_rate_pid.save_parameters(float_buffer_in);
            break;

            //Update roll_pid Gains
            case 63:
            control.roll_pid.save_parameters(float_buffer_in);
            break;

            //Update pitch_pid Gains
            case 64:
            control.pitch_pid.save_parameters(float_buffer_in);
            break;

            //Update yaw_pid Gains
            case 65:
            control.yaw_pid.save_parameters(float_buffer_in);
            break;

            //Update x_pid Gains
            case 66:
            control.x_pid.save_parameters(float_buffer_in);
            break;

            //Update y_pid Gains
            case 67:
            control.y_pid.save_parameters(float_buffer_in);
            break;

            //Update z_pid Gains
            case 68:
            control.z_pid.save_parameters(float_buffer_in);
            break;

            //Update z_acc_pid Gains
            case 69:
            control.z_acc_pid.save_parameters(float_buffer_in);
            break;

            //Update model parameters
            case 70:
            model.save_parameters(float_buffer_in);
            break;

            //Update z_pid Gains
            case 71:
            control.x_vel_pid.save_parameters(float_buffer_in);
            break;

            //Update z_pid Gains
            case 72:
            control.y_vel_pid.save_parameters(float_buffer_in);
            break;

            //Update Qatt
            case 80:
            nav_ekf.save_Qatt(float_buffer_in);
            break;

            //Update Ratt
            case 81:
            nav_ekf.save_Ratt(float_buffer_in);
            break;

            //Update Tpqr
            case 82:
            nav_ekf.save_Tpqr(float_buffer_in);
            break;

            //Update Qxyz
            case 83:
            nav_ekf.save_Qxyz(float_buffer_in);
            break;

            //Update Rxyz
            case 84:
            nav_ekf.save_Rxyz(float_buffer_in);
            break;

            //Update Tacc
            case 85:
            nav_ekf.save_Tacc(float_buffer_in);
            break;

            //Reset Nav EKF
            case 86:
            nav_ekf.reset_att();
            nav_ekf.reset_xyz();
            break;

            //Update Qz
            case 87:
            nav_ekf.save_Qz(float_buffer_in);
            break; 

            //Update control p1 parameters
            case 90:
            control.save_p1(float_buffer_in);
            break;

            //Update control p2 parameters
            case 91:
            control.save_p2(float_buffer_in);
            break;

            //Update control p3 parameters
            case 92:
            control.save_p3(float_buffer_in);
            break;

            //Update control p4 parameters
            case 93:
            control.save_p4(float_buffer_in);
            break;

            //Send UAV message
            case 94:
            send_uav_data();
            break;

            //Update ID
            case 95:
            uav_out.my_id=(uint8_t)round(float_buffer_in[0]);
            sprintf(msgs,"ID %d Updated!\n",uav_out.my_id);
            send_msgs();
            break;

            //Update mission waypoint
            case 250:
            control.save_x_waypoint(float_buffer_in);
            break;

            case 251:
            control.save_y_waypoint(float_buffer_in);
            break;

            case 252:
            control.save_z_waypoint(float_buffer_in);
            break;
            
            //Default case (do nothing)
            default:
            defer_wait=1;
            break;
        }
    
        //Wait for 5 seconds to see report of results
        if(backoff_timer<receive_dt){
            backoff_timer=backoff_time;
            if(defer_wait)defer_wait=0;
            // else sys.wait(5);
        }
    }    
    
    //General messages
    switch(msg_in.msg_id1){
        //Disarm 
        case 21: 
        sys.disarm(Coms);
        break;
        
        //Arm 
        case 22:
        receiver.force_disarm=0;
        sys.arm(Coms);
        break;

        //Take Off
        case 30:
        if(control.flying_state==Idle){
            memcpy(temp,&nav_ekf.xyz,12);
            if(float_buffer_in[0]<1)temp[2]+=1;
            else temp[2]+=float_buffer_in[0];
            if(control.set_xyz_refs(temp))control.set_flying_state(Flying);
        }
        break;

        //Set mode to Stabilise
        case 100:
        control.set_mode(Stabilise,Coms);
        break;
    
        //Set mode to Acro
        case 101:
        control.set_mode(Acro,Coms);
        break;
    
        //Set mode to alt_hold
        case 102:
        control.set_mode(Alt_Hold,Coms);
        break;

        //Set mode to pos hold
        case 103:
        control.set_mode(Pos_Hold,Coms);
        break;
    
        //Set mode to guided
        case 104:
        control.set_mode(Guided,Coms);
        break;

        //Set mode to land
        case 105:
        control.set_mode(Land,Coms);
        break;

        //Set mode to mission
        case 106:
        control.set_mode(Mission,Coms);
        break;

        //Set mode to tracking
        case 107:
        control.set_mode(Track,Coms);
        break;

        //Set mode to rtl
        case 108:
        control.set_mode(RTL,Coms);
        break;

        // //Process vicon
        // case 143
        // process_vicon(udp.buffer_in+7);
        // break;  

        //Update xyz reference
        case 150:
        control.set_xyz_refs(float_buffer_in);
        break;

        //Update wind
        case 151:
        model.update_wind(float_buffer_in);
        break;
        
        //Update xyz reference
        case 152:
        control.set_yaw_ref(float_buffer_in);
        break;

        //Default case (do nothing)
        default:
        break;
    }
}

void COMS::update_vicon_fs(){
    float period=sys.Time-vicon_fs_stamp;
    if(period>vicon_fs_limit&&!vicon_fs_triggered){
        send_msgs("VICON Lost!\n");
        sprintf(msgs,"Time VICON Lost %f \n",period);
        coms.send_msgs();
        vicon_active=0;
        vicon_fs_triggered=1;
        sys.trigger_failsafe(vicon_fs);
    }
}


void COMS::reset_vicon_fs(){
    vicon_fs_triggered=0;
    vicon_fs_stamp=sys.Time;
    if(!vicon_active){
        vicon_active=1;
        send_msgs("VICON Connected!\n");
    }
    
}

void COMS::update_gcs_fs(){
    float period=sys.Time-gcs_fs_stamp;
    if(period>gcs_fs_limit&&!gcs_fs_triggered){
        gcs_active=0;
        gcs_fs_triggered=1;
        send_msgs("GCS Lost!\n");
        sprintf(msgs,"Time GCS Lost %f \n",period);
        coms.send_msgs();
        sys.trigger_failsafe(gcs_fs);        
    }
}

void COMS::reset_gcs_fs(){
    gcs_fs_triggered=0;
    gcs_fs_stamp=sys.Time;
    if(!gcs_active){
        gcs_active=1;
        send_msgs("GCS Connected!\n");
    }
}